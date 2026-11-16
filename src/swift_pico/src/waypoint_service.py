#!/usr/bin/env python3

"""
Task 3A: Autonomous Waypoint Navigation Service with Vision Perception, Obstacle Avoidance, & Dynamic Re-planning
Team: KD_6479

Workflow:
1. Subscribes to /image_raw once with Transient Local / Reliable QoS.
2. Rectifies arena using ArUco boundary markers (80, 85, 90, 95) to 900x900 pixels.
3. Detects:
   - 3 Critical Survivors (red triangles)
   - 3 Stable Survivors (yellow circles)
   - 3 Obstacles (black extruded polygons)
4. Expands obstacles into off-limits keep-out zones with safety clearance.
5. Plans globally optimal critical route from F6 through the 3 critical survivors avoiding obstacles using A* and line-of-sight shortcutting.
6. Serves the critical route on /waypoints (GetWaypoints).
7. Serves dynamic re-plan requests on /replan_waypoints (ReplanRoute) to plan from wherever the drone is at the Switch through the 3 stable survivors and back to F6.
8. Generates checkpoint visualization image matching task specification.
"""

import os
import sys
import math
import itertools
import threading
import json
import csv
import heapq
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, Point
from waypoint_navigation.srv import GetWaypoints, ReplanRoute
from ament_index_python.packages import get_package_share_directory


# Affine mapping constants between 900x900 pixel space and metric WhyCode space
# wx = M_A[0] * px + M_A[1] * py + M_B[0]
# wy = M_A[2] * px + M_A[3] * py + M_B[1]
AFFINE_PX_TO_WX = -0.01620
AFFINE_PY_TO_WY = -0.01621
AFFINE_BX = 7.2964
AFFINE_BY = 7.3006


def pixel_to_whycode(px, py):
    """Converts 900x900 pixel coordinates to metric WhyCode coordinates."""
    wx = AFFINE_BX + AFFINE_PX_TO_WX * px
    wy = AFFINE_BY + AFFINE_PY_TO_WY * py
    return float(wx), float(wy)


def whycode_to_pixel(wx, wy):
    """Converts metric WhyCode coordinates to 900x900 pixel coordinates."""
    px = (wx - AFFINE_BX) / AFFINE_PX_TO_WX
    py = (wy - AFFINE_BY) / AFFINE_PY_TO_WY
    return float(px), float(py)


def detect_aruco_markers(image):
    """Detects ArUco markers using DICT_4X4_250 dictionary."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if hasattr(cv2, 'aruco') and hasattr(cv2.aruco, 'ArucoDetector'):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
    return corners, ids


def get_arena_boundary_points(corners, ids):
    """
    Extracts the 4 inner corners of the arena boundary ArUco markers:
    Top-Left: 80, Top-Right: 85, Bottom-Right: 90, Bottom-Left: 95.
    """
    required_ids = {80, 85, 90, 95}
    if ids is None or len(ids) == 0:
        raise ValueError("No ArUco markers detected in camera frame.")

    detected_ids = [int(x) for x in ids.flatten()]
    missing = required_ids - set(detected_ids)
    if missing:
        raise ValueError(f"Missing required boundary marker IDs: {missing}")

    marker_dict = {}
    for i, mid in enumerate(detected_ids):
        if mid in required_ids:
            marker_dict[mid] = corners[i][0]

    all_pts = np.vstack([marker_dict[mid] for mid in required_ids])
    arena_center = np.mean(all_pts, axis=0)

    inner_corners = {}
    for mid in required_ids:
        pts = marker_dict[mid]
        dists = np.linalg.norm(pts - arena_center, axis=1)
        inner_corners[mid] = pts[np.argmin(dists)]

    src_pts = np.array([
        inner_corners[80],
        inner_corners[85],
        inner_corners[90],
        inner_corners[95]
    ], dtype=np.float32)

    return src_pts


def map_pixel_to_intersection(x, y):
    """Snaps pixel coordinate on 900x900 board to the nearest grid intersection (A1 to K11)."""
    col_idx = int(round(x / 75.0))
    row_idx = int(round(y / 75.0))
    col_idx = max(1, min(11, col_idx))
    row_idx = max(1, min(11, row_idx))
    col_letter = chr(ord('A') + col_idx - 1)
    row_number = str(row_idx)
    return f"{col_letter}{row_number}"


def intersection_to_pixel(name):
    """Converts grid intersection name (e.g. 'F6') back to 900x900 pixel coordinates."""
    col_idx = ord(name[0].upper()) - ord('A') + 1
    row_idx = int(name[1:])
    return int(col_idx * 75.0), int(row_idx * 75.0)


def extract_survivors_and_obstacles(warped_arena):
    """
    Segments:
    - Critical Survivors (red triangles)
    - Stable Survivors (yellow circles)
    - Obstacles (black extruded shapes)
    """
    hsv = cv2.cvtColor(warped_arena, cv2.COLOR_BGR2HSV)

    # 1. Red Critical Survivors
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    mask_red = cv2.bitwise_or(
        cv2.inRange(hsv, lower_red1, upper_red1),
        cv2.inRange(hsv, lower_red2, upper_red2)
    )
    kernel_sm = np.ones((3, 3), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel_sm)

    # 2. Yellow Stable Survivors
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([38, 255, 255])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel_sm)

    # 3. Black Obstacles (RGB all < 25)
    mask_black = ((warped_arena[:, :, 0] < 25) &
                  (warped_arena[:, :, 1] < 25) &
                  (warped_arena[:, :, 2] < 25)).astype(np.uint8) * 255

    cnts_black, _ = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    obstacle_contours = [c for c in cnts_black if cv2.contourArea(c) > 5000]

    clean_obstacle_mask = np.zeros_like(mask_black)
    cv2.drawContours(clean_obstacle_mask, obstacle_contours, -1, 255, -1)

    # Dilate obstacles by safety clearance (42 pixels ≈ 0.68m drone clearance)
    safety_margin = 42
    kernel_safety = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * safety_margin + 1, 2 * safety_margin + 1))
    dilated_obstacle_mask = cv2.dilate(clean_obstacle_mask, kernel_safety)

    def locate_survivors(mask, min_area=300, max_area=6000):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        centers = {}
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if min_area <= area <= max_area:
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx = M["m10"] / M["m00"]
                    cy = M["m01"] / M["m00"]
                    label = map_pixel_to_intersection(cx, cy)
                    centers[label] = (cx, cy)
        return centers

    critical_centers = locate_survivors(mask_red)
    stable_centers = locate_survivors(mask_yellow)

    return critical_centers, stable_centers, obstacle_contours, dilated_obstacle_mask


def line_of_sight(p1, p2, mask):
    """Returns True if the line segment between p1 and p2 does not intersect the mask."""
    dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    steps = max(2, int(math.ceil(dist / 2.0)))
    xs = np.linspace(p1[0], p2[0], steps)
    ys = np.linspace(p1[1], p2[1], steps)
    for x, y in zip(xs, ys):
        ix = int(round(x))
        iy = int(round(y))
        if 0 <= ix < mask.shape[1] and 0 <= iy < mask.shape[0]:
            if mask[iy, ix] > 0:
                return False
        else:
            return False
    return True


def densify_path(path, max_step=160.0):
    """
    Inserts intermediate pass-through waypoints along straight segments exceeding max_step (~1.45m).
    Ensures the drone never drifts off-course over long spans.
    """
    densified = [path[0]]
    for i in range(len(path) - 1):
        p1 = path[i]
        p2 = path[i + 1]
        dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        if dist > max_step:
            n_div = int(math.ceil(dist / max_step))
            for k in range(1, n_div):
                alpha = k / float(n_div)
                inter = (p1[0] + alpha * (p2[0] - p1[0]), p1[1] + alpha * (p2[1] - p1[1]))
                densified.append(inter)
        densified.append(p2)
    return densified


def plan_path_between(start_px, goal_px, dilated_mask, cell_size=10):
    """
    Plans the shortest collision-free 2D path between start_px and goal_px
    avoiding dilated obstacle keep-out zones using grid A* and line-of-sight shortcutting.
    """
    if line_of_sight(start_px, goal_px, dilated_mask):
        return densify_path([start_px, goal_px], max_step=160.0)

    H, W = dilated_mask.shape
    grid_h, grid_w = H // cell_size, W // cell_size
    grid = dilated_mask.reshape(grid_h, cell_size, grid_w, cell_size).max(axis=(1, 3)) > 0

    s = (int(start_px[1] // cell_size), int(start_px[0] // cell_size))
    g = (int(goal_px[1] // cell_size), int(goal_px[0] // cell_size))

    # Clamp to grid bounds
    s = (max(0, min(grid_h - 1, s[0])), max(0, min(grid_w - 1, s[1])))
    g = (max(0, min(grid_h - 1, g[0])), max(0, min(grid_w - 1, g[1])))

    # If start is blocked, find nearest free cell
    if grid[s[0], s[1]]:
        found = False
        for rad in range(1, 15):
            for dr in range(-rad, rad + 1):
                for dc in range(-rad, rad + 1):
                    nr, nc = s[0] + dr, s[1] + dc
                    if 0 <= nr < grid_h and 0 <= nc < grid_w and not grid[nr, nc]:
                        s = (nr, nc)
                        found = True
                        break
                if found:
                    break
            if found:
                break

    if s == g:
        return densify_path([start_px, goal_px], max_step=160.0)

    dirs = [
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)
    ]

    pq = [(0.0, 0.0, s, [s])]
    visited = {s: 0.0}

    raw_pixel_path = None
    while pq:
        _, cost, u, path = heapq.heappop(pq)
        if u == g:
            raw_pixel_path = [start_px] + [
                (c * cell_size + cell_size // 2, r * cell_size + cell_size // 2)
                for r, c in path[1:-1]
            ] + [goal_px]
            break

        for dr, dc, weight in dirs:
            nr, nc = u[0] + dr, u[1] + dc
            if 0 <= nr < grid_h and 0 <= nc < grid_w and not grid[nr, nc]:
                new_cost = cost + weight
                if (nr, nc) not in visited or new_cost < visited[(nr, nc)]:
                    visited[(nr, nc)] = new_cost
                    h = math.hypot(nr - g[0], nc - g[1])
                    heapq.heappush(pq, (new_cost + h, new_cost, (nr, nc), path + [(nr, nc)]))

    if not raw_pixel_path:
        return densify_path([start_px, goal_px], max_step=160.0)

    # Shortcut line-of-sight smoothing
    smoothed = [raw_pixel_path[0]]
    curr = 0
    while curr < len(raw_pixel_path) - 1:
        next_idx = len(raw_pixel_path) - 1
        while next_idx > curr + 1:
            if line_of_sight(raw_pixel_path[curr], raw_pixel_path[next_idx], dilated_mask):
                break
            next_idx -= 1
        smoothed.append(raw_pixel_path[next_idx])
        curr = next_idx

    return densify_path(smoothed, max_step=160.0)


def path_length(path):
    """Calculates total Euclidean distance of a 2D polyline."""
    d = 0.0
    for i in range(len(path) - 1):
        d += math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
    return d


def load_grid_coordinates():
    """Loads (X, Y) WhyCode lookup table for all intersections from CSV or JSON."""
    candidate_paths = [
        os.path.expanduser("~/pico_ws/src/swift_pico/scripts/grid_whycode_coordinates.csv"),
        os.path.expanduser("~/pico_ws/src/swift_pico/src/grid_whycode_coordinates.csv"),
        os.path.expanduser("~/pico_ws/src/swift_pico/src/grid_whycode_coordinates.json"),
        os.path.join(os.path.dirname(__file__), "grid_whycode_coordinates.csv"),
        os.path.join(os.path.dirname(__file__), "grid_whycode_coordinates.json"),
    ]

    try:
        share_dir = get_package_share_directory("swift_pico")
        candidate_paths.insert(0, os.path.join(share_dir, "scripts", "grid_whycode_coordinates.csv"))
        candidate_paths.insert(1, os.path.join(share_dir, "scripts", "grid_whycode_coordinates.json"))
    except Exception:
        pass

    for path in candidate_paths:
        if os.path.isfile(path):
            if path.endswith(".csv"):
                coords = {}
                with open(path, "r", encoding="utf-8") as f:
                    reader = csv.DictReader(f)
                    for row in reader:
                        name = row.get("intersection") or row.get("name") or row.get("grid")
                        coords[name.strip()] = {
                            "x": float(row["x"]),
                            "y": float(row["y"])
                        }
                return coords
            elif path.endswith(".json"):
                with open(path, "r", encoding="utf-8") as f:
                    return json.load(f)

    raise FileNotFoundError("Could not find grid_whycode_coordinates.csv or .json in any candidate location.")


def draw_arena_visualization(warped_arena, crit_centers, stab_centers,
                             obstacle_contours, dilated_mask,
                             crit_route_pts, crit_order, output_path):
    """
    Draws the full Task 3 arena checkpoint visualization matching Figure 1 and Figure 8.
    """
    overlay = warped_arena.copy()

    # 1. Draw off-limits dilated obstacle zones in semi-transparent red
    red_layer = np.zeros_like(warped_arena)
    dilated_cnts, _ = cv2.findContours(dilated_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.fillPoly(red_layer, dilated_cnts, (0, 0, 160))
    cv2.addWeighted(red_layer, 0.45, overlay, 0.55, 0, overlay)

    # Draw obstacle contour borders and labels
    cv2.drawContours(overlay, dilated_cnts, -1, (0, 0, 255), 2, cv2.LINE_AA)
    for c in obstacle_contours:
        cv2.drawContours(overlay, [c], -1, (0, 0, 200), 2, cv2.LINE_AA)
        M = cv2.moments(c)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.putText(overlay, "OBSTACLE", (cx - 50, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 50, 255), 2, cv2.LINE_AA)

    # 2. Draw F6 Center Marker
    f6_px = (450, 450)
    cv2.circle(overlay, f6_px, 16, (255, 120, 0), 2, cv2.LINE_AA)
    cv2.circle(overlay, f6_px, 24, (255, 120, 0), 2, cv2.LINE_AA)
    cv2.drawMarker(overlay, f6_px, (255, 150, 0), cv2.MARKER_CROSS, 20, 2)
    cv2.putText(overlay, "F6 take-off & landing", (f6_px[0] - 80, f6_px[1] - 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 200, 0), 2, cv2.LINE_AA)

    # 3. Draw Stable Survivors (Yellow)
    for name, pt in stab_centers.items():
        px, py = int(pt[0]), int(pt[1])
        cv2.circle(overlay, (px, py), 26, (0, 255, 255), 3, cv2.LINE_AA)
        cv2.circle(overlay, (px, py), 32, (0, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(overlay, f"{name} Stable", (px - 40, py - 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 2, cv2.LINE_AA)

    # 4. Draw Critical Survivors (Red)
    for name, pt in crit_centers.items():
        px, py = int(pt[0]), int(pt[1])
        cv2.circle(overlay, (px, py), 26, (0, 0, 255), 3, cv2.LINE_AA)
        cv2.circle(overlay, (px, py), 32, (0, 100, 255), 1, cv2.LINE_AA)
        cv2.putText(overlay, f"{name} Critical", (px - 40, py - 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2, cv2.LINE_AA)

    # 5. Draw Planned Critical Route Legs
    if crit_route_pts and len(crit_route_pts) >= 2:
        for i in range(len(crit_route_pts) - 1):
            p1 = (int(crit_route_pts[i][0]), int(crit_route_pts[i][1]))
            p2 = (int(crit_route_pts[i + 1][0]), int(crit_route_pts[i + 1][1]))
            cv2.line(overlay, p1, p2, (0, 220, 255), 3, cv2.LINE_AA)

        # Mark intermediate avoidance waypoints
        for i, pt in enumerate(crit_route_pts):
            p = (int(pt[0]), int(pt[1]))
            cv2.circle(overlay, p, 5, (0, 255, 0), -1, cv2.LINE_AA)

    cv2.imwrite(output_path, overlay)


class WaypointServiceNode(Node):
    """
    Autonomous Waypoint Navigation Service for Task 3A.
    """

    def __init__(self):
        super().__init__('waypoints_service')

        self.cb_group = ReentrantCallbackGroup()
        self.bridge = CvBridge()

        self.coords_map = load_grid_coordinates()
        self.get_logger().info(f"Loaded {len(self.coords_map)} grid intersection coordinates.")

        self.warped_arena = None
        self.crit_centers = {}
        self.stab_centers = {}
        self.obstacle_contours = []
        self.dilated_mask = None

        self.critical_waypoints = []  # List of dict: {'wx': float, 'wy': float, 'wz': 13.0, 'id': str, 'hover': bool}
        self.route_ready_event = threading.Event()

        # Latched, Reliable QoS matching simulator publisher
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # 1. Subscribe to /image_raw once
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            qos,
            callback_group=self.cb_group
        )

        # 2. Provide /waypoints service (GetWaypoints.srv)
        self.srv_waypoints = self.create_service(
            GetWaypoints,
            'waypoints',
            self.waypoint_callback,
            callback_group=self.cb_group
        )

        # 3. Provide /replan_waypoints service (ReplanRoute.srv)
        self.srv_replan = self.create_service(
            ReplanRoute,
            'replan_waypoints',
            self.replan_callback,
            callback_group=self.cb_group
        )

        self.get_logger().info("Waypoint service ready on /waypoints and /replan_waypoints. Waiting for /image_raw...")

    def image_callback(self, msg):
        self.get_logger().info("Received arena /image_raw frame! Running perception and path planning...")

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert ROS Image to OpenCV: {e}")
            return

        # 1. Detect ArUco corners & rectify arena
        try:
            corners, ids = detect_aruco_markers(cv_image)
            src_pts = get_arena_boundary_points(corners, ids)
            dst_pts = np.array([
                [0.0, 0.0],
                [900.0, 0.0],
                [900.0, 900.0],
                [0.0, 900.0]
            ], dtype=np.float32)

            M = cv2.getPerspectiveTransform(src_pts, dst_pts)
            self.warped_arena = cv2.warpPerspective(cv_image, M, (900, 900))
        except Exception as e:
            self.get_logger().error(f"Failed arena rectification: {e}")
            return

        # 2. Extract survivors and obstacles
        self.crit_centers, self.stab_centers, self.obstacle_contours, self.dilated_mask = \
            extract_survivors_and_obstacles(self.warped_arena)

        crit_names = sorted(list(self.crit_centers.keys()))
        stab_names = sorted(list(self.stab_centers.keys()))

        self.get_logger().info(f"Detected {len(crit_names)} Critical Survivors: {crit_names}")
        self.get_logger().info(f"Detected {len(stab_names)} Stable Survivors: {stab_names}")
        self.get_logger().info(f"Detected {len(self.obstacle_contours)} Obstacles")

        # 3. Plan optimal Critical Route from F6 through the 3 critical survivors
        f6_px = (450, 450)
        best_order = None
        best_dist = float('inf')
        best_path_segments = []

        for perm in itertools.permutations(crit_names):
            order_pts = [f6_px] + [self.crit_centers[name] for name in perm]
            total_d = 0.0
            segments = []
            for i in range(len(order_pts) - 1):
                seg = plan_path_between(order_pts[i], order_pts[i + 1], self.dilated_mask)
                total_d += path_length(seg)
                segments.append(seg)
            if total_d < best_dist:
                best_dist = total_d
                best_order = perm
                best_path_segments = segments

        self.get_logger().info(f"Optimal critical route: {best_order} (Distance: {best_dist:.1f} px)")

        # 4. Construct critical waypoints list
        self.critical_waypoints = []
        crit_full_polyline = [f6_px]

        for s_idx, seg in enumerate(best_path_segments):
            survivor_name = best_order[s_idx]
            # Intermediate avoidance points (excluding start and end)
            for pt in seg[1:-1]:
                wx, wy = pixel_to_whycode(pt[0], pt[1])
                self.critical_waypoints.append({
                    'wx': wx,
                    'wy': wy,
                    'wz': 13.0,
                    'id': f"NAV_{survivor_name}",
                    'hover': False
                })
                crit_full_polyline.append(pt)

            # Target critical survivor
            target_pt = self.crit_centers[survivor_name]
            crit_full_polyline.append(target_pt)
            if survivor_name in self.coords_map:
                wx = self.coords_map[survivor_name]["x"]
                wy = self.coords_map[survivor_name]["y"]
            else:
                wx, wy = pixel_to_whycode(target_pt[0], target_pt[1])

            self.critical_waypoints.append({
                'wx': wx,
                'wy': wy,
                'wz': 13.0,
                'id': survivor_name,
                'hover': True
            })
            self.get_logger().info(f"  Critical Goal: {survivor_name} -> WhyCode [{wx:.2f}, {wy:.2f}, 13.00]")

        # 5. Save Checkpoint visualization matching Fig 1
        try:
            draw_arena_visualization(
                self.warped_arena, self.crit_centers, self.stab_centers,
                self.obstacle_contours, self.dilated_mask,
                crit_full_polyline, best_order, "task3_route.png"
            )
            self.get_logger().info("Saved checkpoint route visualization to task3_route.png")
        except Exception as e:
            self.get_logger().warn(f"Failed to save visualization: {e}")

        # 6. Signal route is ready
        self.route_ready_event.set()

        # 7. Drop /image_raw subscription
        if self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
            self.get_logger().info("Dropped /image_raw subscription after single frame capture.")

    def waypoint_callback(self, request, response):
        """Serves the critical route on /waypoints."""
        if request.get_waypoints:
            self.get_logger().info("Incoming request on /waypoints for critical route...")

            if not self.route_ready_event.wait(timeout=20.0):
                self.get_logger().error("Timeout waiting for route planning to finish! No waypoints sent.")
                return response

            response.waypoints.poses = []
            response.marker_ids = []

            for wp in self.critical_waypoints:
                pose = Pose()
                pose.position.x = float(wp['wx'])
                pose.position.y = float(wp['wy'])
                pose.position.z = float(wp['wz'])
                pose.orientation.w = 1.0
                # Use orientation.z as hover flag for standard GetWaypoints (1.0 = hover/save, 0.0 = pass-through)
                pose.orientation.z = 1.0 if wp['hover'] else 0.0
                response.waypoints.poses.append(pose)
                response.marker_ids.append(wp['id'])

            self.get_logger().info(
                f"Responding with {len(response.waypoints.poses)} critical route goals: {response.marker_ids}"
            )
            return response
        else:
            self.get_logger().info("Request rejected (get_waypoints is False)")
            return response

    def replan_callback(self, request, response):
        """
        Serves dynamic re-planning on /replan_waypoints at the Switch.
        Plans from request.position through all 3 stable survivors and ending at F6.
        """
        self.get_logger().info(
            f"Incoming replan request from drone position: [{request.position.x:.2f}, {request.position.y:.2f}, {request.position.z:.2f}]"
        )

        if not self.route_ready_event.wait(timeout=20.0):
            self.get_logger().error("Perception not ready for replanning!")
            return response

        # Convert drone position at the switch to 900x900 pixel space
        cur_px, cur_py = whycode_to_pixel(request.position.x, request.position.y)
        switch_px = (max(20.0, min(880.0, cur_px)), max(20.0, min(880.0, cur_py)))

        stab_names = sorted(list(self.stab_centers.keys()))
        f6_px = (450, 450)

        # Plan optimal sequence through stable survivors ending at F6
        best_order = None
        best_dist = float('inf')
        best_segments = []

        for perm in itertools.permutations(stab_names):
            order_pts = [switch_px] + [self.stab_centers[name] for name in perm] + [f6_px]
            total_d = 0.0
            segments = []
            for i in range(len(order_pts) - 1):
                seg = plan_path_between(order_pts[i], order_pts[i + 1], self.dilated_mask)
                total_d += path_length(seg)
                segments.append(seg)
            if total_d < best_dist:
                best_dist = total_d
                best_order = perm
                best_segments = segments

        self.get_logger().info(f"Optimal stable route from switch: {best_order} -> F6 (Distance: {best_dist:.1f} px)")

        # Build stable route waypoints
        response.waypoints.poses = []
        response.marker_ids = []
        response.hover = []

        stable_full_polyline = [switch_px]

        # Segments for each survivor
        for s_idx in range(len(best_order)):
            survivor_name = best_order[s_idx]
            seg = best_segments[s_idx]

            # Intermediate avoidance points
            for pt in seg[1:-1]:
                wx, wy = pixel_to_whycode(pt[0], pt[1])
                pose = Pose()
                pose.position.x = wx
                pose.position.y = wy
                pose.position.z = 13.0
                pose.orientation.w = 1.0
                pose.orientation.z = 0.0
                response.waypoints.poses.append(pose)
                response.marker_ids.append(f"NAV_{survivor_name}")
                response.hover.append(False)
                stable_full_polyline.append(pt)

            # Target stable survivor
            target_pt = self.stab_centers[survivor_name]
            stable_full_polyline.append(target_pt)
            if survivor_name in self.coords_map:
                wx = self.coords_map[survivor_name]["x"]
                wy = self.coords_map[survivor_name]["y"]
            else:
                wx, wy = pixel_to_whycode(target_pt[0], target_pt[1])

            pose = Pose()
            pose.position.x = wx
            pose.position.y = wy
            pose.position.z = 13.0
            pose.orientation.w = 1.0
            pose.orientation.z = 1.0
            response.waypoints.poses.append(pose)
            response.marker_ids.append(survivor_name)
            response.hover.append(True)
            self.get_logger().info(f"  Stable Goal: {survivor_name} -> WhyCode [{wx:.2f}, {wy:.2f}, 13.00]")

        # Final segment from last stable survivor back to F6
        f6_seg = best_segments[-1]
        for pt in f6_seg[1:-1]:
            wx, wy = pixel_to_whycode(pt[0], pt[1])
            pose = Pose()
            pose.position.x = wx
            pose.position.y = wy
            pose.position.z = 13.0
            pose.orientation.w = 1.0
            pose.orientation.z = 0.0
            response.waypoints.poses.append(pose)
            response.marker_ids.append("NAV_F6")
            response.hover.append(False)
            stable_full_polyline.append(pt)

        # F6 Landing Target
        f6_coords = self.coords_map.get("F6", {"x": 0.0067, "y": 0.0068})
        pose_f6 = Pose()
        pose_f6.position.x = float(f6_coords["x"])
        pose_f6.position.y = float(f6_coords["y"])
        pose_f6.position.z = 13.0
        pose_f6.orientation.w = 1.0
        pose_f6.orientation.z = 1.0
        response.waypoints.poses.append(pose_f6)
        response.marker_ids.append("F6")
        response.hover.append(True)
        stable_full_polyline.append(f6_px)
        self.get_logger().info(f"  Final Home Goal: F6 -> WhyCode [{pose_f6.position.x:.2f}, {pose_f6.position.y:.2f}, 13.00]")

        # Draw replan route checkpoint
        try:
            draw_arena_visualization(
                self.warped_arena, self.crit_centers, self.stab_centers,
                self.obstacle_contours, self.dilated_mask,
                stable_full_polyline, best_order, "task3_replan_route.png"
            )
            self.get_logger().info("Saved replan route visualization to task3_replan_route.png")
        except Exception as e:
            self.get_logger().warn(f"Failed to save replan visualization: {e}")

        self.get_logger().info(f"Replan service responding with {len(response.waypoints.poses)} goals ending at F6.")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = WaypointServiceNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()