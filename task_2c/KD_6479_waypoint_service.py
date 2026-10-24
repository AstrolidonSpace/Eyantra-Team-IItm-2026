#!/usr/bin/env python3

"""
Task 2C: Autonomous Waypoint Navigation Service with Vision Pipeline & Path Planning
Team: KD_6479

Workflow:
1. Subscribes once to /image_raw with Transient Local / Reliable QoS.
2. Rectifies arena using ArUco markers (80, 85, 90, 95) to 900x900.
3. Detects all Critical Survivors (red triangles) and maps them to nearest grid intersections.
4. Looks up coordinates (X, Y) from grid_whycode_coordinates.csv (Z=13.0).
5. Plans globally optimal visiting route starting from arena centre (F6) via brute-force TSP.
6. Stores the waypoints, saves route checkpoint visualization, and drops the camera subscription.
7. Serves GetWaypoints requests on /waypoints.
"""

import os
import sys
import math
import itertools
import threading
import json
import csv
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from waypoint_navigation.srv import GetWaypoints
from ament_index_python.packages import get_package_share_directory


def detect_aruco_markers(image):
    """
    Detects ArUco markers using DICT_4X4_250 dictionary.
    Supports OpenCV 4.7+ (ArucoDetector) and earlier OpenCV versions.
    """
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

    # Canonical order: Top-Left (80), Top-Right (85), Bottom-Right (90), Bottom-Left (95)
    src_pts = np.array([
        inner_corners[80],
        inner_corners[85],
        inner_corners[90],
        inner_corners[95]
    ], dtype=np.float32)

    return src_pts


def map_pixel_to_intersection(x, y):
    """
    Snaps pixel coordinate on 900x900 board to the nearest grid intersection (A1 to K11).
    Grid cell spacing = 75px. Columns: A-K, Rows: 1-11.
    """
    col_idx = int(round(x / 75.0))
    row_idx = int(round(y / 75.0))

    col_idx = max(1, min(11, col_idx))
    row_idx = max(1, min(11, row_idx))

    col_letter = chr(ord('A') + col_idx - 1)
    row_number = str(row_idx)
    return f"{col_letter}{row_number}"


def intersection_to_pixel(name):
    """
    Converts grid intersection name (e.g. 'F6', 'H4') back to 900x900 pixel coordinates.
    """
    col_idx = ord(name[0].upper()) - ord('A') + 1
    row_idx = int(name[1:])
    return int(col_idx * 75.0), int(row_idx * 75.0)


def extract_critical_survivors(warped_arena):
    """
    Detects Critical Survivors (red triangles) using HSV color thresholding.
    Returns sorted list of unique intersection labels and dictionary of centroid pixel positions.
    """
    hsv = cv2.cvtColor(warped_arena, cv2.COLOR_BGR2HSV)

    # Red hue wraps around 0 and 180
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    mask_red = cv2.bitwise_or(
        cv2.inRange(hsv, lower_red1, upper_red1),
        cv2.inRange(hsv, lower_red2, upper_red2)
    )

    kernel = np.ones((3, 3), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    survivor_centers = {}
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if 30 <= area <= 6000:
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = M["m10"] / M["m00"]
                cy = M["m01"] / M["m00"]
                label = map_pixel_to_intersection(cx, cy)
                survivor_centers[label] = (cx, cy)

    critical_labels = sorted(list(survivor_centers.keys()))
    return critical_labels, survivor_centers


def load_grid_coordinates():
    """
    Loads (X, Y) WhyCode lookup table for all intersections from CSV or JSON.
    Searches standard workspace paths and installed share directory.
    """
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


def plan_optimal_route(start_label, survivor_labels, coords):
    """
    Finds the shortest visiting route starting from start_label (F6) through all
    survivor_labels by evaluating all N! permutations (brute-force TSP).
    """
    start_pt = coords[start_label]

    def dist(p1, p2):
        return math.hypot(p1["x"] - p2["x"], p1["y"] - p2["y"])

    best_dist = float("inf")
    best_order = None

    for p in itertools.permutations(survivor_labels):
        d = dist(start_pt, coords[p[0]])
        for i in range(len(p) - 1):
            d += dist(coords[p[i]], coords[p[i + 1]])
        if d < best_dist:
            best_dist = d
            best_order = p

    return best_order, best_dist


def draw_checkpoint_route(warped_arena, route_order, output_path):
    """
    Draws the planned route and checkpoints onto the warped frame, matching Fig 8 of the spec.
    """
    overlay = warped_arena.copy()
    pts = [intersection_to_pixel("F6")] + [intersection_to_pixel(s) for s in route_order]

    # Draw START F6 marker
    f6_px = pts[0]
    cv2.drawMarker(overlay, f6_px, (0, 255, 255), cv2.MARKER_TRIANGLE_UP, 24, 2)
    cv2.putText(overlay, "START F6", (f6_px[0] - 40, f6_px[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2, cv2.LINE_AA)

    # Draw route lines between intersections
    for i in range(len(pts) - 1):
        cv2.line(overlay, pts[i], pts[i + 1], (0, 165, 255), 3, cv2.LINE_AA)

    # Draw target survivor circles and numbering (1: S1, 2: S2, etc.)
    for idx, s in enumerate(route_order, 1):
        p = intersection_to_pixel(s)
        cv2.circle(overlay, p, 28, (0, 0, 255), 3, cv2.LINE_AA)
        cv2.circle(overlay, p, 32, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(overlay, f"{idx}: {s}", (p[0] + 15, p[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)

    cv2.imwrite(output_path, overlay)


class WaypointServiceNode(Node):
    """
    ROS 2 Service Node for Task 2C.
    Subscribes to /image_raw, detects survivors, plans shortest route, and serves waypoints.
    """

    def __init__(self):
        super().__init__('waypoints_service')

        self.cb_group = ReentrantCallbackGroup()
        self.bridge = CvBridge()

        self.waypoints = []
        self.marker_ids = []
        self.route_ready_event = threading.Event()

        # Load grid coordinates map
        self.coords_map = load_grid_coordinates()
        self.get_logger().info(f"Loaded {len(self.coords_map)} grid intersection coordinates.")

        # Latched, Reliable QoS profile matching publisher
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

        # 2. Provide /waypoints service
        self.srv = self.create_service(
            GetWaypoints,
            'waypoints',
            self.waypoint_callback,
            callback_group=self.cb_group
        )

        self.get_logger().info("Waypoint service node ready on /waypoints. Waiting for arena /image_raw frame...")

    def image_callback(self, msg):
        self.get_logger().info("Received arena /image_raw frame! Processing vision pipeline...")

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
            warped = cv2.warpPerspective(cv_image, M, (900, 900))
        except Exception as e:
            self.get_logger().error(f"Failed arena rectification: {e}")
            return

        # 2. Detect Critical Survivors
        crit_names, crit_centers = extract_critical_survivors(warped)
        self.get_logger().info(f"Detected {len(crit_names)} Critical Survivors: {crit_names}")

        if len(crit_names) == 0:
            self.get_logger().warn("No critical survivors detected in frame!")
            return

        # 3. Path Planning: Shortest visiting route from F6
        best_order, best_dist = plan_optimal_route("F6", crit_names, self.coords_map)
        self.get_logger().info(f"Optimal route from F6 (Total Euclidean Dist: {best_dist:.3f} m): {best_order}")

        # 4. Build waypoints list: [X, Y, 13.0]
        self.waypoints = []
        self.marker_ids = list(best_order)

        for s in best_order:
            pt = self.coords_map[s]
            self.waypoints.append([pt["x"], pt["y"], 13.00])
            self.get_logger().info(f"  Survivor {s} -> WhyCode Target: [{pt['x']:.2f}, {pt['y']:.2f}, 13.00]")

        # 5. Save Checkpoint visualization matching Fig 8
        try:
            draw_checkpoint_route(warped, best_order, "task2c_route.png")
            self.get_logger().info("Saved route visualization to task2c_route.png")
        except Exception as e:
            self.get_logger().warn(f"Failed to save route plot: {e}")

        # 6. Signal route is ready
        self.route_ready_event.set()

        # 7. Drop the /image_raw subscription (one frame is all that is needed)
        if self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
            self.get_logger().info("Dropped /image_raw subscription after single frame capture.")

    def waypoint_callback(self, request, response):
        if request.get_waypoints:
            self.get_logger().info("Incoming request for Waypoints...")

            # Ensure route has been planned before answering (handles race conditions)
            if not self.route_ready_event.wait(timeout=20.0):
                self.get_logger().error("Timeout waiting for route planning to finish! No waypoints sent.")
                return response

            response.waypoints.poses = [Pose() for _ in range(len(self.waypoints))]
            for i, wp in enumerate(self.waypoints):
                response.waypoints.poses[i].position.x = float(wp[0])
                response.waypoints.poses[i].position.y = float(wp[1])
                response.waypoints.poses[i].position.z = float(wp[2])
                response.waypoints.poses[i].orientation.w = 1.0

            response.marker_ids = self.marker_ids
            self.get_logger().info(
                f"Waypoints service responding with {len(self.waypoints)} goals: {self.marker_ids}"
            )
            return response
        else:
            self.get_logger().info("Request rejected (get_waypoints is False)")
            return response


def main(args=None):
    rclpy.init(args=args)
    node = WaypointServiceNode()

    # Use multi-threaded executor so image callback & service request can be processed concurrently
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