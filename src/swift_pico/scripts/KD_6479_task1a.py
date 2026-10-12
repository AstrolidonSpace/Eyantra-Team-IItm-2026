#!/usr/bin/env python3


# This script is used to run the first task 1A of Khojo Drone.

# You will be building the image processing pipeline for the drone. The pipeline will take in an image and output a txt file. 
#!/usr/bin/env python3
#!/usr/bin/env python3
import sys
import os
import argparse
import cv2
import numpy as np

def detect_aruco_markers(image):
    """
    Detects DICT_4X4_250 markers, supporting both OpenCV 4.5 and 4.7+ APIs.
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Check for OpenCV 4.7+ ArucoDetector class
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
    Validates presence of IDs 80, 85, 90, 95 and determines the inner corner
    of each marker that outlines the 900x900 playing arena.
    """
    required_ids = {80, 85, 90, 95}
    if ids is None or len(ids) == 0:
        print("Error: No ArUco markers detected in the image.")
        sys.exit(1)
        
    # Flatten array to handle both 1D and 2D array formats across OpenCV versions
    detected_ids = [int(x) for x in ids.flatten()]
    missing = required_ids - set(detected_ids)
    if missing:
        print(f"Error: Missing required marker IDs: {missing}")
        sys.exit(1)
        
    marker_dict = {}
    for i, mid in enumerate(detected_ids):
        if mid in required_ids:
            marker_dict[mid] = corners[i][0]  # Array of 4 (x, y) coordinates

    # Approximate arena center from all corners of the 4 markers
    all_pts = np.vstack([marker_dict[mid] for mid in required_ids])
    arena_center = np.mean(all_pts, axis=0)

    # The arena boundary vertex is the corner of each marker closest to the center
    inner_corners = {}
    for mid in required_ids:
        pts = marker_dict[mid]
        dists = np.linalg.norm(pts - arena_center, axis=1)
        inner_corners[mid] = pts[np.argmin(dists)]

    # Perspective order: Top-Left (80), Top-Right (85), Bottom-Right (90), Bottom-Left (95)
    src_pts = np.array([
        inner_corners[80],
        inner_corners[85],
        inner_corners[90],
        inner_corners[95]
    ], dtype=np.float32)

    return src_pts, detected_ids

def map_pixel_to_intersection(x, y):
    """
    Snaps an (x, y) pixel location on a 900x900 board to the nearest intersection.
    Grid cell width = 75px. Valid intersection indexes are 1 through 11.
    """
    col_idx = int(round(x / 75.0))
    row_idx = int(round(y / 75.0))

    # Clamp to intersection boundaries (A-K / 1-11)
    col_idx = max(1, min(11, col_idx))
    row_idx = max(1, min(11, row_idx))

    col_letter = chr(ord('A') + col_idx - 1)
    row_number = str(row_idx)
    return f"{col_letter}{row_number}"

def extract_survivors(warped_arena):
    """
    Segments yellow circles (Stable) and red triangles (Critical) using HSV masks.
    """
    hsv = cv2.cvtColor(warped_arena, cv2.COLOR_BGR2HSV)

    # Red hue wraps around 0/180
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    
    mask_red = cv2.bitwise_or(
        cv2.inRange(hsv, lower_red1, upper_red1),
        cv2.inRange(hsv, lower_red2, upper_red2)
    )

    # Yellow range
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([38, 255, 255])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Morphological noise removal
    kernel = np.ones((3, 3), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel)

    def locate_marker_centers(mask, min_area=30, max_area=6000):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        labels = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if min_area <= area <= max_area:
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx = M["m10"] / M["m00"]
                    cy = M["m01"] / M["m00"]
                    labels.append(map_pixel_to_intersection(cx, cy))
        return sorted(list(set(labels)))

    critical = locate_marker_centers(mask_red)
    stable = locate_marker_centers(mask_yellow)

    return critical, stable

def main():
    parser = argparse.ArgumentParser(description="Task 1A: Arena Survivor Detection")
    parser.add_argument("--image", required=True, help="Path to input arena image")
    args = parser.parse_args()

    # Verify image exists
    if not os.path.isfile(args.image):
        print(f"Error: File '{args.image}' not found.")
        sys.exit(1)

    image = cv2.imread(args.image)
    if image is None:
        print(f"Error: Could not load image from '{args.image}'.")
        sys.exit(1)

    # 1. Detect ArUco corners
    corners, ids = detect_aruco_markers(image)
    src_pts, detected_ids = get_arena_boundary_points(corners, ids)

    # 2. Straighten the arena to 900x900
    dst_pts = np.array([
        [0.0, 0.0],
        [900.0, 0.0],
        [900.0, 900.0],
        [0.0, 900.0]
    ], dtype=np.float32)

    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    warped = cv2.warpPerspective(image, M, (900, 900))

    # 3. Detect survivor positions
    critical_survivors, stable_survivors = extract_survivors(warped)

    # 4. Construct output file path (<name>_results.txt in same directory)
    base_name = os.path.splitext(args.image)[0]
    output_filename = f"{base_name}_results.txt"

    # Format output according to task specification
    ids_line = f"Detected marker IDs: [{', '.join(map(str, sorted(detected_ids)))}]"
    crit_line = f"Critical Survivors: {', '.join(critical_survivors)}"
    stab_line = f"Stable Survivors: {', '.join(stable_survivors)}"

    with open(output_filename, "w") as f:
        f.write(f"{ids_line}\n\n")
        f.write(f"{crit_line}\n")
        f.write(f"{stab_line}\n")

if __name__ == "__main__":
    main()
