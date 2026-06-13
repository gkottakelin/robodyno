#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import os
import time

from controller import Robot

try:
    import cv2
    import numpy as np
except ImportError as exc:
    cv2 = None
    np = None
    import_error = exc


WINDOW_NAME = "top_camera_preview"
BASE_DIR = os.path.dirname(__file__)
SNAPSHOT_FILE = os.path.join(BASE_DIR, "camera_preview_latest.png")
VISION_RESULT_FILE = os.path.join(BASE_DIR, "vision_latest.json")

# The Webots top camera image is opposite to the competition field view.
# Detection, preview, snapshots, and exported coordinates all use this corrected image.
IMAGE_ROTATE_180 = True
IMAGE_TRANSFORM_LABEL = "rotate_180" if IMAGE_ROTATE_180 else "none"

# The camera is fixed in province_scene.wbt, so use a calibrated table box first.
# This avoids coordinate drift when the pick table and floor have low contrast.
TABLE_BOUNDS_MODE = "fixed_calibration"

# Ratio bounds are x, y, width, height in the corrected 640x480 preview.
# Tuned for top_camera translation -0.032 0.245 0.45.
ROTATED_PICK_TABLE_BOUNDS_RATIO = (0.11, 0.22, 0.82, 0.52)
RAW_PICK_TABLE_BOUNDS_RATIO = (0.08, 0.36, 0.84, 0.44)
TOP_CAMERA_TRANSLATION_M = (-0.032, 0.245, 0.45)

# World coordinates of the pick table in province_scene.wbt.
PICK_TABLE_CENTER_M = (-0.025, 0.245)
PICK_TABLE_SIZE_M = (0.300, 0.140)
PICK_TABLE_Z_M = 0.010

MIN_OBJECT_AREA_PX = 450
AXIS_TIE_RADIUS_PX = 2.5
AXIS_TIE_RADIUS_RATIO = 0.04

UNDIRECTED_AXIS_SHAPES = (
    "FivePointed",
    "Pentagonal",
    "Triangular",
    "Quincunx",
    "Cruciform",
)

ANGLE_FREE_SHAPES = ("Cylindrical", "Cube")

COLOR_RANGES = {
    "red": [((0, 80, 70), (8, 255, 255)), ((170, 80, 70), (180, 255, 255))],
    "orange": [((9, 80, 70), (26, 255, 255))],
    "yellow": [((27, 70, 70), (42, 255, 255))],
    "green": [((43, 70, 45), (88, 255, 255))],
    "cyan": [((80, 45, 60), (100, 255, 255))],
    "blue": [((100, 70, 45), (130, 255, 255))],
    "purple": [((130, 55, 45), (150, 255, 255))],
    "pink": [((151, 55, 60), (169, 255, 255))],
}

COLOR_LABEL = {
    "red": "red",
    "orange": "orange",
    "yellow": "yellow",
    "green": "green",
    "cyan": "cyan",
    "blue": "blue",
    "purple": "purple",
    "pink": "pink",
}

# The simulated materials currently use a fixed color-to-part mapping.
# Geometry is still checked first when it is reliable.
COLOR_TO_SHAPE = {
    "red": ("Cuboid", "cuboid"),
    "orange": ("Cube", "cube"),
    "yellow": ("Cylindrical", "cylindrical"),
    "green": ("Pentagonal", "pentagonal"),
    "cyan": ("Quincunx", "quincunx"),
    "blue": ("Parallelogram", "parallelogram"),
    "purple": ("Cruciform", "cruciform"),
    "pink": ("FivePointed", "five_pointed"),
}


def main():
    robot = Robot()
    time_step = int(robot.getBasicTimeStep())
    camera = robot.getDevice("camera")
    camera.enable(time_step)

    try:
        camera.recognitionEnable(time_step)
    except Exception:
        pass

    if cv2 is None or np is None:
        print("OpenCV preview is unavailable.")
        print("Install dependencies in the Webots Python environment:")
        print("pip install opencv-python numpy")
        print(f"Original import error: {import_error}")
        print(f"Saving one snapshot per second to: {SNAPSHOT_FILE}")
        frame_id = 0
        frames_per_second = max(1, int(1000 / time_step))
        while robot.step(time_step) != -1:
            frame_id += 1
            if frame_id % frames_per_second == 0:
                camera.saveImage(SNAPSHOT_FILE, 100)
        return

    width = camera.getWidth()
    height = camera.getHeight()

    cv2.startWindowThread()
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, width, height)

    print(f"Camera vision started: {width}x{height}")
    print(f"Vision result file: {VISION_RESULT_FILE}")
    print("Press q/Esc to close. Press s to save a snapshot.")

    frame_count = 0
    last_time = robot.getTime()
    fps = 0.0

    while robot.step(time_step) != -1:
        raw_image = camera.getImage()
        if raw_image is None:
            continue

        image = np.frombuffer(raw_image, np.uint8).reshape((height, width, 4))
        frame = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        frame = apply_image_transform(frame)

        frame_count += 1
        now = robot.getTime()
        if now - last_time >= 1.0:
            fps = frame_count / (now - last_time)
            frame_count = 0
            last_time = now

        detections, calibration = detect_blocks(frame)
        write_vision_result(detections, calibration, now)
        draw_overlay(frame, fps, detections, calibration)

        cv2.imshow(WINDOW_NAME, frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break
        if key == ord("s"):
            cv2.imwrite(SNAPSHOT_FILE, frame)
            print(f"Saved snapshot: {SNAPSHOT_FILE}")

    cv2.destroyAllWindows()


def apply_image_transform(frame):
    if IMAGE_ROTATE_180:
        return cv2.rotate(frame, cv2.ROTATE_180)
    return frame


def detect_blocks(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    table_bounds = detect_pick_table_bounds(hsv, frame.shape)
    calibration = build_calibration(table_bounds)

    roi_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    x, y, w, h = table_bounds
    roi_mask[y : y + h, x : x + w] = 255

    detections = []
    for color_name, ranges in COLOR_RANGES.items():
        color_mask = make_color_mask(hsv, ranges)
        color_mask = cv2.bitwise_and(color_mask, roi_mask)
        color_mask = clean_mask(color_mask)

        for contour in find_external_contours(color_mask):
            area = cv2.contourArea(contour)
            if area < MIN_OBJECT_AREA_PX:
                continue

            center = contour_center(contour)
            if center is None:
                continue

            px, py = center
            world_x, world_y = pixel_to_world(px, py, calibration)

            geometry_shape, geometry_label, geometry_conf = infer_shape_from_geometry(contour)
            color_shape, color_shape_label = COLOR_TO_SHAPE[color_name]
            if geometry_conf >= 0.70:
                shape = geometry_shape
                shape_label = geometry_label
            else:
                shape = color_shape
                shape_label = color_shape_label

            angle_deg = measure_angle_deg(shape, contour, center)

            detections.append(
                {
                    "shape": shape,
                    "shape_label": shape_label,
                    "color": color_name,
                    "color_label": COLOR_LABEL[color_name],
                    "pixel_center": [round(px, 1), round(py, 1)],
                    "world_center_m": [round(world_x, 5), round(world_y, 5), PICK_TABLE_Z_M],
                    "angle_z_deg": round(angle_deg, 1),
                    "area_px": round(area, 1),
                    "confidence": estimate_confidence(area, geometry_conf),
                }
            )

    detections.sort(key=lambda item: (item["world_center_m"][1], item["world_center_m"][0]))
    return detections, calibration


def detect_pick_table_bounds(hsv, frame_shape):
    height, width = frame_shape[:2]
    fixed_bounds = clamp_bounds(default_pick_table_bounds(width, height), width, height)
    if TABLE_BOUNDS_MODE == "fixed_calibration":
        return fixed_bounds

    green_edge_mask = cv2.inRange(hsv, np.array((40, 18, 80)), np.array((90, 140, 255)))
    gray_box_mask = cv2.inRange(hsv, np.array((0, 0, 120)), np.array((180, 65, 235)))
    table_mask = cv2.bitwise_or(green_edge_mask, gray_box_mask)

    kernel = np.ones((19, 19), np.uint8)
    table_mask = cv2.morphologyEx(table_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    table_mask = cv2.morphologyEx(table_mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8), iterations=1)
    table_mask = cv2.dilate(table_mask, np.ones((9, 9), np.uint8), iterations=1)

    best = None
    best_area = 0.0
    for contour in find_external_contours(table_mask):
        area = cv2.contourArea(contour)
        if area < width * height * 0.08:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if h == 0:
            continue
        ratio = w / float(h)
        if 1.6 <= ratio <= 3.4 and w > width * 0.45 and h > height * 0.22 and area > best_area:
            best = (x, y, w, h)
            best_area = area

    if best is not None:
        return clamp_bounds(best, width, height)

    return fixed_bounds


def default_pick_table_bounds(width, height):
    ratios = ROTATED_PICK_TABLE_BOUNDS_RATIO if IMAGE_ROTATE_180 else RAW_PICK_TABLE_BOUNDS_RATIO
    x_ratio, y_ratio, w_ratio, h_ratio = ratios
    return (
        int(width * x_ratio),
        int(height * y_ratio),
        int(width * w_ratio),
        int(height * h_ratio),
    )


def clamp_bounds(bounds, width, height):
    x, y, w, h = bounds
    x = max(0, min(int(x), width - 1))
    y = max(0, min(int(y), height - 1))
    w = max(1, min(int(w), width - x))
    h = max(1, min(int(h), height - y))
    return (x, y, w, h)


def build_calibration(table_bounds):
    x, y, w, h = table_bounds
    center_px = (x + w / 2.0, y + h / 2.0)
    meters_per_px = (
        PICK_TABLE_SIZE_M[0] / float(w),
        PICK_TABLE_SIZE_M[1] / float(h),
    )
    return {
        "table_bounds_px": [int(x), int(y), int(w), int(h)],
        "table_bounds_mode": TABLE_BOUNDS_MODE,
        "table_center_px": [round(center_px[0], 2), round(center_px[1], 2)],
        "table_center_m": [PICK_TABLE_CENTER_M[0], PICK_TABLE_CENTER_M[1]],
        "table_size_m": [PICK_TABLE_SIZE_M[0], PICK_TABLE_SIZE_M[1]],
        "top_camera_translation_m": [
            TOP_CAMERA_TRANSLATION_M[0],
            TOP_CAMERA_TRANSLATION_M[1],
            TOP_CAMERA_TRANSLATION_M[2],
        ],
        "meters_per_px": [round(meters_per_px[0], 8), round(meters_per_px[1], 8)],
    }


def pixel_to_world(px, py, calibration):
    center_px_x, center_px_y = calibration["table_center_px"]
    meters_per_px_x, meters_per_px_y = calibration["meters_per_px"]
    world_x = PICK_TABLE_CENTER_M[0] + (px - center_px_x) * meters_per_px_x
    world_y = PICK_TABLE_CENTER_M[1] - (py - center_px_y) * meters_per_px_y
    return world_x, world_y


def make_color_mask(hsv, ranges):
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in ranges:
        part = cv2.inRange(hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
        mask = cv2.bitwise_or(mask, part)
    return mask


def clean_mask(mask):
    open_kernel = np.ones((3, 3), np.uint8)
    close_kernel = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel, iterations=2)
    return mask


def find_external_contours(mask):
    result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(result) == 2:
        contours, _ = result
    else:
        _, contours, _ = result
    return contours


def contour_center(contour):
    moments = cv2.moments(contour)
    if abs(moments["m00"]) < 1e-6:
        return None
    return (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"])


def infer_shape_from_geometry(contour):
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    if perimeter <= 1e-6 or area <= 1e-6:
        return ("Unknown", "unknown", 0.0)

    hull = cv2.convexHull(contour)
    hull_area = max(cv2.contourArea(hull), 1e-6)
    solidity = area / hull_area
    circularity = 4.0 * math.pi * area / (perimeter * perimeter)

    approx = cv2.approxPolyDP(contour, 0.035 * perimeter, True)
    vertices = len(approx)
    ratio, skew = min_area_ratio_and_skew(contour, approx)
    defects = count_convexity_defects(contour)

    if circularity > 0.82 and solidity > 0.88:
        return ("Cylindrical", "cylindrical", 0.88)
    if vertices == 3:
        return ("Triangular", "triangular", 0.86)
    if vertices == 4:
        if skew > 14.0:
            return ("Parallelogram", "parallelogram", 0.80)
        if ratio > 1.22:
            return ("Cuboid", "cuboid", 0.84)
        return ("Cube", "cube", 0.84)
    if vertices == 5 and solidity > 0.80:
        return ("Pentagonal", "pentagonal", 0.82)
    if defects >= 5 and solidity < 0.78:
        return ("FivePointed", "five_pointed", 0.82)
    if defects == 4 and solidity < 0.82:
        return ("Cruciform", "cruciform", 0.70)

    return ("Unknown", "unknown", 0.0)


def min_area_ratio_and_skew(contour, approx):
    rect = cv2.minAreaRect(contour)
    width, height = rect[1]
    if min(width, height) <= 1e-6:
        ratio = 1.0
    else:
        ratio = max(width, height) / min(width, height)

    skew = 0.0
    if len(approx) == 4:
        points = approx.reshape(-1, 2).astype(float)
        points = sort_points_clockwise(points)
        angles = []
        for index in range(4):
            prev_point = points[(index - 1) % 4]
            point = points[index]
            next_point = points[(index + 1) % 4]
            vector_a = prev_point - point
            vector_b = next_point - point
            denom = np.linalg.norm(vector_a) * np.linalg.norm(vector_b)
            if denom > 1e-6:
                cosine = np.dot(vector_a, vector_b) / denom
                cosine = max(-1.0, min(1.0, cosine))
                angles.append(math.degrees(math.acos(cosine)))
        if angles:
            skew = max(abs(angle - 90.0) for angle in angles)

    return ratio, skew


def sort_points_clockwise(points):
    center = np.mean(points, axis=0)
    angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
    return points[np.argsort(angles)]


def count_convexity_defects(contour):
    if len(contour) < 5:
        return 0
    hull = cv2.convexHull(contour, returnPoints=False)
    if hull is None or len(hull) < 4:
        return 0
    defects = cv2.convexityDefects(contour, hull)
    if defects is None:
        return 0
    _, _, w, h = cv2.boundingRect(contour)
    min_depth = max(w, h) * 256.0 * 0.055
    count = 0
    for defect in defects[:, 0, :]:
        depth = defect[3]
        if depth >= min_depth:
            count += 1
    return count


def measure_angle_deg(shape, contour, center):
    if shape in ANGLE_FREE_SHAPES:
        return 0.0
    if shape in UNDIRECTED_AXIS_SHAPES:
        return center_longest_axis_angle_deg(contour, center)
    return rect_like_angle_deg(contour)


def center_longest_axis_angle_deg(contour, center):
    cx, cy = center
    points = contour.reshape(-1, 2).astype(float)
    vectors = points - np.array((cx, cy))
    radii = np.linalg.norm(vectors, axis=1)
    if len(radii) == 0:
        return 0.0

    max_radius = float(np.max(radii))
    if max_radius <= 1e-6:
        return 0.0

    tolerance = max(AXIS_TIE_RADIUS_PX, max_radius * AXIS_TIE_RADIUS_RATIO)
    candidates = []
    for vector, radius in zip(vectors, radii):
        if radius < max_radius - tolerance:
            continue
        dx_world = float(vector[0])
        dy_world = float(-vector[1])
        if dx_world > 0.0:
            dx_world = -dx_world
            dy_world = -dy_world
        candidates.append((dx_world, dy_world, float(radius)))

    if not candidates:
        dx_world = float(vectors[int(np.argmax(radii))][0])
        dy_world = float(-vectors[int(np.argmax(radii))][1])
        if dx_world > 0.0:
            dx_world = -dx_world
            dy_world = -dy_world
        candidates.append((dx_world, dy_world, max_radius))

    # Tie break: first prefer the axis direction that points most to the left,
    # then prefer the longest radius. This keeps regular polygons from jumping
    # between equivalent vertices when the contour has tiny pixel noise.
    dx_world, dy_world, _ = min(candidates, key=lambda item: (item[0], -item[2], abs(item[1])))
    directed_angle = math.degrees(math.atan2(dy_world, dx_world))
    return normalize_symmetric_90(directed_angle)


def rect_like_angle_deg(contour):
    rect = cv2.minAreaRect(contour)
    (width, height) = rect[1]
    angle = rect[2]

    if width < height:
        image_angle = angle + 90.0
    else:
        image_angle = angle

    world_angle = normalize_angle_180(-image_angle)
    return normalize_symmetric_90(world_angle)


def normalize_angle_180(angle):
    while angle >= 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def normalize_symmetric_90(angle):
    angle = normalize_angle_180(angle)
    while angle >= 90.0:
        angle -= 180.0
    while angle < -90.0:
        angle += 180.0
    return angle


def estimate_confidence(area, geometry_conf):
    area_conf = min(1.0, area / 2200.0)
    confidence = 0.45 + 0.35 * area_conf + 0.20 * geometry_conf
    return round(min(0.99, confidence), 2)


def write_vision_result(detections, calibration, sim_time):
    payload = {
        "sim_time_s": round(sim_time, 3),
        "timestamp_unix": round(time.time(), 3),
        "image_transform": IMAGE_TRANSFORM_LABEL,
        "angle_rule": "center_longest_axis_left_priority_min_x_angle",
        "coordinate_rule": {
            "x_m": "table_center_x + (pixel_x - table_center_pixel_x) * meters_per_pixel_x",
            "y_m": "table_center_y - (pixel_y - table_center_pixel_y) * meters_per_pixel_y",
            "z_m": PICK_TABLE_Z_M,
        },
        "calibration": calibration,
        "objects": detections,
    }
    temp_path = VISION_RESULT_FILE + ".tmp"
    with open(temp_path, "w", encoding="utf-8") as file:
        json.dump(payload, file, ensure_ascii=True, indent=2)
    os.replace(temp_path, VISION_RESULT_FILE)


def draw_overlay(frame, fps, detections, calibration):
    height, width = frame.shape[:2]
    table_x, table_y, table_w, table_h = calibration["table_bounds_px"]

    cv2.rectangle(
        frame,
        (table_x, table_y),
        (table_x + table_w, table_y + table_h),
        (80, 210, 80),
        2,
    )

    center_x, center_y = [int(round(value)) for value in calibration["table_center_px"]]
    cv2.drawMarker(
        frame,
        (center_x, center_y),
        (0, 255, 0),
        markerType=cv2.MARKER_CROSS,
        markerSize=28,
        thickness=2,
    )

    draw_text(frame, f"{width}x{height}  FPS:{fps:.1f}", (10, 32), (50, 255, 50), 0.9, 2)
    draw_text(frame, f"objects:{len(detections)}", (10, 62), (50, 255, 50), 0.7, 2)
    draw_text(frame, f"image:{IMAGE_TRANSFORM_LABEL}", (10, 88), (50, 255, 50), 0.55, 1)
    draw_text(frame, f"table:{TABLE_BOUNDS_MODE}", (10, 112), (50, 255, 50), 0.5, 1)

    for detection in detections:
        px, py = detection["pixel_center"]
        cx = int(round(px))
        cy = int(round(py))
        angle = float(detection["angle_z_deg"])
        world_x, world_y, _ = detection["world_center_m"]
        color = color_to_bgr(detection["color"])

        cv2.circle(frame, (cx, cy), 5, color, -1)
        axis_len = 34
        radians = math.radians(-angle)
        endpoint_a = (
            int(round(cx + axis_len * math.cos(radians))),
            int(round(cy + axis_len * math.sin(radians))),
        )
        endpoint_b = (
            int(round(cx - axis_len * math.cos(radians))),
            int(round(cy - axis_len * math.sin(radians))),
        )
        cv2.line(frame, endpoint_a, endpoint_b, color, 2)

        block_w = 190
        block_h = 68
        if cx + 14 + block_w < width:
            label_x = cx + 14
        else:
            label_x = cx - block_w - 14
        label_x = max(4, min(label_x, width - block_w - 4))
        label_y = max(18, min(cy - 34, height - block_h - 4))

        lines = [
            f"shape: {detection['shape_label']}",
            f"coord: ({world_x:.3f}, {world_y:.3f})",
            f"color: {detection['color_label']}",
            f"angle: {angle:.1f} deg",
        ]
        draw_text_block(frame, lines, (label_x, label_y), color, 0.45, 1)


def draw_text(frame, text, origin, color, scale, thickness):
    x, y = origin
    cv2.putText(frame, text, (x + 1, y + 1), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thickness + 2)
    cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness)


def draw_text_block(frame, lines, origin, color, scale, thickness):
    x, y = origin
    line_height = 16
    padding_x = 6
    padding_y = 6
    text_sizes = [cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)[0] for line in lines]
    block_width = max(size[0] for size in text_sizes) + padding_x * 2
    block_height = line_height * len(lines) + padding_y

    x2 = min(frame.shape[1] - 1, x + block_width)
    y2 = min(frame.shape[0] - 1, y + block_height)
    cv2.rectangle(frame, (x, y), (x2, y2), (35, 35, 35), -1)
    cv2.rectangle(frame, (x, y), (x2, y2), color, 1)

    text_x = x + padding_x
    baseline_y = y + padding_y + 10
    for index, line in enumerate(lines):
        draw_text(frame, line, (text_x, baseline_y + index * line_height), color, scale, thickness)


def color_to_bgr(color_name):
    return {
        "red": (0, 0, 255),
        "orange": (0, 120, 255),
        "yellow": (0, 220, 255),
        "green": (0, 200, 0),
        "cyan": (220, 220, 0),
        "blue": (255, 90, 0),
        "purple": (210, 0, 180),
        "pink": (180, 70, 255),
    }.get(color_name, (255, 255, 255))


if __name__ == "__main__":
    main()
