#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import os
import struct
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
SHAPE_DIR = os.path.abspath(os.path.join(BASE_DIR, "..", "..", "robocom_webots", "shapes"))

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
PENTAGONAL_CORNER_LOG = True
PENTAGONAL_CORNER_LOG_INTERVAL_S = 0.1
PENTAGONAL_CORNER_LAST_LOG_TIME = 0.0

ANGLE_FREE_SHAPES = ("Cylindrical",)

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

SHAPE_TEMPLATE_SPECS = (
    ("Cruciform", "cruciform", "cruciform.STL"),
    ("Cube", "cube", "cube.STL"),
    ("Cuboid", "cuboid", "cuboid.STL"),
    ("Cylindrical", "cylindrical", "cylindrical.STL"),
    ("FivePointed", "five_pointed", "five_pointed.STL"),
    ("Parallelogram", "parallelogram", "parallelogram.STL"),
    ("Pentagonal", "pentagonal", "pentagonal.STL"),
    ("Quincunx", "quincunx", "quincunx.STL"),
    ("Triangular", "triangular", "triangle.STL"),
)
SHAPE_TEMPLATE_CACHE = None


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

            shape, shape_label, geometry_conf = infer_shape_from_geometry(contour)

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
    template_match = match_shape_templates(contour)
    if template_match is not None:
        return template_match
    return infer_shape_from_basic_geometry(contour)


def match_shape_templates(contour):
    templates = get_shape_templates()
    if not templates:
        return None

    contour_features = shape_feature_vector(contour)
    scored = []
    for template in templates:
        try:
            hu_score = cv2.matchShapes(contour, template["contour"], cv2.CONTOURS_MATCH_I1, 0.0)
        except cv2.error:
            continue
        feature_score = feature_distance(contour_features, template["features"])
        scored.append((hu_score + feature_score, hu_score, template))

    if not scored:
        return None

    scored.sort(key=lambda item: item[0])

    best_score, best_hu_score, best_template = scored[0]

    if should_try_four_lobed_override(contour, scored):
        four_lobed_match = infer_four_lobed_shape_from_outline(contour, scored)
        if four_lobed_match is not None:
            return refine_detected_shape(contour, scored, four_lobed_match)

    if should_try_quincunx_pentagonal_refine(scored):
        corner_match = infer_quincunx_or_pentagonal_from_corners(contour, scored)
        if corner_match is not None:
            return refine_detected_shape(contour, scored, corner_match)

    second_score = scored[1][0] if len(scored) > 1 else best_score + 1.0
    confidence = template_match_confidence(best_score, second_score, best_hu_score)
    return refine_detected_shape(contour, scored, (best_template["shape"], best_template["label"], confidence))


def should_try_four_lobed_override(contour, scored_templates):
    best_shape = scored_templates[0][2]["shape"]
    if best_shape in ("Cruciform", "Quincunx"):
        return True
    if best_shape == "FivePointed":
        return False
    if best_shape != "Pentagonal":
        return False

    top_shapes = {item[2]["shape"] for item in scored_templates[:3]}
    if "Cruciform" not in top_shapes and "Quincunx" not in top_shapes:
        return False
    return count_convexity_defects(contour) == 4


def get_shape_templates():
    global SHAPE_TEMPLATE_CACHE
    if SHAPE_TEMPLATE_CACHE is not None:
        return SHAPE_TEMPLATE_CACHE

    templates = []
    for shape, label, filename in SHAPE_TEMPLATE_SPECS:
        path = os.path.join(SHAPE_DIR, filename)
        triangles = read_stl_xy_triangles(path)
        contour = projected_triangles_to_contour(triangles)
        if contour is None:
            continue
        templates.append(
            {
                "shape": shape,
                "label": label,
                "contour": contour,
                "features": shape_feature_vector(contour),
            }
        )

    SHAPE_TEMPLATE_CACHE = templates
    return SHAPE_TEMPLATE_CACHE


def read_stl_xy_triangles(path):
    if not os.path.exists(path):
        return []

    with open(path, "rb") as file:
        data = file.read()

    triangles = read_binary_stl_xy_triangles(data)
    if triangles:
        return triangles
    return read_ascii_stl_xy_triangles(data)


def read_binary_stl_xy_triangles(data):
    if len(data) < 84:
        return []

    triangle_count = struct.unpack("<I", data[80:84])[0]
    expected_size = 84 + triangle_count * 50
    if triangle_count <= 0 or expected_size != len(data):
        return []

    triangles = []
    offset = 84
    for _ in range(triangle_count):
        offset += 12
        points = []
        for _ in range(3):
            x, y, _ = struct.unpack("<fff", data[offset : offset + 12])
            points.append((x, y))
            offset += 12
        offset += 2
        triangles.append(np.array(points, dtype=float))
    return triangles


def read_ascii_stl_xy_triangles(data):
    triangles = []
    points = []
    for raw_line in data.decode("utf-8", "ignore").splitlines():
        parts = raw_line.strip().split()
        if len(parts) != 4 or parts[0].lower() != "vertex":
            continue
        try:
            points.append((float(parts[1]), float(parts[2])))
        except ValueError:
            points = []
            continue
        if len(points) == 3:
            triangles.append(np.array(points, dtype=float))
            points = []
    return triangles


def projected_triangles_to_contour(triangles, template_size=180, padding=18):
    if not triangles:
        return None

    all_points = np.vstack(triangles)
    min_xy = np.min(all_points, axis=0)
    max_xy = np.max(all_points, axis=0)
    span_xy = max_xy - min_xy
    max_span = float(np.max(span_xy))
    if max_span <= 1e-9:
        return None

    scale = (template_size - 2.0 * padding) / max_span
    mask = np.zeros((template_size, template_size), dtype=np.uint8)
    for triangle in triangles:
        if polygon_area(triangle) <= 1e-12:
            continue
        points = (triangle - min_xy) * scale + padding
        points[:, 1] = template_size - points[:, 1]
        cv2.fillConvexPoly(mask, np.round(points).astype(np.int32), 255)

    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8), iterations=1)
    contours = find_external_contours(mask)
    if not contours:
        return None
    return max(contours, key=cv2.contourArea)


def polygon_area(points):
    x_values = points[:, 0]
    y_values = points[:, 1]
    return abs(float(np.dot(x_values, np.roll(y_values, -1)) - np.dot(y_values, np.roll(x_values, -1)))) * 0.5


def shape_feature_vector(contour):
    area = max(cv2.contourArea(contour), 1e-6)
    perimeter = max(cv2.arcLength(contour, True), 1e-6)
    hull = cv2.convexHull(contour)
    hull_area = max(cv2.contourArea(hull), 1e-6)
    approx = cv2.approxPolyDP(contour, 0.035 * perimeter, True)
    ratio, skew = min_area_ratio_and_skew(contour, approx)
    return {
        "circularity": 4.0 * math.pi * area / (perimeter * perimeter),
        "solidity": area / hull_area,
        "ratio": ratio,
        "skew": skew / 90.0,
        "defects": min(count_convexity_defects(contour), 8) / 8.0,
        "radial": radial_variation(contour),
    }


def radial_variation(contour):
    center = contour_center(contour)
    if center is None:
        return 0.0
    cx, cy = center
    points = contour.reshape(-1, 2).astype(float)
    radii = np.linalg.norm(points - np.array((cx, cy)), axis=1)
    mean_radius = float(np.mean(radii))
    if mean_radius <= 1e-6:
        return 0.0
    return float(np.std(radii) / mean_radius)


def feature_distance(left, right):
    return (
        0.22 * abs(left["circularity"] - right["circularity"])
        + 0.30 * abs(left["solidity"] - right["solidity"])
        + 0.05 * abs(left["ratio"] - right["ratio"])
        + 0.08 * abs(left["skew"] - right["skew"])
        + 0.20 * abs(left["defects"] - right["defects"])
        + 0.25 * abs(left["radial"] - right["radial"])
    )


def template_match_confidence(best_score, second_score, best_hu_score):
    separation = max(0.0, second_score - best_score) / max(second_score, 1e-6)
    quality = 1.0 / (1.0 + 8.0 * max(best_hu_score, 0.0))
    confidence = 0.50 + 0.25 * separation + 0.25 * quality
    return round(max(0.50, min(0.99, confidence)), 2)


def refine_detected_shape(contour, scored_templates, shape_match):
    shape_match = refine_quincunx_to_cube_or_cuboid(contour, scored_templates, shape_match)
    return refine_cube_or_cuboid_by_ratio(contour, scored_templates, shape_match)


def refine_cube_or_cuboid_by_ratio(contour, scored_templates, shape_match):
    if shape_match[0] not in ("Cube", "Cuboid"):
        return shape_match

    ratio = contour_min_area_ratio(contour)
    if ratio >= 1.18:
        return ("Cuboid", "cuboid", 0.88)
    if ratio <= 1.10:
        return ("Cube", "cube", 0.88)

    return choose_cube_or_cuboid_from_weights(scored_templates, {"ratio": ratio})


def refine_quincunx_to_cube_or_cuboid(contour, scored_templates, shape_match):
    if shape_match[0] != "Quincunx":
        return shape_match

    rectangular_profile = rectangular_geometry_analysis(contour)
    if not is_rectangular_block_candidate(rectangular_profile):
        return shape_match

    return choose_cube_or_cuboid_from_weights(scored_templates, rectangular_profile)


def rectangular_geometry_analysis(contour):
    samples, perimeter = dense_contour_samples(contour, 240)
    empty_profile = {
        "sample_count": 0,
        "edge_count": 0,
        "straight_edge_count": 0,
        "right_angle_count": 0,
        "corner_angles_deg": [],
        "edge_linearity": 0.0,
        "ratio": 1.0,
    }
    if perimeter <= 1e-6 or len(samples) < 200:
        return empty_profile

    ratio = contour_min_area_ratio(contour)
    straight_segments = dense_straight_edge_segments(samples, perimeter)
    right_angles = dense_right_angle_corners(samples, perimeter)
    straight_sample_count = sum(len(segment) for segment in straight_segments)
    edge_linearity = straight_sample_count / max(len(samples), 1)

    return {
        "sample_count": len(samples),
        "edge_count": len(straight_segments),
        "straight_edge_count": len(straight_segments),
        "right_angle_count": len(right_angles),
        "corner_angles_deg": [round(angle, 1) for angle in right_angles],
        "edge_linearity": edge_linearity,
        "ratio": ratio,
    }


def dense_contour_samples(contour, min_sample_count):
    points = contour.reshape(-1, 2).astype(float)
    if len(points) < 2:
        return points, 0.0

    filtered = [points[0]]
    for point in points[1:]:
        if float(np.linalg.norm(point - filtered[-1])) > 1e-6:
            filtered.append(point)
    if len(filtered) > 1 and float(np.linalg.norm(filtered[0] - filtered[-1])) <= 1e-6:
        filtered.pop()
    if len(filtered) < 2:
        return np.array(filtered, dtype=float), 0.0

    points = np.array(filtered, dtype=float)
    closed_points = np.vstack((points, points[0]))
    segment_vectors = closed_points[1:] - closed_points[:-1]
    segment_lengths = np.linalg.norm(segment_vectors, axis=1)
    valid = segment_lengths > 1e-6
    if not np.any(valid):
        return points, 0.0

    start_points = closed_points[:-1][valid]
    segment_vectors = segment_vectors[valid]
    segment_lengths = segment_lengths[valid]
    perimeter = float(np.sum(segment_lengths))
    if perimeter <= 1e-6:
        return points, 0.0

    sample_count = max(min_sample_count, int(math.ceil(perimeter)))
    distances = np.linspace(0.0, perimeter, sample_count, endpoint=False)
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    segment_indices = np.searchsorted(cumulative, distances, side="right") - 1
    segment_indices = np.clip(segment_indices, 0, len(segment_lengths) - 1)
    local_distances = distances - cumulative[segment_indices]
    t_values = local_distances / segment_lengths[segment_indices]
    samples = start_points[segment_indices] + segment_vectors[segment_indices] * t_values[:, None]
    return samples, perimeter


def dense_straight_edge_segments(samples, perimeter):
    local_angles = dense_local_corner_angles(samples, max(5, len(samples) // 60))
    straight_flags = [angle >= 165.0 for angle in local_angles]
    clusters = cyclic_true_clusters(straight_flags)
    min_cluster_size = max(14, len(samples) // 14)
    rms_limit = max(1.8, 0.008 * perimeter)
    max_limit = max(3.2, 0.016 * perimeter)

    straight_segments = []
    for cluster in clusters:
        if len(cluster) < min_cluster_size:
            continue
        rms_error, max_error = line_fit_errors(samples[np.array(cluster)])
        if rms_error <= rms_limit and max_error <= max_limit:
            straight_segments.append(cluster)
    return straight_segments


def dense_right_angle_corners(samples, perimeter):
    angle_step = max(5, len(samples) // 48)
    local_angles = dense_local_corner_angles(samples, angle_step)
    right_angle_flags = [80.0 <= angle <= 100.0 for angle in local_angles]
    clusters = cyclic_true_clusters(right_angle_flags)
    angles = []
    for cluster in clusters:
        center_index = cluster[len(cluster) // 2]
        if not dense_corner_has_straight_arms(samples, center_index, angle_step, perimeter):
            continue
        angles.append(min(local_angles[index] for index in cluster))
    return angles


def dense_local_corner_angles(samples, step):
    angles = []
    sample_count = len(samples)
    for index in range(sample_count):
        prev_point = samples[(index - step) % sample_count]
        point = samples[index]
        next_point = samples[(index + step) % sample_count]
        angles.append(point_corner_angle_deg(prev_point, point, next_point))
    return angles


def dense_corner_has_straight_arms(samples, center_index, angle_step, perimeter):
    sample_count = len(samples)
    arm_span = max(12, sample_count // 28)
    prev_indices = [
        (center_index - offset) % sample_count
        for offset in range(arm_span, angle_step, -1)
    ]
    next_indices = [
        (center_index + offset) % sample_count
        for offset in range(angle_step + 1, arm_span + 1)
    ]
    if len(prev_indices) < 5 or len(next_indices) < 5:
        return False

    rms_limit = max(1.8, 0.008 * perimeter)
    max_limit = max(3.2, 0.016 * perimeter)
    prev_rms, prev_max = line_fit_errors(samples[np.array(prev_indices)])
    next_rms, next_max = line_fit_errors(samples[np.array(next_indices)])
    return (
        prev_rms <= rms_limit
        and prev_max <= max_limit
        and next_rms <= rms_limit
        and next_max <= max_limit
    )


def point_corner_angle_deg(prev_point, point, next_point):
    vector_a = prev_point - point
    vector_b = next_point - point
    denom = float(np.linalg.norm(vector_a) * np.linalg.norm(vector_b))
    if denom <= 1e-6:
        return 180.0
    cosine = float(np.dot(vector_a, vector_b) / denom)
    cosine = max(-1.0, min(1.0, cosine))
    return math.degrees(math.acos(cosine))


def line_fit_errors(points):
    if len(points) < 2:
        return float("inf"), float("inf")
    center = np.mean(points, axis=0)
    centered = points - center
    covariance = centered.T @ centered
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    normal = eigenvectors[:, int(np.argmin(eigenvalues))]
    distances = np.abs(centered @ normal)
    return float(np.sqrt(np.mean(distances * distances))), float(np.max(distances))


def cyclic_true_clusters(flags):
    if not flags or not any(flags):
        return []
    count = len(flags)
    if all(flags):
        return [list(range(count))]

    start = 0
    for index, flag in enumerate(flags):
        if not flag:
            start = (index + 1) % count
            break

    clusters = []
    current = []
    for offset in range(count):
        index = (start + offset) % count
        if flags[index]:
            current.append(index)
        elif current:
            clusters.append(current)
            current = []
    if current:
        clusters.append(current)
    return clusters


def is_rectangular_block_candidate(rectangular_profile):
    right_angle_count = rectangular_profile["right_angle_count"]
    has_four_straight_edges = (
        rectangular_profile["straight_edge_count"] >= 4
        and rectangular_profile["edge_linearity"] >= 0.55
    )
    has_dense_right_angles = (
        2 <= right_angle_count <= 4
        and rectangular_profile["edge_linearity"] >= 0.35
    )
    return has_four_straight_edges or has_dense_right_angles


def choose_cube_or_cuboid_from_weights(scored_templates, rectangular_profile):
    cube_template_score = template_score_for_shape(scored_templates, "Cube")
    cuboid_template_score = template_score_for_shape(scored_templates, "Cuboid")
    ratio = rectangular_profile["ratio"]

    if ratio >= 1.18:
        return ("Cuboid", "cuboid", 0.88)
    if ratio <= 1.10:
        return ("Cube", "cube", 0.88)

    cube_weighted_score = cube_template_score + 0.35 * max(0.0, ratio - 1.10)
    cuboid_weighted_score = cuboid_template_score + 0.35 * max(0.0, 1.18 - ratio)

    if cuboid_weighted_score < cube_weighted_score:
        return ("Cuboid", "cuboid", 0.86)
    return ("Cube", "cube", 0.86)


def template_score_for_shape(scored_templates, shape):
    for score, _, template in scored_templates:
        if template["shape"] == shape:
            return score
    return 1.0


def contour_min_area_ratio(contour):
    rect = cv2.minAreaRect(contour)
    width, height = rect[1]
    if min(width, height) <= 1e-6:
        return 1.0
    return max(width, height) / min(width, height)


def should_try_quincunx_pentagonal_refine(scored_templates):
    best_shape = scored_templates[0][2]["shape"]
    if best_shape == "Pentagonal":
        return True
    if best_shape != "Quincunx":
        return False

    top_shapes = {item[2]["shape"] for item in scored_templates[:3]}
    return "Pentagonal" in top_shapes


def infer_quincunx_or_pentagonal_from_corners(contour, scored_templates):
    best_shape = scored_templates[0][2]["shape"]
    if best_shape not in ("Quincunx", "Pentagonal"):
        return None

    corner_profile = corner_geometry_analysis(contour)
    defects = count_convexity_defects(contour)
    is_pentagonal = is_pentagonal_corner_profile(corner_profile, defects)
    if is_pentagonal:
        log_pentagonal_corner_decision(
            scored_templates, corner_profile, defects, "Pentagonal", "second_pass"
        )
        return ("Pentagonal", "pentagonal", 0.88)

    log_pentagonal_corner_decision(
        scored_templates, corner_profile, defects, "Quincunx", "second_pass"
    )
    return ("Quincunx", "quincunx", 0.86)


def corner_geometry_analysis(contour):
    perimeter = cv2.arcLength(contour, True)
    empty_profile = {
        "edge_count": 0,
        "corner_count": 0,
        "corner_angles_deg": [],
        "edge_linearity": 0.0,
        "angle_spread_deg": 180.0,
        "epsilon_ratio": 0.0,
    }
    if perimeter <= 1e-6:
        return empty_profile

    best_profile = None
    best_score = None
    for epsilon_ratio in (0.012, 0.016, 0.020, 0.024, 0.028, 0.032):
        approx = cv2.approxPolyDP(contour, epsilon_ratio * perimeter, True)
        points = approx.reshape(-1, 2).astype(float)
        points = remove_short_polygon_edges(points, perimeter * 0.018)
        if len(points) < 3:
            continue

        edge_lengths = polygon_edge_lengths(points)
        if not edge_lengths:
            continue

        angles = polygon_corner_angles_deg(points)
        if not angles:
            continue

        corner_count = sum(45.0 <= angle <= 155.0 for angle in angles)
        edge_linearity = sum(edge_lengths) / perimeter
        angle_spread = max(angles) - min(angles)
        edge_count = len(points)
        profile = {
            "edge_count": edge_count,
            "corner_count": corner_count,
            "corner_angles_deg": [round(angle, 1) for angle in angles],
            "edge_linearity": edge_linearity,
            "angle_spread_deg": angle_spread,
            "epsilon_ratio": epsilon_ratio,
        }
        score = (
            4.0 * abs(edge_count - 5)
            + 2.0 * abs(corner_count - 5)
            + 3.0 * max(0.0, 0.86 - edge_linearity)
            + epsilon_ratio
        )
        if best_score is None or score < best_score:
            best_profile = profile
            best_score = score

    return best_profile if best_profile is not None else empty_profile


def remove_short_polygon_edges(points, min_edge_length):
    if len(points) < 3:
        return points

    filtered = list(points)
    changed = True
    while changed and len(filtered) >= 4:
        changed = False
        for index in range(len(filtered)):
            current_point = filtered[index]
            next_point = filtered[(index + 1) % len(filtered)]
            if float(np.linalg.norm(next_point - current_point)) < min_edge_length:
                del filtered[(index + 1) % len(filtered)]
                changed = True
                break
    return np.array(filtered, dtype=float)


def polygon_edge_lengths(points):
    lengths = []
    for index in range(len(points)):
        vector = points[(index + 1) % len(points)] - points[index]
        length = float(np.linalg.norm(vector))
        if length > 1e-6:
            lengths.append(length)
    return lengths


def polygon_corner_angles_deg(points):
    angles = []
    for index in range(len(points)):
        prev_point = points[(index - 1) % len(points)]
        point = points[index]
        next_point = points[(index + 1) % len(points)]
        vector_a = prev_point - point
        vector_b = next_point - point
        denom = float(np.linalg.norm(vector_a) * np.linalg.norm(vector_b))
        if denom <= 1e-6:
            continue
        cosine = float(np.dot(vector_a, vector_b) / denom)
        cosine = max(-1.0, min(1.0, cosine))
        angles.append(math.degrees(math.acos(cosine)))
    return angles


def is_pentagonal_corner_profile(corner_profile, defects):
    checks = pentagonal_corner_checks(corner_profile, defects)
    return all(checks.values())


def pentagonal_corner_checks(corner_profile, defects):
    angles = corner_profile["corner_angles_deg"]
    if not angles:
        return {
            "defects_le_1": defects <= 1,
            "edge_count_eq_5": False,
            "corner_count_eq_5": False,
            "edge_linearity_ge_0_93": False,
            "angles_in_95_125": False,
            "angle_spread_le_20": False,
        }

    return {
        "defects_le_1": defects <= 1,
        "edge_count_eq_5": corner_profile["edge_count"] == 5,
        "corner_count_eq_5": corner_profile["corner_count"] == 5,
        "edge_linearity_ge_0_93": corner_profile["edge_linearity"] >= 0.93,
        "angles_in_95_125": min(angles) >= 95.0 and max(angles) <= 125.0,
        "angle_spread_le_20": corner_profile["angle_spread_deg"] <= 20.0,
    }


def log_pentagonal_corner_decision(scored_templates, corner_profile, defects, result, source):
    if not PENTAGONAL_CORNER_LOG:
        return
    if not should_emit_pentagonal_corner_log():
        return

    top3 = ", ".join(
        f"{template['shape']}:{score:.4f}"
        for score, _, template in scored_templates[:3]
    )
    checks = pentagonal_corner_checks(corner_profile, defects)
    checks_text = ", ".join(
        f"{name}={int(value)}"
        for name, value in checks.items()
    )
    angles_text = ", ".join(f"{angle:.1f}" for angle in corner_profile["corner_angles_deg"])
    print(
        "[pentagonal-debug] "
        f"source={source} "
        f"top3=[{top3}] "
        f"defects={defects} "
        f"edge_count={corner_profile['edge_count']} "
        f"corner_count={corner_profile['corner_count']} "
        f"corner_angles_deg=[{angles_text}] "
        f"edge_linearity={corner_profile['edge_linearity']:.4f} "
        f"angle_spread_deg={corner_profile['angle_spread_deg']:.1f} "
        f"epsilon_ratio={corner_profile['epsilon_ratio']:.3f} "
        f"checks=[{checks_text}] "
        f"result={result}",
        flush=True,
    )


def should_emit_pentagonal_corner_log():
    global PENTAGONAL_CORNER_LAST_LOG_TIME

    now = time.monotonic()
    if now - PENTAGONAL_CORNER_LAST_LOG_TIME < PENTAGONAL_CORNER_LOG_INTERVAL_S:
        return False
    PENTAGONAL_CORNER_LAST_LOG_TIME = now
    return True


def infer_four_lobed_shape_from_outline(contour, scored_templates):
    top_shapes = {item[2]["shape"] for item in scored_templates[:3]}
    if "Cruciform" not in top_shapes and "Quincunx" not in top_shapes:
        return None

    defects = count_convexity_defects(contour)
    if defects < 3:
        return None

    rectilinear_score = rectilinear_edge_score(contour)
    if rectilinear_score >= 0.62:
        return ("Cruciform", "cruciform", 0.86)
    return ("Quincunx", "quincunx", 0.84)


def rectilinear_edge_score(contour):
    perimeter = cv2.arcLength(contour, True)
    if perimeter <= 1e-6:
        return 0.0

    approx = cv2.approxPolyDP(contour, 0.012 * perimeter, True).reshape(-1, 2).astype(float)
    if len(approx) < 4:
        return 0.0

    angles = []
    lengths = []
    for index in range(len(approx)):
        vector = approx[(index + 1) % len(approx)] - approx[index]
        length = float(np.linalg.norm(vector))
        if length < 1.5:
            continue
        dx_world, dy_world = image_vector_to_world(vector)
        angle = normalize_angle_180(math.degrees(math.atan2(dy_world, dx_world)))
        if angle < 0.0:
            angle += 180.0
        angles.append(angle)
        lengths.append(length)

    if not angles:
        return 0.0

    angles = np.array(angles)
    lengths = np.array(lengths)
    total_length = float(np.sum(lengths))
    best_score = 0.0
    for candidate_angle in np.linspace(0.0, 89.5, 180):
        diffs = np.minimum(
            axis_angle_difference_180(angles, candidate_angle),
            axis_angle_difference_180(angles, candidate_angle + 90.0),
        )
        score = float(np.sum(lengths[diffs <= 12.0]) / total_length)
        best_score = max(best_score, score)
    return best_score


def axis_angle_difference_180(angles, target_angle):
    diff = np.abs((angles - target_angle + 90.0) % 180.0 - 90.0)
    return diff


def infer_shape_from_basic_geometry(contour):
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
    if shape in ("Cube", "Cuboid"):
        return rect_like_angle_deg(contour)
    if shape == "Cruciform":
        return nearest_point_equivalent_angle_deg(contour, center, 90.0)

    equivalent_periods = {
        "Parallelogram": 180.0,
        "Quincunx": 90.0,
        "Pentagonal": 72.0,
        "FivePointed": 72.0,
        "Triangular": 120.0,
    }
    period = equivalent_periods.get(shape)
    if period is not None:
        return farthest_point_equivalent_angle_deg(contour, center, period)
    return farthest_point_angle_deg(contour, center)


def farthest_point_angle_deg(contour, center):
    return extremum_point_angle_deg(contour, center, "farthest")


def farthest_point_equivalent_angle_deg(contour, center, period_deg):
    angle = farthest_point_angle_deg(contour, center)
    return min_abs_equivalent_angle_deg(angle, period_deg)


def nearest_point_equivalent_angle_deg(contour, center, period_deg):
    angle = extremum_point_angle_deg(contour, center, "nearest")
    return min_abs_equivalent_angle_deg(angle, period_deg)


def extremum_point_angle_deg(contour, center, mode):
    cx, cy = center
    points = contour.reshape(-1, 2).astype(float)
    if len(points) == 0:
        return 0.0

    vectors = points - np.array((cx, cy))
    radii = np.linalg.norm(vectors, axis=1)
    if len(radii) == 0 or float(np.max(radii)) <= 1e-6:
        return 0.0

    if mode == "nearest":
        index = int(np.argmin(radii))
    else:
        index = int(np.argmax(radii))
    vector = vectors[index]
    return directed_angle_from_image_vector(vector)


def min_abs_equivalent_angle_deg(angle, period_deg):
    if period_deg <= 0.0:
        return normalize_angle_180_prefer_positive(angle)

    count = max(1, int(round(360.0 / period_deg)))
    candidates = [
        normalize_angle_180_prefer_positive(angle + index * period_deg)
        for index in range(count)
    ]
    return min(candidates, key=lambda value: (abs(value), 0 if value >= 0.0 else 1))


def rect_like_angle_deg(contour):
    rect = cv2.minAreaRect(contour)
    box = cv2.boxPoints(rect).astype(float)
    box = sort_points_clockwise(box)

    edges = []
    for index in range(4):
        vector = box[(index + 1) % 4] - box[index]
        length = float(np.linalg.norm(vector))
        edges.append((vector, length))

    max_length = max(length for _, length in edges)
    min_length = max(min(length for _, length in edges), 1e-6)
    if max_length / min_length < 1.10:
        candidates = edges
    else:
        tolerance = max(1.0, max_length * 0.04)
        candidates = [item for item in edges if item[1] >= max_length - tolerance]

    vector, length = min(candidates, key=lambda item: axis_priority_key(item[0], item[1]))
    return axis_angle_from_image_vector(vector)


def axis_priority_key(image_vector, length):
    dx_world, dy_world = preferred_world_vector(image_vector)
    angle = normalize_symmetric_90(math.degrees(math.atan2(dy_world, dx_world)))
    return (abs(angle), -length)


def axis_angle_from_image_vector(image_vector):
    dx_world, dy_world = preferred_world_vector(image_vector)
    directed_angle = math.degrees(math.atan2(dy_world, dx_world))
    return normalize_symmetric_90(directed_angle)


def directed_angle_from_image_vector(image_vector):
    dx_world, dy_world = image_vector_to_world(image_vector)
    directed_angle = math.degrees(math.atan2(dy_world, dx_world))
    return normalize_angle_180_prefer_positive(directed_angle)


def image_vector_to_world(image_vector):
    return float(image_vector[0]), float(-image_vector[1])


def preferred_world_axis(world_axis):
    dx_world = float(world_axis[0])
    dy_world = float(world_axis[1])
    if dx_world > 1e-6:
        dx_world = -dx_world
        dy_world = -dy_world
    elif abs(dx_world) <= 1e-6 and dy_world < 0.0:
        dy_world = -dy_world
    return dx_world, dy_world


def preferred_world_vector(image_vector):
    return preferred_world_axis(image_vector_to_world(image_vector))


def normalize_angle_180_prefer_positive(angle):
    while angle > 180.0:
        angle -= 360.0
    while angle <= -180.0:
        angle += 360.0
    return angle


def normalize_angle_180(angle):
    while angle >= 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def normalize_symmetric_90(angle):
    angle = normalize_angle_180(angle)
    while angle > 90.0:
        angle -= 180.0
    while angle <= -90.0:
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
        "shape_rule": "geometry_only_official_stl_template_match",
        "angle_rule": "circle_zero_cube_cuboid_rect_axis_cruciform_nearest_equivalent_else_farthest_equivalent_min_abs_x_angle",
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
    try:
        os.replace(temp_path, VISION_RESULT_FILE)
    except PermissionError:
        try:
            if os.path.exists(temp_path):
                os.remove(temp_path)
        except OSError:
            pass


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
        endpoint = (
            int(round(cx + axis_len * math.cos(radians))),
            int(round(cy + axis_len * math.sin(radians))),
        )
        cv2.arrowedLine(frame, (cx, cy), endpoint, color, 2, tipLength=0.25)

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
