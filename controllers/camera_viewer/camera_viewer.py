#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from controller import Robot
import os

try:
    import cv2
    import numpy as np
except ImportError as exc:
    cv2 = None
    np = None
    import_error = exc


WINDOW_NAME = "top_camera_preview"
SNAPSHOT_FILE = os.path.join(os.path.dirname(__file__), "camera_preview_latest.png")


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
        frame = 0
        frames_per_second = max(1, int(1000 / time_step))
        while robot.step(time_step) != -1:
            frame += 1
            if frame % frames_per_second == 0:
                camera.saveImage(SNAPSHOT_FILE, 100)
        return

    width = camera.getWidth()
    height = camera.getHeight()

    cv2.startWindowThread()
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, width, height)

    print(f"Camera preview started: {width}x{height}")
    print("Press q in the preview window to close the controller.")
    print("Press s in the preview window to save a snapshot.")

    frame_count = 0
    last_time = robot.getTime()
    fps = 0.0

    while robot.step(time_step) != -1:
        raw_image = camera.getImage()
        image = np.frombuffer(raw_image, np.uint8).reshape((height, width, 4))
        frame = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

        frame_count += 1
        now = robot.getTime()
        if now - last_time >= 1.0:
            fps = frame_count / (now - last_time)
            frame_count = 0
            last_time = now

        draw_overlay(frame, camera, fps)
        cv2.imshow(WINDOW_NAME, frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break
        if key == ord("s"):
            camera.saveImage(SNAPSHOT_FILE, 100)
            print(f"Saved snapshot: {SNAPSHOT_FILE}")

    cv2.destroyAllWindows()


def draw_overlay(frame, camera, fps):
    height, width = frame.shape[:2]
    center = (width // 2, height // 2)

    cv2.line(frame, (center[0] - 18, center[1]), (center[0] + 18, center[1]), (0, 255, 0), 1)
    cv2.line(frame, (center[0], center[1] - 18), (center[0], center[1] + 18), (0, 255, 0), 1)
    cv2.circle(frame, center, 4, (0, 255, 0), 1)

    status = f"{width}x{height}  FPS:{fps:.1f}"
    cv2.putText(frame, status, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (30, 240, 30), 2)

    try:
        objects = camera.getRecognitionObjects()
    except Exception:
        objects = []

    for obj in objects:
        box = get_image_box(obj)
        label = get_object_label(obj)
        if box is None:
            continue
        x, y, w, h = box
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 210, 255), 2)
        cv2.putText(frame, label, (x, max(18, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 210, 255), 1)


def get_image_box(obj):
    try:
        position = obj.getPositionOnImage()
        size = obj.getSizeOnImage()
    except Exception:
        return None

    x = int(position[0] - size[0] / 2)
    y = int(position[1] - size[1] / 2)
    w = int(size[0])
    h = int(size[1])
    return x, y, w, h


def get_object_label(obj):
    try:
        model = obj.getModel()
        if model:
            return str(model)
    except Exception:
        pass
    try:
        return str(obj.getId())
    except Exception:
        return "object"


if __name__ == "__main__":
    main()
