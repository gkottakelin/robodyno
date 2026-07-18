# Desktop Sorter — 桌面组 (Robodyno + Webots)

Vision-guided desktop sorting system for the Zhejiang Province College Student
Engineering Practice and Innovation Competition (浙江省大学生工程实践与创新能力大赛).

Uses a SCARA 4-DOF robot with vacuum gripper and an overhead camera to identify,
pick, and sort workpieces by shape into designated tray positions.

## Scene Layout

```
        PICK TABLE                  PLACE TABLE
   ┌─────────────────┐      ┌──────────────────┐
   │ Box1    Box2     │      │  ┌────────────┐  │
   │ [C] [C] [F] [P] │      │  │  TOP TRAY  │  │
   │ Box3    Box4     │      │  └────────────┘  │
   │ [Y] [T] [X] [L] │  ──► │  ┌────────────┐  │
   └─────────────────┘      │  │ BOTTOM TRAY │  │
                            │  └────────────┘  │
       CAMERA               └──────────────────┘
         ↓
   [camera_viewer]              [SCARA ROBOT]
      vision data ──────────►  [desktop_sorter]
```

## Shape Types (9)

| Shape          | Tray Slot (top)   | Tray Slot (bottom) |
|----------------|-------------------|--------------------|
| Cube           | (+0.055, -0.055)  | (+0.055, -0.055)  |
| Cuboid         | (+0.055, +0.055)  | (+0.055, +0.055)  |
| FivePointed    | (-0.055, -0.055)  | (-0.055, -0.055)  |
| Pentagonal     | (+0.000, -0.055)  | (+0.000, -0.055)  |
| Cylindrical    | (-0.055, +0.055)  | (-0.055, +0.055)  |
| Triangular     | (+0.000, +0.055)  | (+0.000, +0.055)  |
| Parallelogram  | (-0.055, +0.000)  | (-0.055, +0.000)  |
| Cruciform      | (+0.000, +0.000)  | (+0.000, +0.000)  |
| Quincunx       | (+0.055, +0.000)  | (+0.055, +0.000)  |

## File Structure

```
robodyno/robodyno/
├── controllers/
│   ├── desktop_sorter/          # SCARA pick-and-place controller
│   │   └── desktop_sorter.py
│   ├── camera_viewer/           # Overhead camera vision system
│   │   └── camera_viewer.py
│   └── ...
├── worlds/
│   ├── competition.wbt          # Main competition scene (8 diverse shapes)
│   ├── competition_v1.wbt       # Alternative competition scene
│   ├── competition_full.wbt     # Full demo with all 9 types supported
│   ├── province_scene.wbt       # Provincial-level scene (4 types)
│   └── ...
├── robocom_webots/              # Workpiece protos (Cube, Cuboid, etc.)
└── robots/                      # Robot protos (FourDofScaraRobot)
```

## Requirements

- Webots R2023b or newer
- Python 3.8+
- robodyno >= 1.7.1
- OpenCV (`opencv-python`) + NumPy (for camera vision)
- NCNN (optional, for neural network shape classification)

```bash
pip install robodyno>=1.7.1 opencv-python numpy
```

## How to Run

1. Open Webots
2. File → Open World → Select `worlds/competition.wbt`
3. Both controllers (`desktop_sorter` + `camera_viewer`) start automatically
4. The camera viewer opens a preview window showing detected objects
5. The sorter waits for stable vision detection, then begins sorting
6. To stop: close the preview window or stop simulation in Webots

## Architecture

### Controllers (run simultaneously in Webots)

1. **camera_viewer** — Overhead camera processing
   - Captures raw images from the top camera
   - Detects objects by color (HSV thresholding) and shape (contour matching against STL templates)
   - Computes world-frame coordinates via calibrated table bounds
   - Writes `vision_latest.json` with detection results (atomic write)

2. **desktop_sorter** — SCARA robot motion control
   - Reads `vision_latest.json` to get object positions, shapes, and colors
   - Waits for stable vision before starting
   - Builds a dynamic task list ordered by distance
   - Executes pick-and-place using planar IK with smooth interpolation
   - Reports status to `desktop_sorter_status.json`

### Vision Pipeline (camera_viewer)

```
Camera Image (640x480)
  → 180° rotation (camera mounted upside-down)
  → Pick table ROI extraction
  → Per-color HSV thresholding (8 colors)
  → Contour extraction and filtering
  → Shape classification:
      - STL template matching (Hu moments + feature vector)
      - Fallback: basic geometry (vertices, solidity, circularity)
  → Angle estimation (minAreaRect for rect-like, farthest-point for others)
  → Pixel-to-world coordinate transform
  → JSON output
```

### Motion Planning (desktop_sorter)

```
Target (x, y) → Planar IK → Joint angles (q1, q2)
  → Smooth-step interpolation (s-curve easing)
  → Position-mode motor commands
  → Vacuum gripper on/off for pick/release
```

## Competition Levels

| Level      | World File          | Objects | Types  |
|------------|---------------------|---------|--------|
| Provincial | province_scene.wbt  | 8       | 4      |
| Competition| competition.wbt     | 8       | 8      |
| Full Demo  | competition_full.wbt| 8       | 8      |

## Status File Format (desktop_sorter_status.json)

```json
{
  "status": "running",
  "completed": 3,
  "total": "dynamic",
  "current_task": "cube (red)"
}
```

Status values: `started` → `running` → `done` / `error`
