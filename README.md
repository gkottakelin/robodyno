# Robodyno Webots 桌面分拣仿真系统

![robodyno_webots](https://img.shields.io/badge/robodyno_webots-v2.0.0-aquamarine) [![Python](https://img.shields.io/pypi/pyversions/robodyno)](https://www.python.org/downloads/) [![webots_version](https://img.shields.io/badge/webots-2023b-orange)](https://cyberbotics.com/) [![robodyno_version](https://img.shields.io/badge/robodyno->=1.7.1-green)](https://pypi.org/project/robodyno/) [![](https://img.shields.io/badge/license-Apache-000000.svg)](http://www.apache.org/licenses/)

基于 Webots 仿真平台与 Robodyno 模块化机器人组件构建的**视觉引导桌面分拣系统**。系统使用四自由度 SCARA 机械臂 + 真空吸盘 + 顶部俯视相机，自动识别取料区内工件的形状、颜色、坐标与角度，并将其分拣码放至指定托盘槽位。

---

## 一、赛题背景

**赛事**：浙江省大学生工程实践与创新能力大赛 — 桌面组

**任务**：在桌面场景中，机械臂需从取料区抓取多种形状的工件，按形状分类码放到码放区托盘的对应槽位。要求：

- 视觉自主识别工件形状、坐标、颜色与姿态角；
- 机械臂自主完成「定位 → 下降 → 吸取 → 抬升 → 搬运 → 释放」全流程；
- 多形状、多颜色工件混排时仍能稳定、精准地完成分拣。

**支持的工件类型（9 种）**：

| 形状 | 英文名 | 备注 |
|------|--------|------|
| 正方形 | Cube | 矩形类，minAreaRect 估角 |
| 长方形 | Cuboid | 矩形类，minAreaRect 估角 |
| 圆柱 | Cylindrical | 角度自由 |
| 三角形 | Triangular | 120° 等效周期 |
| 五边形 | Pentagonal | 72° 等效周期 |
| 五角星 | FivePointed | 72° 等效周期 |
| 平行四边形 | Parallelogram | 180° 等效周期 |
| 十字形 | Cruciform | 90° 等效周期，最近点估角 |
| 梅花形 | Quincunx | 90° 等效周期 |

**支持的工件颜色（8 种）**：红、橙、黄、绿、青、蓝、紫、粉。

**场景级别**：

| 级别 | 世界文件 | 工件数 | 形状种类 |
|------|----------|--------|----------|
| 省赛 | `worlds/province_scene.wbt` | 8 | 4 |
| 国赛 | `worlds/competition.wbt` | 8 | 8 |
| 完整演示 | `worlds/competition_full.wbt` | 8 | 8 |
| 现场演示批注版 | `worlds/现场演示_分段批注版.wbt.txt` | 8 | 4 |

---

## 二、目录结构

```
robodyno/robodyno/
├── controllers/                         # Webots 控制器（核心代码）
│   ├── desktop_sorter/                  # ★ SCARA 分拣主控制器
│   │   ├── desktop_sorter.py            #   机械臂运动控制（1139 行）
│   │   ├── desktop_sorter_status.json   #   运行状态输出
│   │   └── README.md                    #   控制器详细说明
│   ├── camera_viewer/                   # ★ 顶部相机视觉控制器
│   │   ├── camera_viewer.py             #   视觉识别管线（961 行）
│   │   ├── vision_latest.json           #   识别结果输出
│   │   └── camera_preview_latest.png    #   最新预览快照
│   ├── measure_fk/                      # 正运动学标定工具
│   └── slider_test/                     # 丝杠电机调试工具
│
├── worlds/                              # 仿真世界文件
│   ├── competition.wbt                  # 国赛场景（8 形状）
│   ├── competition_v1.wbt               # 国赛场景（变体）
│   ├── competition_full.wbt             # 完整演示场景
│   ├── province_scene.wbt               # 省赛场景（4 形状）
│   └── 现场演示_分段批注版.wbt.txt       # 现场复制批注说明
│
├── robots/                              # 机器人整机 PROTO
│   ├── FourDofScaraRobot.proto          # ★ 四自由度 SCARA（本项目使用）
│   ├── FourDofScaraRobot1.proto         # SCARA 构型二
│   ├── FourDofScaraRobot2.proto         # SCARA 构型三
│   ├── SixDofCollaborationRobot.proto   # 六自由度协作机械臂
│   ├── ThreeDofCartesianRobot.proto     # 三自由度笛卡尔机器人
│   └── ThreeDofDeltaRobot.proto         # 三自由度 Delta 机器人
│
├── joints/                              # 关节电机 PROTO
│   ├── Pro_JP12 / Pro_JP15 / Pro_JP44 / Pro_JP66.proto
│   └── SliderModule.proto               # 直线滑台模组
│
├── endEffector/                         # 末端执行器 PROTO
│   ├── Vacuum_Gripper.proto             # ★ 真空吸盘（本项目使用）
│   ├── RobotHand.proto                  # 双指夹持器
│   ├── HBPencil.proto                   # 绘图铅笔
│   └── CoordinateSystem.proto           # 坐标系标记
│
├── camera/                              # 相机 PROTO
│   ├── RobodynoCamera.proto             # ★ 顶部相机（本项目使用）
│   └── CameraBracket.proto              # 相机支架
│
├── conveyorBelt/                        # 传送带 PROTO
├── robocom_webots/                      # 比赛工件 PROTO（Cube/Cuboid/...）+ 形状 STL
├── objects/                             # 机械结构件 PROTO（连杆、底板、支架等）
├── plugins/                             # Webots 插件（物理/远程控制/机器人窗口）
├── libraries/                           # 共享代码库（预留）
├── protos/                              # 其他 PROTO（预留）
└── README_1.md                          # Robodyno 官方 PROTO 使用说明
```

带 ★ 标记的为本项目赛题实现的核心文件。

---

## 三、技术方案

### 3.1 系统整体架构

系统采用**感知—决策—执行**三层架构，两个控制器在 Webots 中并行运行，通过 JSON 文件解耦通信：

```
        ┌───────────────────────┐
        │   顶部相机 (俯视)      │
        │   RobodynoCamera      │
        └──────────┬────────────┘
                   │ BGRA 图像流
                   ▼
   ┌──────────────────────────────┐
   │  camera_viewer.py (感知层)    │
   │  · HSV 颜色分割 (8 色)        │
   │  · STL 模板匹配 + 几何兜底    │
   │  · 像素→世界坐标变换          │
   │  · 姿态角估计                 │
   └──────────────┬───────────────┘
                  │ vision_latest.json (原子写)
                  ▼
   ┌──────────────────────────────┐
   │  desktop_sorter.py (决策+执行)│
   │  · 任务调度（按距离排序）     │
   │  · 平面 IK 解析（余弦定理）   │
   │  · 关节限位惩罚选解           │
   │  · S 曲线平滑插值             │
   │  · L 型安全走廊搬运           │
   │  · 真空吸盘控制 + 工件绑定    │
   └──────────────┬───────────────┘
                  │ 电机指令 / Supervisor 节点操作
                  ▼
        ┌───────────────────────┐
        │  FourDofScaraRobot    │
        │  + Vacuum_Gripper     │
        │  + Supervisor 模式    │
        └───────────────────────┘
```

### 3.2 视觉感知（camera_viewer.py）

视觉管线位于 `controllers/camera_viewer/camera_viewer.py`，主要流程：

1. **图像预处理**：相机倒装，对原始 640×480 BGRA 图像旋转 180° 校正。
2. **取料台 ROI 提取**：采用 `fixed_calibration` 模式，按预标定的桌面边界截取取料区，避免码放区与桌面边缘误识别。
3. **HSV 颜色分割**：8 种颜色分别设定上下限，红色分两段（0–8 与 170–180），生成 8 张二值掩膜。
4. **掩膜清洗**：3×3 开运算去噪 + 7×7 闭运算补洞。
5. **轮廓提取与过滤**：`RETR_EXTERNAL` 外轮廓，面积阈值 450 px。
6. **形状分类（两级）**：
   - **一级：STL 模板匹配**。从 `robocom_webots/shapes/*.STL` 读取 9 种形状的三角面片，投影为 180×180 模板轮廓；用 `cv2.matchShapes`（Hu 矩，CONTOURS_MATCH_I1）+ 自定义 6 维特征向量（圆度、实度、长宽比、倾斜度、凹陷数、径向方差）加权融合评分。
   - **二级：基本几何兜底**。当 STL 文件缺失时，按顶点数、圆度、实度、凹陷数判定形状。
   - **十字/梅花专项判别**：当 top-3 候选含 Cruciform/Quincunx 且凹陷数 ≥ 3 时，用「直线性边长占比」≥ 0.62 判为十字，否则为梅花。
7. **姿态角估计**：按形状分别处理
   - 圆柱：恒为 0°（角度自由）
   - 正方形/长方形：`minAreaRect` 长边方向，对称归一化到 [-90°, 90°]
   - 十字：最近点角度，90° 等效归一化
   - 五边/五角星：最远点角度，72° 等效归一化
   - 三角形：最远点角度，120° 等效归一化
   - 平行四边形：最远点角度，180° 等效归一化
   - 梅花：最远点角度，90° 等效归一化
8. **像素→世界坐标变换**：
   ```
   world_x = table_center_x + (px - center_px_x) * meters_per_px_x
   world_y = table_center_y - (py - center_px_y) * meters_per_px_y
   ```
   标定参数：桌面中心 (-0.025, 0.245)，尺寸 0.30×0.14 m。
9. **结果输出**：原子写（先写 `.tmp` 再 `os.replace`）至 `vision_latest.json`，包含每个工件的 shape / color / world_center_m / angle_z_deg / confidence。
10. **可视化**：OpenCV 窗口实时显示桌面框、十字标定中心、检测点、姿态箭头与标签块；按 `s` 抓拍，按 `q`/`Esc` 退出。

### 3.3 运动规划与控制（desktop_sorter.py）

控制器位于 `controllers/desktop_sorter/desktop_sorter.py`，运行于 `Supervisor` 模式，主要技术点：

#### 3.3.1 平面运动学逆解（IK）

基于余弦定理的 2 连杆平面 IK：

```
cos_q2 = (r² - L1² - L2²) / (2·L1·L2)
q2 = ± acos(cos_q2)        # 上肘 / 下肘两解
q1 = atan2(y_rel, x_rel) - atan2(L2·sin(q2), L1 + L2·cos(q2))
```

- 自动将世界坐标变换到机械臂局部坐标（读取 `rotation` 字段获取 base_yaw）
- 减去 J2 肩部偏移 (0.055, -0.003)
- 臂展保护：超出 `[|L1-L2|+5mm, L1+L2-5mm]` 时按比例缩放
- 末端姿态：`q4 = tool_yaw_local - q1 - q2`

#### 3.3.2 双解裁决与关节限位惩罚

对肘上 / 肘下两解计算 `joint_limit_penalty`：

- J2 越界权重 ×10（最易碰撞丝杠立柱）
- J3 越界权重 ×2
- J4 越界权重 ×0.5

按 `(penalty, joint_distance)` 字典序选最优解，规避奇点姿态。

#### 3.3.3 S 曲线平滑插值

使用 `smoothstep(t) = t²·(3-2t)` 实现起停速度归零的 S 型加减速，避免急停急起造成的工件抖动与吸盘脱落。

#### 3.3.4 丝杠 Z 轴独立控制

修正 SliderModule.proto 中 `multiplier = 0.0016` 的单位换算：

```python
motor_command = (slider_pose_m - 0.245) / 0.0016
```

XY 水平运动与 Z 轴升降完全解耦，下降/抬升时保持 XY 不变。

#### 3.3.5 L 型安全走廊搬运

为避免机械臂走直线斜穿取料区/码放区碰乱其他工件，搬运路径强制：

```
取料点 → (px, safe_y=0.335) → (center_x=-0.020, safe_y) → (tx, safe_y) → 码放点
```

#### 3.3.6 二次确认机制

由于命令位置与真实电机位置存在惯性滞后，引入两层确认：

- **下降前确认** (`wait_until_tool_over_xy`)：XY 误差 ≤ 10mm 且连续 10 帧稳定才允许丝杠下降
- **释放前确认** (`wait_until_tool_over_place`)：XY 误差 ≤ 8mm 且连续 8 帧稳定才允许关吸盘

真实位置来自各电机的 `PositionSensor`，而非理论命令值。

#### 3.3.7 工件刚性跟随（Supervisor）

抓取瞬间通过 `capture_attach_offset` 锁定「工具—工件」相对偏移；之后每一步用 `tool_world_xyz_from_pose` 计算真实吸盘位姿（含 Z 轴编码器反馈），减去偏移得到工件目标位姿，并通过 `node.setTranslation` / `node.setRotation` / `node.resetPhysics()` 持续更新。这样工件严格跟随吸盘运动，规避了 Webots 物理引擎中吸盘接触力不稳定的问题。

#### 3.3.8 任务调度

- 优先从 `vision_latest.json` 加载任务（`USE_VISION=True` 时）
- 否则使用 `DEFAULT_TASKS`（按 `province_scene.wbt` 中 8 个工件的固定位置）
- 视觉任务按 `(-y, x)` 排序（先远后近、先左后右），减少运动交叉
- 每种形状最多取 2 件，按奇偶分配到 top/bottom 托盘
- 仅识别取料台范围内的工件（`PICK_TABLE_X_RANGE` / `PICK_TABLE_Y_RANGE`），防止误把码放盘上的工件当作待抓件

#### 3.3.9 状态上报

通过 `desktop_sorter_status.json` 实时上报：`started → running → done / error`，含 `completed` / `total` / `current_task` 字段，便于外部 UI 监控。

### 3.4 场地与码放策略

码放盘采用 3×3 网格布局，两个托盘上下分布：

```
        码放区 (place_table)
   ┌─────────────────────┐
   │  TOP TRAY (上托盘)   │   中心 (-0.2625, +0.0875)
   │  ┌───┬───┬───┐      │
   │  │Cr │Tr │Cu │      │   槽位间距 55mm
   │  ├───┼───┼───┤      │
   │  │Pa │Pe │Cb │      │
   │  ├───┼───┼───┤      │
   │  │Qu │Fi │Cy │      │
   │  └───┴───┴───┘      │
   ├─────────────────────┤
   │  BOTTOM TRAY (下托盘)│  中心 (-0.2625, -0.0875)
   │  └───┴───┴───┘      │
   └─────────────────────┘
```

每种形状在上下托盘各占一个对应槽位，槽位坐标见 `TRAY_LOCAL_TARGETS_M`。

---

## 四、效果

### 4.1 功能完成度

- ✅ 省赛场景（4 形状 × 2 件 = 8 件）稳定分拣完成
- ✅ 国赛场景（8 形状 × 1 件 = 8 件）支持
- ✅ 完整演示场景（9 形状全支持）
- ✅ 8 色工件 HSV 颜色识别
- ✅ 视觉自主识别 + 默认任务表双模式

### 4.2 精度指标

| 指标 | 实测值 |
|------|--------|
| 视觉坐标误差 | < 5 mm |
| 视觉形状识别置信度 | 0.97 – 0.99 |
| 下降定位 XY 容差 | ≤ 10 mm |
| 释放定位 XY 容差 | ≤ 8 mm |
| 工件厚度 | 15 mm |
| 单件分拣耗时 | 约 12–15 秒 |

### 4.3 工程稳定性

- 丝杠单位换算修正（`multiplier=0.0016`），解决「命令 -0.28 仅产生 0.45mm 位移」问题
- 角度计算底层逻辑重构，修复物块振动不收敛问题
- 摄像头位置与 ROI 标定修正，坐标误差从厘米级降至 5mm 内
- L 型走廊路径 + 二次确认机制，消除「抄近道」与「提前撒手」
- 工件刚性跟随，消除视觉上零件脱离吸盘的问题
- 关节限位惩罚，避免 J2 碰撞丝杠立柱

### 4.4 运行状态示例

`desktop_sorter_status.json` 运行结束时：

```json
{
  "status": "running",
  "completed": 8,
  "total": 8,
  "current_task": "PENTAGONAL_2:Pentagonal->bottom"
}
```

`vision_latest.json` 检测结果示例（节选）：

```json
{
  "shape": "Pentagonal",
  "color": "green",
  "world_center_m": [0.08572, 0.2846, 0.01],
  "angle_z_deg": 0.7,
  "confidence": 0.99
}
```

---

## 五、配置方法

### 5.1 软件环境

| 组件 | 版本要求 | 说明 |
|------|----------|------|
| Webots | R2023b 或更新 | 仿真平台 |
| Python | 3.8+ | 控制器语言 |
| robodyno | ≥ 1.7.1 | Robodyno Python SDK |
| opencv-python | 任意稳定版 | 视觉处理 |
| numpy | 任意稳定版 | 数值计算 |

### 5.2 安装步骤

1. **安装 Webots**：参考 [Webots 官方文档](https://cyberbotics.com/doc/guide/installation-procedure)，安装 R2023b 或更新版本。

2. **安装 Python 依赖**：

   ```bash
   pip install robodyno>=1.7.1 opencv-python numpy
   ```

3. **设置 Webots Python 环境**（若 Webots 未自动识别系统 Python）：
   - 打开 `Tools → Preferences → Python command`
   - 填入系统 Python 解释器路径，例如 `python` 或 `C:/Python39/python.exe`

### 5.3 关键配置项

#### 视觉控制器（camera_viewer.py）

| 配置项 | 默认值 | 说明 |
|--------|--------|------|
| `IMAGE_ROTATE_180` | `True` | 相机倒装补偿 |
| `TABLE_BOUNDS_MODE` | `"fixed_calibration"` | 取料台 ROI 模式 |
| `PICK_TABLE_CENTER_M` | `(-0.025, 0.245)` | 取料台世界中心 |
| `PICK_TABLE_SIZE_M` | `(0.300, 0.140)` | 取料台尺寸 |
| `TOP_CAMERA_TRANSLATION_M` | `(-0.032, 0.245, 0.45)` | 相机标定位置 |
| `MIN_OBJECT_AREA_PX` | `450` | 最小工件面积阈值 |
| `VISION_PREVIEW` 环境变量 | `1` | 设为 `0` 关闭预览窗口 |

#### 分拣控制器（desktop_sorter.py）

| 配置项 | 默认值 | 说明 |
|--------|--------|------|
| `USE_VISION` | `False` | 是否从视觉结果加载任务（省赛固定场景建议 `False`）|
| `LINK_1_M` / `LINK_2_M` | `0.215` | 有效平面臂长（含末端结构） |
| `SLIDER_PICK_POSE_M` | `0.120` | 取料下降高度 |
| `SLIDER_PLACE_POSE_M` | `0.120` | 码放下降高度 |
| `SLIDER_SAFE_POSE_M` | `0.245` | 水平搬运安全高度 |
| `CARRY_ROUTE_SAFE_Y_M` | `0.335` | 搬运走廊 Y 坐标 |
| `J2_MOTOR_MIN/MAX_RAD` | `-1.55 / 1.55` | J2 关节限位（防碰撞） |
| `RELEASE_XY_TOL_M` | `0.008` | 释放前 XY 容差 |
| `DESCENT_XY_TOL_M` | `0.010` | 下降前 XY 容差 |

### 5.4 场地配置

码放盘槽位、托盘中心、取料台范围等场地参数集中在 `desktop_sorter.py` 顶部的常量区，迁移到不同尺寸场地时按注释说明调整即可：

```python
TRAY_CENTERS_M = {
    "top":    (-0.2625,  0.0875),
    "bottom": (-0.2625, -0.0875),
}

TRAY_LOCAL_TARGETS_M = {
    "Cruciform":    (-0.055,  0.055),
    "Cuboid":       ( 0.055,  0.000),
    "Cube":         ( 0.055,  0.055),
    # ... 其余形状见源码
}
```

---

## 六、使用方法

### 6.1 快速启动

1. 打开 Webots
2. `File → Open World` → 选择 `worlds/province_scene.wbt`（省赛场景）
3. 点击工具栏 ▶ 运行仿真
4. 两个控制器（`desktop_sorter` + `camera_viewer`）自动启动
5. 弹出 OpenCV 预览窗口显示视觉识别结果
6. 机械臂等待视觉稳定后自动开始分拣
7. 停止：关闭预览窗口（按 `q` 或 `Esc`）或点击 Webots ⏸

### 6.2 切换场景

| 需求 | 打开文件 |
|------|----------|
| 省赛 4 形状固定演示 | `worlds/province_scene.wbt` |
| 国赛 8 形状分拣 | `worlds/competition.wbt` |
| 完整 9 形状演示 | `worlds/competition_full.wbt` |

### 6.3 启用视觉自主模式

编辑 `controllers/desktop_sorter/desktop_sorter.py`：

```python
USE_VISION = True   # 原为 False
```

此时机械臂会从 `vision_latest.json` 读取工件位置、形状、角度，自主生成任务列表，适用于工件位置随机的场景。

### 6.4 调试工具

| 控制器 | 用途 |
|--------|------|
| `controllers/slider_test/` | 丝杠电机基础运动测试 |
| `controllers/measure_fk/`  | 丝杠位置到吸盘平面 Z 高度的正运动学标定 |

### 6.5 状态监控

运行过程中可实时查看两个 JSON 文件：

- `controllers/camera_viewer/vision_latest.json` — 视觉识别结果
- `controllers/desktop_sorter/desktop_sorter_status.json` — 分拣进度

---

## 七、移植方法

### 7.1 整体迁移到新机器

1. 复制整个 `robodyno/robodyno/` 目录到目标机器。
2. 安装软件环境（见第五节）。
3. 用 Webots 打开 `worlds/*.wbt`，场景文件内 EXTERNPROTO 使用**相对路径**（如 `../robots/FourDofScaraRobot.proto`），无需修改即可运行。

### 7.2 现场快速重建场景

`worlds/现场演示_分段批注版.wbt.txt` 提供了**分段批注版**场景复制说明，适合比赛现场从零搭建：

1. 在 Webots 中新建一个 World；
2. 按「第 1 步 → 第 8 步」顺序复制代码段：
   - 第 1 步：导入所有 EXTERNPROTO
   - 第 2 步：WorldInfo / Viewpoint / Background / DirectionalLight
   - 第 3 步：地面与三块底板
   - 第 4 步：取料台 pick_table
   - 第 5 步：码放区、取料小格、码放托盘
   - 第 6 步：8 个工件
   - 第 7 步：SCARA 机械臂 + 真空吸盘
   - 第 8 步：顶部相机
3. **路径适配**：批注版使用绝对路径 `D:/robodyno1/robodyno/robodyno/...`，需替换为目标机器实际路径，或改为相对路径。

### 7.3 更换机械臂构型

本项目使用 `FourDofScaraRobot`（构型一）。如需更换为其他构型，修改世界文件中的 PROTO 引用：

```diff
- EXTERNPROTO "../robots/FourDofScaraRobot.proto"
+ EXTERNPROTO "../robots/FourDofScaraRobot1.proto"   # 构型二
```

并相应调整 `desktop_sorter.py` 中的 `LINK_1_M` / `LINK_2_M` / `J2_ORIGIN_X_LOCAL_M` 等几何参数。

### 7.4 适配新工件形状

1. 在 `robocom_webots/` 下新建 `NewShape.proto` 与 `shapes/new_shape.STL`；
2. 在 `camera_viewer.py` 的 `SHAPE_TEMPLATE_SPECS` 中追加模板：
   ```python
   ("NewShape", "new_shape", "new_shape.STL"),
   ```
3. 在 `desktop_sorter.py` 的 `TRAY_LOCAL_TARGETS_M`、`SHAPE_DEF_PREFIXES`、`OBJECT_HEIGHT_M` 中追加对应条目；
4. 调整 `measure_angle_deg` 中的角度估计分支。

### 7.5 适配新场地尺寸

修改 `desktop_sorter.py` 顶部常量：

- `TRAY_CENTERS_M` — 两个托盘中心
- `TRAY_LOCAL_TARGETS_M` — 槽位局部坐标（按 3×3 网格调整间距）
- `PICK_TABLE_X_RANGE` / `PICK_TABLE_Y_RANGE` — 取料台范围
- `CARRY_ROUTE_SAFE_Y_M` — 搬运走廊 Y 坐标

同步修改 `camera_viewer.py` 的 `PICK_TABLE_CENTER_M` / `PICK_TABLE_SIZE_M` / `TOP_CAMERA_TRANSLATION_M`。

---

## 八、关键问题与解决记录

| 问题 | 根因 | 解决方案 |
|------|------|----------|
| 丝杠几乎不升降 | SliderModule.proto 中 `multiplier=0.0016`，命令单位非米 | `motor_command = (pose_m - 0.245) / 0.0016` |
| 物块振动不收敛 | 角度计算底层逻辑错误 | 重构角度估计，按形状分策略 + 等效周期归一化 |
| 视觉坐标误差大 | 摄像头位置与 ROI 未标定 | 修正 `TOP_CAMERA_TRANSLATION_M` + 固定桌面标定 |
| 零件「抄近道」脱离吸盘 | 命令位置领先真实位置 | 引入 PositionSensor 反馈 + 工件刚性跟随 |
| J2 碰撞丝杠立柱 | IK 选解未考虑碰撞 | `joint_limit_penalty` 惩罚越界解 |
| 释放时工件被抛出 | 运动惯性中提前关吸盘 | `wait_until_tool_over_place` 二次确认 |
| 码放盘工件被误识别为待抓件 | 视觉无范围限制 | `PICK_TABLE_X/Y_RANGE` 过滤 |

---

## 九、致谢

- [Robodyno](https://github.com/robodyno/robodyno) — 模块化机器人硬件与仿真模型
- [Webots](https://cyberbotics.com/) — 开源机器人仿真平台
- [OpenCV](https://opencv.org/) — 计算机视觉库

---

## License

本项目遵循 Apache License 2.0。Robodyno 仿真模型 Proto 文件遵循 Creative Commons Attribution 4.0 International License。
