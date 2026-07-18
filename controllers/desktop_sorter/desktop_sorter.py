#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！禁止删除注释！！！！！！！！！！！！！！！
author: Mulan
data: 2026-06-14
桌面组 FourDofScaraRobot 分拣控制器

放置位置：
    robodyno/controllers/desktop_sorter/desktop_sorter.py

模型：
    FourDofScaraRobot
    firstJointId  "0x10"  -> 丝杠电机设备名  "0x10::slider"
    secondJointId "0x11"  -> 关节电机设备名  "0x11::motor"
    thirdJointId  "0x12"  -> 关节电机设备名  "0x12::motor"
    fourthJointId "0x13"  -> 末端电机设备名  "0x13::motor"

重点修正：
1. 不再用 "0x11" / "0x12" / "0x13" 找电机，而是使用真实设备名 "::motor"。
2. 丝杠单独升降：XY 水平移动和 Z 轴升降完全分开。
3. 自动读取机械臂安装角 rotation，世界坐标会先转换到机械臂局部坐标再做 IK。
4. 码放盘坑位按真实 3x3 布局重排，避免形状放错。
5. 先下降接触、再吸盘开启、再搬运，避免视觉上凌空抓取。
6. 搬运时零件 XY 不再单独直线插值，而是每一步跟随吸盘正运动学轨迹。
6. 修正 SliderModule 的 multiplier=0.0016：丝杠命令必须用“电机单位”，不是米。
"""

import json
import math
import os
import traceback
from controller import Supervisor

BASE_DIR = os.path.dirname(__file__)
STATUS_FILE = os.path.join(BASE_DIR, "desktop_sorter_status.json")
VISION_RESULT_FILE = os.path.join(BASE_DIR, "vision_latest.json")

# 当前省赛固定场景建议先关闭视觉文件，避免读取旧的 vision_latest.json
# 把码放盘上的工件误当成取料区工件。需要随机取料时再改成 True。
USE_VISION = False

# 只允许视觉结果落在取料台范围内，防止把码放盘/已放工件识别成待抓工件。
PICK_TABLE_X_RANGE = (-0.18, 0.13)
PICK_TABLE_Y_RANGE = (0.17, 0.32)

# 搬运路径防抄近道：
# 抓取后不允许直接从取料点斜线切到码放点，而是先走到上方安全走廊，
# 再横向移动，最后下到目标坑位上方。
CARRY_ROUTE_SAFE_Y_M = 0.335
CARRY_ROUTE_CENTER_X_M = -0.020
CARRY_ROUTE_STEP_DURATION_S = 1.35

# ========================= 机械臂参数 =========================

# FourDofScaraRobot 的 PROTO 里 upperarmLength/forearmLength 是 0.15，
# 但端部结构件会让有效平面臂长更长。。
LINK_1_M = 0.215
LINK_2_M = 0.215

# 第二关节中心相对 Robot 原点的局部偏移，按当前 FourDofScaraRobot 结构估计。
J2_ORIGIN_X_LOCAL_M = 0.055
J2_ORIGIN_Y_LOCAL_M = -0.003

# 关节方向。若出现整体左右镜像，再只改这三个符号，不要改坐标表。
JOINT2_SIGN = -1.0
JOINT3_SIGN = 1.0
JOINT4_SIGN = 1.0

# ------------------------- 关节安全限位 -------------------------
# 重点限制 0x11::motor，也就是靠近丝杠立柱的第一个水平旋转关节。
# 这个关节转得太靠左/靠右会让黑色关节壳体碰到丝杠滑台和立柱。
# 单位：弧度。数值越窄越安全，但工作空间越小。
J2_MOTOR_MIN_RAD = -1.55
J2_MOTOR_MAX_RAD = 1.55

# 下面两个是保险限位，正常不会影响任务；防止 IK 选到极端折叠姿态。
J3_MOTOR_MIN_RAD = -2.65
J3_MOTOR_MAX_RAD = 2.65
J4_MOTOR_MIN_RAD = -3.05
J4_MOTOR_MAX_RAD = 3.05

# 旋转关节速度限制。用于 setVelocity，也用于自动延长每段动作时间，
# 防止电机真实位置落后于程序计算位置，造成零件“抄近道”。
J2_SPEED_RAD_S = 1.50
J3_SPEED_RAD_S = 1.50
J4_SPEED_RAD_S = 2.20
ARM_MOVE_EXTRA_SETTLE_S = 0.22

# 释放前二次确认：
# 机械臂高速运动时，命令位置已到但真实电机可能还有滞后。
# 只有真实吸盘 XY 连续多帧接近码放点，才允许关闭吸盘。
RELEASE_XY_TOL_M = 0.008
RELEASE_STABLE_STEPS = 8
RELEASE_WAIT_TIMEOUT_S = 2.50

# 下降前二次确认：
# 只有真实吸盘 XY 已经到达取件/放件目标上方，才允许丝杠下降。
DESCENT_XY_TOL_M = 0.010
DESCENT_STABLE_STEPS = 10
DESCENT_WAIT_TIMEOUT_S = 3.00

# 丝杠/滑台关键修正：
# SliderModule.proto 里的 LinearMotor 写了 multiplier 0.0016。
# Webots 的 setPosition() 传入的是“电机命令单位”，实际滑台位移 = 命令 * 0.0016 米。
# 所以前几版用 -0.280 只会产生约 -0.45 mm 位移，看起来几乎不升降。
SLIDER_MULTIPLIER_M_PER_CMD = 0.0016
SLIDER_SPEED_CMD_PER_S = 25.0
SLIDER_EXTRA_SETTLE_S = 0.45

# SliderModule 的可见滑台位姿范围来自 initPose 注释：[0.25, 0.045]，单位 m。
# PROTO 里 SliderJoint endPoint 初始 translation = sliderPose - 0.245。
SLIDER_POSE_TO_JOINT_OFFSET_M = 0.245
SLIDER_LOWEST_POSE_M = -0.040    # 允许更低的丝杠位姿；若压桌面，调回 -0.020
SLIDER_TOP_POSE_M = 0.250      # 最高
SLIDER_SAFE_POSE_M = 0.245     # 水平搬运安全高度；接近最高位，避免水平移动时碰乱工件
# 下降高度修正：数值越大，吸盘越高；数值越小，吸盘越低。
# 之前 0.047 会让末端压到码放盘/桌面。
# 取料区工件在取料盒上，允许稍低；码放区托盘更低，所以放置高度要比取料更高。
SLIDER_PICK_POSE_M = 0.120  # 取料下降高度；数值越小吸盘越低，本版放低一点
SLIDER_PLACE_POSE_M = 0.120  # 码放下降高度；数值越小吸盘越低，本版放低一点

# 搬运时 Supervisor 给工件设置的是“工件底面高度”，不是吸盘高度。
# 之前 0.085 会把工件抬到连杆附近，导致穿模；0.050 可保证离台面约 3 cm 且不碰连杆。
# 注意：压桌面问题不要改这里，改 SLIDER_PICK_POSE_M / SLIDER_PLACE_POSE_M。
CARRY_Z_M = 0.075

# VacuumGripper 的 device 原点不等于白色吸盘接触面。
# 真实跟随时给工件 Z 方向额外下移，避免工件被吸到末端关节上方。
# 不再使用大幅 Z 偏置；之前负偏置会导致吸住瞬间把零件拉到桌面下方。
ATTACH_Z_BIAS_M = 0.000

# 工件厚度，当前 9 种工件基本都是 15mm。
DEFAULT_OBJECT_HEIGHT_M = 0.015
OBJECT_HEIGHT_M = {
    "Cube": 0.015,
    "Cuboid": 0.015,
    "FivePointed": 0.015,
    "Pentagonal": 0.015,
    "Cylindrical": 0.015,
    "Triangular": 0.015,
    "Triangle": 0.015,
    "Parallelogram": 0.015,
    "Cruciform": 0.015,
    "Quincunx": 0.015,
}

# 世界文件里工件 translation z 是工件底面高度；取料盒上是 0.021/0.022。
PICK_BOTTOM_Z_M = 0.021
# 托盘在 place_table 上，坑底/托盘顶面附近约 0.013。
PLACE_BOTTOM_Z_M = 0.018

# ========================= 场地与码放盘 =========================

# 两个码放盘中心，与你的 province_scene.wbt 一致。
TRAY_CENTERS_M = {
    "top": (-0.2625, 0.0875),
    "bottom": (-0.2625, -0.0875),
}

# 码放盘 3x3 坑位布局。
# 按实测反馈修正：长方形与三角形坐标对调；五角星与平行四边形坐标对调。
# 坐标含义：x 从左到右为 -0.055, 0, +0.055；y 从下到上为 -0.055, 0, +0.055。
TRAY_LOCAL_TARGETS_M = {
    "Cruciform": (-0.055, 0.055),
    "Cuboid": (0.055, 0.000),   # 长方形：与三角形坐标对调，修正放到三角形坑的问题
    "Cube": (0.055, 0.055),
    "FivePointed": (0.000, -0.055),  # 五角星：与平行四边形坐标对调，修正放到平行四边形坑的问题
    "Pentagonal": (0.000, 0.000),
    "Triangular": (0.000, 0.055),
    "Triangle": (0.000, 0.055),
    "Quincunx": (-0.055, -0.055),
    "Parallelogram": (-0.055, 0.000),
    "Cylindrical": (0.055, -0.055),
}

# 需要让工件旋转适配坑位时，改这里。
TRAY_WORLD_YAW_RAD = {
    "Cruciform": 0.0,
    "Cuboid": 0.0,
    "Cube": 0.0,
    "FivePointed": 0.0,
    "Pentagonal": 0.0,
    "Triangular": 0.0,
    "Triangle": 0.0,
    "Quincunx": 0.0,
    "Parallelogram": 0.0,
    "Cylindrical": 0.0,
}

# 没有 vision_latest.json 时，按你 province_scene.wbt 里的 8 个工件执行。
DEFAULT_TASKS = (
    {"def": "CUBE_1", "shape": "Cube", "pick": (-0.130, 0.210, PICK_BOTTOM_Z_M), "tray": "top", "angle": 0.0},
    {"def": "CUBE_2", "shape": "Cube", "pick": (-0.065, 0.210, PICK_BOTTOM_Z_M), "tray": "bottom", "angle": 0.0},
    {"def": "CUBOID_1", "shape": "Cuboid", "pick": (0.015, 0.210, PICK_BOTTOM_Z_M), "tray": "top", "angle": 0.0},
    {"def": "CUBOID_2", "shape": "Cuboid", "pick": (0.080, 0.210, PICK_BOTTOM_Z_M), "tray": "bottom", "angle": 0.0},
    {"def": "FIVE_POINTED_1", "shape": "FivePointed", "pick": (-0.130, 0.280, 0.022), "tray": "top", "angle": 0.0},
    {"def": "FIVE_POINTED_2", "shape": "FivePointed", "pick": (-0.065, 0.280, 0.022), "tray": "bottom", "angle": 0.0},
    {"def": "PENTAGONAL_1", "shape": "Pentagonal", "pick": (0.015, 0.280, PICK_BOTTOM_Z_M), "tray": "top", "angle": 0.0},
    {"def": "PENTAGONAL_2", "shape": "Pentagonal", "pick": (0.080, 0.280, PICK_BOTTOM_Z_M), "tray": "bottom", "angle": 0.0},
)

SHAPE_DEF_PREFIXES = {
    "Cube": ("CUBE",),
    "Cuboid": ("CUBOID",),
    "FivePointed": ("FIVE_POINTED", "FIVEPOINTED"),
    "Pentagonal": ("PENTAGONAL",),
    "Cylindrical": ("CYLINDRICAL",),
    "Triangular": ("TRIANGULAR", "TRIANGLE"),
    "Triangle": ("TRIANGULAR", "TRIANGLE"),
    "Parallelogram": ("PARALLELOGRAM",),
    "Cruciform": ("CRUCIFORM",),
    "Quincunx": ("QUINCUNX",),
}

# ========================= 数学与辅助函数 =========================

def clamp(value, low, high):
    """将数值限制在 [low, high] 区间内"""
    return max(low, min(high, value))

def lerp(a, b, t):
    """线性插值：基于比例 t (0到1) 在 a 和 b 之间取值"""
    return a + (b - a) * t

def smoothstep(t):
    """平滑插值：让运动在起始和结束时速度放缓（S型曲线），避免机械臂急停急起"""
    return t * t * (3.0 - 2.0 * t)

def normalize_angle(angle):
    """将角度标准化到 [-pi, pi] 范围内"""
    return math.atan2(math.sin(angle), math.cos(angle))

def forward_delta(start, target):
    """返回从 start 到 target 的纯正向旋转增量 [0, 2*pi)。
       防止电机出现先反转再正转的抽搐现象。
    """
    return (float(target) - float(start)) % (2.0 * math.pi)

def forward_target(start, target):
    """展开目标角度，确保只通过正向旋转到达"""
    return float(start) + forward_delta(start, target)

def forward_lerp_angle(start, target, t):
    """仅使用正向旋转的插值方式"""
    return float(start) + forward_delta(start, target) * float(t)

def mix_vec3(start, end, t):
    """对三维向量进行线性插值"""
    return [lerp(start[i], end[i], t) for i in range(3)]

def slider_pose_to_motor_command(slider_pose_m):
    """将滑台的可见高度转换为 Webots 线性电机的指令值。
       SliderModule.proto 中的 endPoint 偏移为 initPose - 0.245
       倍率为 0.0016。因此 指令 = (pose - 0.245) / 0.0016
    """
    pose = clamp(float(slider_pose_m), SLIDER_LOWEST_POSE_M, 0.250)
    return (pose - SLIDER_POSE_TO_JOINT_OFFSET_M) / SLIDER_MULTIPLIER_M_PER_CMD

def motor_command_to_slider_pose(command):
    """将电机指令值转回真实的可见高度（米）"""
    return float(command) * SLIDER_MULTIPLIER_M_PER_CMD + SLIDER_POSE_TO_JOINT_OFFSET_M

def write_status(status, completed=0, total=0, current_task=None, error=None):
    """将当前的运行状态写入 JSON，供外部或 UI 监控进程"""
    payload = {"status": status, "completed": completed, "total": total}
    if current_task:
        payload["current_task"] = current_task
    if error:
        payload["error"] = error
    try:
        with open(STATUS_FILE, "w", encoding="utf-8") as file:
            json.dump(payload, file, ensure_ascii=True, indent=2)
    except Exception:
        pass


class DesktopSorter:
    def __init__(self):
        self.robot = Supervisor()
        self.time_step = int(self.robot.getBasicTimeStep())
        self.self_node = self.robot.getSelf()

        self.base_translation = self.read_self_translation()
        self.base_yaw = self.read_self_yaw()

        self.first_id = self.read_proto_string_field("firstJointId", "0x10")
        self.second_id = self.read_proto_string_field("secondJointId", "0x11")
        self.third_id = self.read_proto_string_field("thirdJointId", "0x12")
        self.fourth_id = self.read_proto_string_field("fourthJointId", "0x13")

        # ... 打印一系列初始配置日志 ...
        print("Robot base: translation=(%.4f, %.4f, %.4f), yaw=%.2f deg" % (
            self.base_translation[0], self.base_translation[1], self.base_translation[2], math.degrees(self.base_yaw)
        ))
        
        # 获取真实设备名（追加 ::motor 以兼容新模型结构）
        self.slider_motor = self.get_device_with_fallback([self.first_id + "::slider"])
        self.j2_motor = self.get_device_with_fallback([self.second_id + "::motor", self.second_id])
        self.j3_motor = self.get_device_with_fallback([self.third_id + "::motor", self.third_id])
        self.j4_motor = self.get_device_with_fallback([self.fourth_id + "::motor", self.fourth_id])
        self.motors = [self.slider_motor, self.j2_motor, self.j3_motor, self.j4_motor]

        # 读取真实电机位置传感器。用于真实位置反馈，避免零件根据理论位置“抄近道”。
        self.position_sensors = []
        for motor in self.motors:
            sensor = None
            try:
                sensor = motor.getPositionSensor()
                if sensor is not None:
                    sensor.enable(self.time_step)
            except Exception:
                sensor = None
            self.position_sensors.append(sensor)

        # 速度设置
        self.slider_motor.setVelocity(SLIDER_SPEED_CMD_PER_S)
        self.j2_motor.setVelocity(J2_SPEED_RAD_S)
        self.j3_motor.setVelocity(J3_SPEED_RAD_S)
        self.j4_motor.setVelocity(J4_SPEED_RAD_S)

        # 初始化目标位姿数组
        self.targets = [slider_pose_to_motor_command(SLIDER_TOP_POSE_M), 0.0, 0.0, 0.0]
        self.command_motors(self.targets)

        # 尝试获取吸盘设备
        self.vacuum = None
        try:
            self.vacuum = self.robot.getDevice("0x21")
            if self.vacuum:
                self.vacuum.enablePresence(self.time_step)
        except Exception:
            self.vacuum = None

        # 尝试找到吸盘的外部实体节点（Solid Node），用于获取真实的物理空间坐标
        self.gripper_node = None
        try:
            self.gripper_node = self.find_node_by_name("vacuum_gripper")
        except Exception:
            self.gripper_node = None

        if self.gripper_node is not None:
            print("Using outer Solid node 'vacuum_gripper' for object following.")
        elif self.vacuum is not None:
            try:
                self.gripper_node = self.robot.getFromDevice(self.vacuum)
            except Exception as exc:
                self.gripper_node = None
                print("WARNING: could not get VacuumGripper node; using FK fallback: %s" % exc)

        self.attach_offset = [0.0, 0.0, 0.0]
        self.tasks = list(DEFAULT_TASKS)
        write_status("started", 0, len(self.tasks))

    # ========================= 主流程 =========================

    def run(self):
        self.step_for(0.8)

        # 根据配置决定是否从视觉解析结果加载任务
        vision_tasks = self.load_tasks_from_vision() if USE_VISION else None
        self.tasks = vision_tasks if vision_tasks else list(DEFAULT_TASKS)
        write_status("running", 0, len(self.tasks))

        # 1. 归零起步：丝杠先升到安全高度，再进行水平展开
        self.move_slider_to(SLIDER_SAFE_POSE_M, 6.0)
        self.move_arm_to_xy(0.000, 0.180, 0.0, 1.2)
        self.step_for(0.2)

        # 2. 依次执行每个工件的抓取放置任务
        for completed, task in enumerate(self.tasks):
            self.pick_and_place(task)
            write_status("running", completed + 1, len(self.tasks), self.task_label(task))

        # 3. 任务结束：返回安全位姿
        self.move_slider_to(SLIDER_SAFE_POSE_M, 1.0)
        self.move_arm_to_xy(0.000, 0.180, 0.0, 1.2)
        write_status("done", len(self.tasks), len(self.tasks))
        print("Desktop sorting finished: %d pieces placed." % len(self.tasks))

        # 保持仿真运行
        while self.robot.step(self.time_step) != -1:
            pass

    def task_label(self, task):
        return "%s:%s->%s" % (task.get("def") or "VISION", task["shape"], task["tray"])

    def carry_route_points(self, pick_pos, place_pos):
        """生成搬运路径点，防止机械臂走直线直接斜穿障碍物。
           轨迹逻辑：取料点 -> 安全走廊 Y -> 放置区对应 X -> 目标放置点
        """
        px, py = float(pick_pos[0]), float(pick_pos[1])
        tx, ty = float(place_pos[0]), float(place_pos[1])

        safe_y = CARRY_ROUTE_SAFE_Y_M
        points = [
            (px, safe_y),
            (CARRY_ROUTE_CENTER_X_M, safe_y),
            (tx, safe_y),
            (tx, ty),
        ]

        cleaned = []
        last_x, last_y = px, py
        for x, y in points:
            # 过滤掉距离过近的重复点，使路径更丝滑
            if math.hypot(x - last_x, y - last_y) > 0.010:
                cleaned.append((x, y))
                last_x, last_y = x, y

        return cleaned

    def pick_and_place(self, task):
        """执行单一物品的抓取和放置周期。包含 8 个严谨的运动步骤。"""
        shape = self.normalize_shape(task["shape"])
        node = self.node_for_task(task)
        pick_pos = list(task["pick"])

        # 优先读取仿真环境里工件的真实物理位置，对抗随机初始摆放的误差
        if node is not None:
            current = self.node_translation(node)
            if current is not None:
                pick_pos = current

        place_pos = self.place_position(shape, task["tray"])
        pick_yaw_world = math.radians(float(task.get("angle", 0.0)))
        place_yaw_world = TRAY_WORLD_YAW_RAD.get(shape, 0.0)

        pick_slider = self.pick_slider_pose(shape, pick_pos[2])
        place_slider = self.place_slider_pose(shape, place_pos[2])
        held_pick = [pick_pos[0], pick_pos[1], self.held_bottom_z(SLIDER_SAFE_POSE_M, shape)]
        
        # 步骤 1：提升至安全高度，XY 水平移动至工件正上方
        self.move_slider_to(SLIDER_SAFE_POSE_M, 0.6)
        self.move_arm_to_xy(pick_pos[0], pick_pos[1], pick_yaw_world, 1.4)

        # 步骤 2：二次确认真实吸盘 XY 已经到位，防止边走边降
        self.wait_until_tool_over_xy((pick_pos[0], pick_pos[1]), "pick", carried_node=None, object_yaw=pick_yaw_world)

        # 丝杠单独下降，下降到底部稍作停顿消除震动
        self.move_slider_to(pick_slider, 1.3)
        self.step_for(0.45)

        # 步骤 3：接触后开启真空吸盘
        self.turn_vacuum(True)
        self.step_for(0.35)

        if node is not None:
            # 重置刚体物理状态并捕获吸盘抓取瞬间的真实空间偏差（实现完美刚性跟随）
            self.set_node_pose(node, pick_pos, [0, 0, 1, pick_yaw_world])
            self.capture_attach_offset(pick_pos)
        self.step_for(0.10)

        # 步骤 4：保持 XY 不变，单独升起丝杠拔出工件
        self.move_slider_to(SLIDER_SAFE_POSE_M, 1.3, carried_node=node, object_start=pick_pos, object_end=held_pick, object_yaw=pick_yaw_world)

        # 步骤 5：按 L 型安全走廊路径将工件平移到码放区正上方
        route_start = held_pick
        for waypoint_x, waypoint_y in self.carry_route_points(pick_pos, place_pos):
            route_end = [waypoint_x, waypoint_y, CARRY_Z_M]
            self.move_arm_to_xy(
                waypoint_x, waypoint_y, place_yaw_world, CARRY_ROUTE_STEP_DURATION_S,
                carried_node=node, object_start=route_start, object_end=route_end, object_yaw=place_yaw_world
            )
            route_start = route_end

        held_place = [place_pos[0], place_pos[1], CARRY_Z_M]

        # 步骤 6：确认 XY 真实抵达坑位上方后，再开始下降
        self.wait_until_tool_over_xy((place_pos[0], place_pos[1]), "place", carried_node=node, object_yaw=place_yaw_world)
        self.move_slider_to(place_slider, 1.3, carried_node=node, object_start=held_place, object_end=place_pos, object_yaw=place_yaw_world)
        
        # 释放前确认静止
        release_ok = self.wait_until_tool_over_place(place_pos, carried_node=node, object_yaw=place_yaw_world)
        if not release_ok:
            self.settle_and_sync_carried(self.targets, 0.45, carried_node=node, object_start=place_pos, object_end=place_pos, object_yaw=place_yaw_world, final_t=1.0)

        # 步骤 7：关闭吸盘释放工件
        self.turn_vacuum(False)
        if node is not None:
            self.set_node_pose(node, place_pos, [0, 0, 1, place_yaw_world])
        self.step_for(0.25)

        # 步骤 8：空载升起丝杠离开
        self.move_slider_to(SLIDER_SAFE_POSE_M, 1.2)

    # ========================= 运动控制 =========================

    def settle_and_sync_carried(self, target_pose, settle_s, carried_node=None, object_start=None, object_end=None, object_yaw=None, final_t=1.0):
        """保持当前目标位姿，并在等待阶段持续同步零件与物理吸盘的绑定。"""
        steps = max(1, int(float(settle_s) * 1000 / self.time_step))
        target_pose = self.apply_joint_limits(target_pose)

        for _ in range(steps):
            self.command_motors(target_pose)
            if self.robot.step(self.time_step) == -1:
                return

            if carried_node is not None and object_start is not None and object_end is not None:
                actual_pose = self.actual_motor_pose(target_pose)
                self.set_carried_node_pose(carried_node, actual_pose, object_start, object_end, final_t, object_yaw)

    def wait_until_tool_over_xy(self, target_xy, label, carried_node=None, object_yaw=None, xy_tol=DESCENT_XY_TOL_M, stable_steps=DESCENT_STABLE_STEPS, timeout_s=DESCENT_WAIT_TIMEOUT_S):
        """阻塞当前线程，不断读取真实传感器反馈，直到吸盘在水平面上达到目标 XY 的容差范围内。"""
        target_pose = self.apply_joint_limits(self.targets)
        ok_count = 0
        max_steps = max(1, int(float(timeout_s) * 1000 / self.time_step))

        for _ in range(max_steps):
            self.command_motors(target_pose)
            if self.robot.step(self.time_step) == -1:
                return False

            actual_pose = self.actual_motor_pose(target_pose)
            tool_x, tool_y = self.tool_world_xy_from_pose(actual_pose)

            if carried_node is not None:
                self.set_carried_node_pose(carried_node, actual_pose, [tool_x, tool_y, CARRY_Z_M], [tool_x, tool_y, CARRY_Z_M], 1.0, object_yaw)

            error_xy = math.hypot(tool_x - float(target_xy[0]), tool_y - float(target_xy[1]))
            if error_xy <= float(xy_tol):
                ok_count += 1
                if ok_count >= int(stable_steps):
                    return True
            else:
                ok_count = 0
        return False

    def wait_until_tool_over_place(self, place_pos, carried_node=None, object_yaw=None):
        """专门用于放置前确认：防止在运动惯性中提早撒手导致工件抛出"""
        target_pose = self.apply_joint_limits(self.targets)
        stable_steps = 0
        max_steps = max(1, int(RELEASE_WAIT_TIMEOUT_S * 1000 / self.time_step))

        for _ in range(max_steps):
            self.command_motors(target_pose)
            if self.robot.step(self.time_step) == -1:
                return False

            actual_pose = self.actual_motor_pose(target_pose)
            tool_x, tool_y = self.tool_world_xy_from_pose(actual_pose)

            if carried_node is not None:
                self.set_carried_node_pose(carried_node, actual_pose, place_pos, place_pos, 1.0, object_yaw)

            error_xy = math.hypot(tool_x - float(place_pos[0]), tool_y - float(place_pos[1]))
            if error_xy <= RELEASE_XY_TOL_M:
                stable_steps += 1
                if stable_steps >= RELEASE_STABLE_STEPS:
                    return True
            else:
                stable_steps = 0
        return False

    def move_slider_to(self, slider_pose_m, duration_s, carried_node=None, object_start=None, object_end=None, object_yaw=None):
        """控制丝杠 Z 轴升降，并基于平滑插值进行过度。如果抓着工件，则工件 Z 轴同步运动。"""
        slider_command = slider_pose_to_motor_command(slider_pose_m)
        actual_start = self.actual_motor_pose(self.targets)
        hold_rotary = list(self.targets)

        start = [actual_start[0], actual_start[1], actual_start[2], actual_start[3]]
        target = [float(slider_command), hold_rotary[1], hold_rotary[2], hold_rotary[3]]

        command_distance = abs(target[0] - start[0])
        # 根据电机速度自适应计算所需的最短时长，防止给的时间太短还没到达终点程序就进入下一步
        needed_s = 1.70 * command_distance / max(SLIDER_SPEED_CMD_PER_S, 1e-6) + SLIDER_EXTRA_SETTLE_S
        actual_duration_s = max(float(duration_s), needed_s)
        steps = max(1, int(actual_duration_s * 1000 / self.time_step))

        for index in range(1, steps + 1):
            t = smoothstep(index / steps)
            pose = [lerp(start[i], target[i], t) for i in range(4)]
            pose = self.apply_joint_limits(pose)
            self.command_motors(pose)

            if self.robot.step(self.time_step) == -1:
                return

            if carried_node is not None and object_start is not None and object_end is not None:
                self.set_carried_node_pose(carried_node, pose, object_start, object_end, t, object_yaw)

        self.settle_and_sync_carried(target, 0.22, carried_node, object_start, object_end, object_yaw, final_t=1.0)
        self.targets = self.apply_joint_limits(target)

    def move_arm_to_xy(self, x_world, y_world, tool_yaw_world, duration_s, carried_node=None, object_start=None, object_end=None, object_yaw=None):
        """将机械臂末端移动到指定的 XY 世界坐标。内部调用 IK 逆解，并对多个解进行安全性裁决。"""
        # 求出左手系与右手系的两个解析解
        q1_up, q2_up = self.solve_planar_ik_world(x_world, y_world, elbow_up=True)
        q1_down, q2_down = self.solve_planar_ik_world(x_world, y_world, elbow_up=False)
        tool_yaw_local = normalize_angle(tool_yaw_world - self.base_yaw)

        def motor_target(q1, q2):
            return [
                self.targets[0],
                JOINT2_SIGN * q1,
                JOINT3_SIGN * q2,
                JOINT4_SIGN * (tool_yaw_local - q1 - q2),
            ]

        t_up = motor_target(q1_up, q2_up)
        t_down = motor_target(q1_down, q2_down)
        start = self.actual_motor_pose(self.targets)

        def joint_distance(a, b):
            # 计算运动弧长代价
            return (
                abs(normalize_angle(b[1] - a[1]))
                + abs(normalize_angle(b[2] - a[2]))
                + 0.35 * abs(normalize_angle(b[3] - a[3]))
            )

        candidates = [t_up, t_down]
        # 核心：利用 joint_limit_penalty 对越界可能导致模型碰撞的逆解进行惩罚，优先选择安全姿态
        target = min(
            candidates,
            key=lambda candidate: (
                self.joint_limit_penalty(candidate),
                joint_distance(start, candidate),
            ),
        )
        target = [target[0]] + [normalize_angle(value) for value in target[1:]]
        target = self.apply_joint_limits(target)

        # 根据真实速度限制自动计算并延长动作时间
        d2 = abs(normalize_angle(target[1] - start[1]))
        d3 = abs(normalize_angle(target[2] - start[2]))
        d4 = abs(normalize_angle(target[3] - start[3]))
        needed_s = max(
            d2 / max(J2_SPEED_RAD_S, 1e-6),
            d3 / max(J3_SPEED_RAD_S, 1e-6),
            d4 / max(J4_SPEED_RAD_S, 1e-6),
        ) * 1.35 + ARM_MOVE_EXTRA_SETTLE_S
        
        actual_duration_s = max(float(duration_s), needed_s)
        steps = max(1, int(actual_duration_s * 1000 / self.time_step))
        
        for index in range(1, steps + 1):
            t = smoothstep(index / steps)
            pose = [start[0], 0.0, 0.0, 0.0]
            for j in range(1, 4):
                # 角度使用最短圆弧插值，允许电机适度倒转
                pose[j] = start[j] + normalize_angle(target[j] - start[j]) * t
            pose = self.apply_joint_limits(pose)
            self.command_motors(pose)

            if self.robot.step(self.time_step) == -1:
                return

            if carried_node is not None and object_start is not None and object_end is not None:
                self.set_carried_node_pose(carried_node, pose, object_start, object_end, t, object_yaw)

        self.settle_and_sync_carried(target, 0.28, carried_node, object_start, object_end, object_yaw, final_t=1.0)
        self.targets = target

    def solve_planar_ik_world(self, x_world, y_world, elbow_up=True):
        """计算 SCARA 机械臂的平面运动学逆解（IK）。基于余弦定理推导。"""
        # 将世界坐标映射到机械臂本体的局部坐标系
        x_local, y_local = self.world_to_local(x_world, y_world)
        # 减去基座到肩关节的固有偏移
        x_rel = x_local - J2_ORIGIN_X_LOCAL_M
        y_rel = y_local - J2_ORIGIN_Y_LOCAL_M

        radius = math.hypot(x_rel, y_rel)
        max_radius = LINK_1_M + LINK_2_M - 0.005
        min_radius = abs(LINK_1_M - LINK_2_M) + 0.005

        # 极限保护：超出臂展则截断至最大边缘，内缩则保护至最小半径
        if radius > max_radius:
            scale = max_radius / radius
            x_rel *= scale
            y_rel *= scale
            radius = max_radius
        elif radius < min_radius:
            if radius < 1e-9:
                return (0.0, math.pi / 2.0) if elbow_up else (0.0, -math.pi / 2.0)
            scale = min_radius / radius
            x_rel *= scale
            y_rel *= scale
            radius = min_radius

        # 余弦定理计算第二连杆相对第一连杆的夹角 (q2)
        cos_q2 = (radius * radius - LINK_1_M * LINK_1_M - LINK_2_M * LINK_2_M) / (2.0 * LINK_1_M * LINK_2_M)
        cos_q2 = clamp(cos_q2, -1.0, 1.0)
        q2 = math.acos(cos_q2) if elbow_up else -math.acos(cos_q2)
        
        # 几何推导计算第一连杆的绝对角度 (q1)
        q1 = math.atan2(y_rel, x_rel) - math.atan2(
            LINK_2_M * math.sin(q2),
            LINK_1_M + LINK_2_M * math.cos(q2),
        )
        return q1, q2

    def joint_limit_penalty(self, target):
        """评估目标位姿的关节角越界惩罚值，用于剔除容易引发碰撞的奇点逆解"""
        j2 = normalize_angle(target[1])
        j3 = normalize_angle(target[2])
        j4 = normalize_angle(target[3])
        penalty = 0.0

        if j2 < J2_MOTOR_MIN_RAD:
            penalty += (J2_MOTOR_MIN_RAD - j2) * 10.0
        elif j2 > J2_MOTOR_MAX_RAD:
            penalty += (j2 - J2_MOTOR_MAX_RAD) * 10.0

        if j3 < J3_MOTOR_MIN_RAD:
            penalty += (J3_MOTOR_MIN_RAD - j3) * 2.0
        elif j3 > J3_MOTOR_MAX_RAD:
            penalty += (j3 - J3_MOTOR_MAX_RAD) * 2.0

        if j4 < J4_MOTOR_MIN_RAD:
            penalty += (J4_MOTOR_MIN_RAD - j4) * 0.5
        elif j4 > J4_MOTOR_MAX_RAD:
            penalty += (j4 - J4_MOTOR_MAX_RAD) * 0.5

        return penalty

    def apply_joint_limits(self, pose):
        """强制对目标位姿施加钳制，保证发出给电机的信号是安全的"""
        limited = list(pose)
        limited[1] = clamp(normalize_angle(limited[1]), J2_MOTOR_MIN_RAD, J2_MOTOR_MAX_RAD)
        limited[2] = clamp(normalize_angle(limited[2]), J3_MOTOR_MIN_RAD, J3_MOTOR_MAX_RAD)
        limited[3] = clamp(normalize_angle(limited[3]), J4_MOTOR_MIN_RAD, J4_MOTOR_MAX_RAD)
        return limited

    def command_motors(self, pose):
        """执行下发控制指令到伺服电机节点"""
        pose = self.apply_joint_limits(pose)
        for motor, value in zip(self.motors, pose):
            motor.setPosition(float(value))

    # ========================= 坐标、高度、节点 =========================

    def world_to_local(self, x_world, y_world):
        """利用旋转平移矩阵将环境坐标变换至机器人自身坐标系下"""
        dx = float(x_world) - self.base_translation[0]
        dy = float(y_world) - self.base_translation[1]
        c = math.cos(-self.base_yaw)
        s = math.sin(-self.base_yaw)
        return c * dx - s * dy, s * dx + c * dy

    def local_to_world(self, x_local, y_local):
        """利用逆变换将自身坐标返回到世界坐标系上"""
        c = math.cos(self.base_yaw)
        s = math.sin(self.base_yaw)
        x_world = self.base_translation[0] + c * float(x_local) - s * float(y_local)
        y_world = self.base_translation[1] + s * float(x_local) + c * float(y_local)
        return x_world, y_world

    def tool_world_xy_from_pose(self, pose):
        """吸盘的正运动学 (FK)：通过实时的各个关节角反推出吸盘目前的理论空间坐标。
           取代之前的轨迹直接插值，防止产生视觉上零件独立于吸盘运动的问题。
        """
        q1 = float(pose[1]) / JOINT2_SIGN
        q2 = float(pose[2]) / JOINT3_SIGN

        x_local = (
            J2_ORIGIN_X_LOCAL_M
            + LINK_1_M * math.cos(q1)
            + LINK_2_M * math.cos(q1 + q2)
        )
        y_local = (
            J2_ORIGIN_Y_LOCAL_M
            + LINK_1_M * math.sin(q1)
            + LINK_2_M * math.sin(q1 + q2)
        )
        return self.local_to_world(x_local, y_local)
    """
     禁止删除和修改以上代码注释
     author: Mulan
     data: 2026-06-14
    """
    # 递归查找节点帮助函数
    def find_node_by_name(self, target_name):
        root = self.robot.getRoot()
        return self.find_node_by_name_from(root, target_name)

    def find_node_by_name_from(self, node, target_name, depth=0):
        if node is None or depth > 80:
            return None

        try:
            name_field = node.getField("name")
            if name_field is not None and name_field.getSFString() == target_name:
                return node
        except Exception:
            pass

        for field_name in ("children", "endPoint", "boundingObject"):
            try:
                field = node.getField(field_name)
                if field is None:
                    continue
                try:
                    count = field.getCount()
                    for index in range(count):
                        child = field.getMFNode(index)
                        found = self.find_node_by_name_from(child, target_name, depth + 1)
                        if found is not None:
                            return found
                    continue
                except Exception:
                    pass
                try:
                    child = field.getSFNode()
                    found = self.find_node_by_name_from(child, target_name, depth + 1)
                    if found is not None:
                        return found
                except Exception:
                    pass
            except Exception:
                pass

        return None

    def actual_motor_pose(self, fallback_pose=None):
        """读取物理传感器的真实数据。当由于惯性使得电机跟不上指令时，此数据更具参考意义。"""
        base_pose = list(fallback_pose if fallback_pose is not None else self.targets)
        sensors = getattr(self, "position_sensors", [])
        if len(sensors) != 4:
            return base_pose

        pose = list(base_pose)
        for index, sensor in enumerate(sensors):
            if sensor is None:
                continue
            try:
                value = sensor.getValue()
                if math.isfinite(value):
                    pose[index] = float(value)
            except Exception:
                pass
        return pose

    def tool_world_xyz_from_pose(self, pose):
        """计算工具端的 3D 全坐标。结合实际编码器数据以及视觉抓手的高程。"""
        """
         禁止删除和修改以上代码注释
          author: Mulan
          data: 2026-06-14
        """
        actual_pose = self.actual_motor_pose(pose)
        x, y = self.tool_world_xy_from_pose(actual_pose)

        z = 0.0
        gripper_node = getattr(self, "gripper_node", None)
        if gripper_node is not None:
            try:
                pos = gripper_node.getPosition()
                z = float(pos[2])
            except Exception:
                z = 0.0

        if z == 0.0:
            slider_pose = motor_command_to_slider_pose(actual_pose[0])
            z = self.base_translation[2] + 0.020 + slider_pose

        return [x, y, z]

    def capture_attach_offset(self, object_translation):
        """
        在开启吸盘的一瞬间冻结此时刻“工具”与“工件”的物理相对偏移，
        之后在运动全程将工件强绑定在该偏执上，实现无脱节的完美搬运。
        """
        if not hasattr(self, "attach_offset"):
            self.attach_offset = [0.0, 0.0, 0.0]
        if not hasattr(self, "gripper_node"):
            self.gripper_node = None

        tool = self.tool_world_xyz_from_pose(self.actual_motor_pose(self.targets))
        self.attach_offset = [
            tool[0] - float(object_translation[0]),
            tool[1] - float(object_translation[1]),
            tool[2] - float(object_translation[2]),
        ]
        print("Rigid attach XYZ offset=(%.4f, %.4f, %.4f); object follows real gripper Z" % (
            self.attach_offset[0], self.attach_offset[1], self.attach_offset[2]
        ))

    def carried_translation_from_tool(self, pose, object_start, object_end, t):
        """计算搬运物体的实际坐标 = 实时的物理吸盘位置 - 抓取时锁定的偏移量mulan"""
        actual_pose = self.actual_motor_pose(pose)
        tool = self.tool_world_xyz_from_pose(actual_pose)

        x = tool[0] - self.attach_offset[0]
        y = tool[1] - self.attach_offset[1]
        z = tool[2] - self.attach_offset[2]

        z = max(z, 0.018) # 地板防穿模兜底
        return [x, y, z]

    def set_carried_node_pose(self, node, pose, object_start, object_end, t, object_yaw):
        """重置 Supervisor 接管的节点，清理重力、惯性、碰撞对搬运物体的干扰"""
        if node is None:
            return
        translation = self.carried_translation_from_tool(pose, object_start, object_end, t)
        self.set_node_translation(node, translation, reset=False)
        if object_yaw is not None:
            self.set_node_rotation(node, [0, 0, 1, object_yaw], reset=False)
        try:
            node.resetPhysics()
        except Exception:
            pass

    # ========================= 其他状态读取与管理 =========================
    # (此部分主要是节点解析、字段查找，按其表面含义处理)

    def read_self_translation(self):
        field = self.self_node.getField("translation") if self.self_node else None
        if field:
            return list(field.getSFVec3f())
        return [0.0, 0.0, 0.0]

    def read_self_yaw(self):
        field = self.self_node.getField("rotation") if self.self_node else None
        if field:
            rot = list(field.getSFRotation())
            return normalize_angle(rot[2] * rot[3])
        return 0.0

    def read_proto_string_field(self, field_name, default_value):
        try:
            if self.self_node:
                field = self.self_node.getField(field_name)
                if field:
                    return field.getSFString()
        except Exception:
            pass
        return default_value

    def normalize_shape(self, shape):
        aliases = {
            "Triangle": "Triangular",
            "triangle": "Triangular",
            "triangular": "Triangular",
            "five_pointed": "FivePointed",
            "fivepointed": "FivePointed",
            "cube": "Cube",
            "cuboid": "Cuboid",
            "pentagonal": "Pentagonal",
            "cylindrical": "Cylindrical",
            "parallelogram": "Parallelogram",
            "cruciform": "Cruciform",
            "quincunx": "Quincunx",
        }
        return aliases.get(str(shape), str(shape))

    def object_height(self, shape):
        return OBJECT_HEIGHT_M.get(shape, DEFAULT_OBJECT_HEIGHT_M)

    def held_bottom_z(self, slider_pose_m, shape):
        return CARRY_Z_M

    def pick_slider_pose(self, shape, bottom_z):
        return SLIDER_PICK_POSE_M

    def place_slider_pose(self, shape, bottom_z):
        return SLIDER_PLACE_POSE_M

    def place_position(self, shape, tray_name):
        tray_x, tray_y = TRAY_CENTERS_M[tray_name]
        local_x, local_y = TRAY_LOCAL_TARGETS_M[shape]
        return [tray_x + local_x, tray_y + local_y, PLACE_BOTTOM_Z_M]

    def load_tasks_from_vision(self):
        """解析视觉识别结果，生成放置任务表"""
        if not os.path.exists(VISION_RESULT_FILE):
            return None
        try:
            with open(VISION_RESULT_FILE, "r", encoding="utf-8") as file:
                data = json.load(file)
        except Exception as exc:
            print("Vision file could not be read, using default tasks: %s" % exc)
            return None

        objects = data.get("objects") or []
        if not objects:
            return None

        counts_by_shape = {}
        tasks = []
        for item in objects:
            shape = self.normalize_shape(item.get("shape", ""))
            if shape not in TRAY_LOCAL_TARGETS_M:
                continue
            world = item.get("world_center_m") or []
            if len(world) < 2:
                continue

            wx = float(world[0])
            wy = float(world[1])
            if not (PICK_TABLE_X_RANGE[0] <= wx <= PICK_TABLE_X_RANGE[1] and PICK_TABLE_Y_RANGE[0] <= wy <= PICK_TABLE_Y_RANGE[1]):
                continue

            counts_by_shape[shape] = counts_by_shape.get(shape, 0) + 1
            if counts_by_shape[shape] > 2:
                continue
            tray_name = "top" if counts_by_shape[shape] % 2 == 1 else "bottom"
            task = {
                "def": None,
                "shape": shape,
                "pick": (wx, wy, PICK_BOTTOM_Z_M),
                "tray": tray_name,
                "angle": float(item.get("angle_z_deg", 0.0)),
            }
            task["def"] = self.closest_def_name(shape, task["pick"])
            tasks.append(task)

        tasks.sort(key=lambda t: (-t["pick"][1], t["pick"][0]))
        if tasks:
            print("Using %d tasks from vision_latest.json" % len(tasks))
            return tasks
        return None

    def node_for_task(self, task):
        def_name = task.get("def")
        if def_name:
            node = self.robot.getFromDef(def_name)
            if node:
                return node
        return self.closest_node(task["shape"], task["pick"])

    def candidate_def_names(self, shape):
        shape = self.normalize_shape(shape)
        prefixes = SHAPE_DEF_PREFIXES.get(shape, (shape.upper(),))
        names = []
        for prefix in prefixes:
            names.append(prefix)
            for index in range(1, 10):
                names.append("%s_%d" % (prefix, index))
        return names

    def closest_def_name(self, shape, pick_pos):
        best_name = None
        best_dist = 1e9
        for name in self.candidate_def_names(shape):
            node = self.robot.getFromDef(name)
            pos = self.node_translation(node) if node else None
            if pos is None:
                continue
            dist = math.hypot(pos[0] - pick_pos[0], pos[1] - pick_pos[1])
            if dist < best_dist:
                best_dist = dist
                best_name = name
        return best_name

    def closest_node(self, shape, pick_pos):
        name = self.closest_def_name(shape, pick_pos)
        return self.robot.getFromDef(name) if name else None

    def node_translation(self, node):
        if node is None:
            return None
        field = node.getField("translation")
        if field is None:
            return None
        return list(field.getSFVec3f())

    def set_node_pose(self, node, translation, rotation):
        self.set_node_translation(node, translation, reset=False)
        self.set_node_rotation(node, rotation, reset=False)
        try:
            node.resetPhysics()
        except Exception:
            pass

    def set_node_translation(self, node, translation, reset=True):
        field = node.getField("translation") if node else None
        if field:
            field.setSFVec3f([float(translation[0]), float(translation[1]), float(translation[2])])
        if reset and node:
            try:
                node.resetPhysics()
            except Exception:
                pass

    def set_node_rotation(self, node, rotation, reset=False):
        field = node.getField("rotation") if node else None
        if field:
            field.setSFRotation([float(rotation[0]), float(rotation[1]), float(rotation[2]), float(rotation[3])])
        if reset and node:
            try:
                node.resetPhysics()
            except Exception:
                pass

    # ========================= 设备与通用工具 =========================
    """
     禁止删除和修改以上代码注释
     author: Mulan
     data: 2026-06-14
    """
    def get_device_with_fallback(self, names):
        """按照后备名单查找硬件节点。主要用于规避新老版本模型结构定义名字不同的问题。"""
        tried = []
        for name in names:
            tried.append(name)
            try:
                device = self.robot.getDevice(name)
                if device is not None:
                    print("Using device: %s" % name)
                    return device
            except Exception:
                pass

        available = []
        try:
            for i in range(self.robot.getNumberOfDevices()):
                dev = self.robot.getDeviceByIndex(i)
                available.append(dev.getName())
        except Exception:
            pass
        raise RuntimeError("Device not found. Tried %s. Available devices: %s" % (tried, available))

    def turn_vacuum(self, enabled):
        """执行吸盘的吸气/放气动作"""
        if self.vacuum is None:
            return
        try:
            if enabled:
                self.vacuum.turnOn()
            else:
                self.vacuum.turnOff()
        except Exception:
            pass

    def step_for(self, duration_s):
        """空转仿真指定的时间，用于延时等待和稳定"""
        steps = max(1, int(duration_s * 1000 / self.time_step))
        for _ in range(steps):
            if self.robot.step(self.time_step) == -1:
                return

if __name__ == "__main__":
    try:
        DesktopSorter().run()
    except Exception as exc:
        write_status("error", 0, 0, error="".join(traceback.format_exception_only(type(exc), exc)).strip())
        raise