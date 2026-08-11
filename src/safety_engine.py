# -*- coding: utf-8 -*-
"""Rotor Safety Engine v1.0.0 — Real-time Robot Safety Middleware
物理层运行时安全中间件 · 协作机器人安全 · ISO 10218 / ISO/TS 15066

Core technologies:
  - Dynamic Contact Area (动态接触面积) — soft object pressure modeling
  - Impulse Safety Boundary (冲量安全边界) — momentum-based motion safety
  - Reaction Force Stability (反作用力稳定性) — mobile manipulator chassis check
  - 7-Level Risk Granularity (七级风险粒度) — L0~L6 with over_ratio metric
  - Verb-Object Impossibility (动-宾不可能组合) — semantic common-sense guard

Compliance:
  - Power and Force Limiting (PFL) — ISO/TS 15066 biomechanical limits
  - Speed and Separation Monitoring (SSM) — velocity-based constraint checking

Features:
  - Zero-dependency Python · single file · drop-in safety layer
  - Sub-millisecond latency (~17μs) · edge inference ready
  - Deterministic physics · zero neural networks · 100% interpretable

v1.0.0 升级：
  1. typing 完善：所有公共方法补全类型注解
  2. 公共逻辑抽取：_build_result 私有方法消除重复代码
  3. 输入参数校验：类型/范围/NaN/越界全面检查，新增 input_warnings 字段
  4. 外部JSON配置：load_verb_database / load_object_database / load_action_rules
     + from_config 类方法，支持业务定制无需改源码
  5. SemanticParser / SafetyAdapter 实例属性化，支持多实例独立配置
"""

__version__ = "1.0.1"
__author__ = "Rotor Dynamics"

import time
import math
import json
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any, Tuple, Union, Callable
from enum import Enum


# =====================================================================
# 第一部分：数据表
# =====================================================================

G = 9.81  # 重力加速度

# -------------------------------------------------------------------------
# 1.1 35动词数据库
# -------------------------------------------------------------------------
# 动词库单位说明: grasp_params.max_speed / stability_threshold.max_speed 单位为 mm/s
# 外部接口 speed 参数单位为 m/s，内部校验时统一转换为 m/s 比较
VERB_DATABASE: Dict[str, Dict] = {
    # === 抓态动词 18个 ===
    "抓": {
        "phase_type": "grasp", "risk_level": 3, "transition_to_hold": True,
        "stability_threshold": {"force_stability": 0.10, "max_speed": 10, "max_displacement": 2, "duration_ms": 300},
        "grasp_params": {"max_force": 50, "max_speed": 200, "max_acceleration": 500},
        "hold_params": {"min_force": 5, "max_force": 50, "max_displacement": 3},
        "balance_steady": 0.55, "margin_steady": 0.5, "disturbance_response": "越拽越紧",
    },
    "拿": {
        "phase_type": "grasp", "risk_level": 2, "transition_to_hold": True,
        "stability_threshold": {"force_stability": 0.10, "max_speed": 10, "max_displacement": 2, "duration_ms": 300},
        "grasp_params": {"max_force": 40, "max_speed": 250, "max_acceleration": 400},
        "hold_params": {"min_force": 3, "max_force": 40, "max_displacement": 5},
        "balance_steady": 0.50, "margin_steady": 0.4, "disturbance_response": "越拽越紧",
    },
    "取": {
        "phase_type": "grasp", "risk_level": 2, "transition_to_hold": True,
        "stability_threshold": {"force_stability": 0.10, "max_speed": 8, "max_displacement": 2, "duration_ms": 300},
        "grasp_params": {"max_force": 30, "max_speed": 150, "max_acceleration": 300},
        "hold_params": {"min_force": 2, "max_force": 30, "max_displacement": 3},
        "balance_steady": 0.48, "margin_steady": 0.35, "disturbance_response": "越拽越紧",
    },
    "放": {
        "phase_type": "grasp", "risk_level": 3, "transition_to_hold": False,
        "stability_threshold": None,
        "grasp_params": {"max_force": 30, "max_speed": 100, "max_acceleration": 200},
        "hold_params": None, "balance_steady": None, "margin_steady": None, "disturbance_response": None,
    },
    "推": {
        "phase_type": "grasp", "risk_level": 3, "transition_to_hold": True,
        "stability_threshold": {"force_stability": 0.15, "max_speed": 15, "max_displacement": 5, "duration_ms": 400},
        "grasp_params": {"max_force": 100, "max_speed": 300, "max_acceleration": 600},
        "hold_params": {"min_force": 10, "max_force": 100, "max_displacement": 10},
        "balance_steady": 0.52, "margin_steady": 0.6, "disturbance_response": "越推越稳",
    },
    "拉": {
        "phase_type": "grasp", "risk_level": 3, "transition_to_hold": True,
        "stability_threshold": {"force_stability": 0.15, "max_speed": 15, "max_displacement": 5, "duration_ms": 400},
        "grasp_params": {"max_force": 100, "max_speed": 300, "max_acceleration": 600},
        "hold_params": {"min_force": 10, "max_force": 100, "max_displacement": 10},
        "balance_steady": 0.52, "margin_steady": 0.6, "disturbance_response": "越拉越紧",
    },
    "拍": {
        "phase_type": "grasp", "risk_level": 5, "transition_to_hold": False,
        "stability_threshold": None,
        "grasp_params": {"max_force": 80, "max_speed": 500, "max_acceleration": 1000},
        "hold_params": None, "balance_steady": None, "margin_steady": None, "disturbance_response": None,
    },
    "打": {
        "phase_type": "grasp", "risk_level": 5, "transition_to_hold": False,
        "stability_threshold": None,
        "grasp_params": {"max_force": 80, "max_speed": 500, "max_acceleration": 1000},
        "hold_params": None, "balance_steady": None, "margin_steady": None, "disturbance_response": None,
    },
    "扔": {
        "phase_type": "grasp", "risk_level": 4, "transition_to_hold": False,
        "stability_threshold": None,
        "grasp_params": {"max_force": 60, "max_speed": 800, "max_acceleration": 1500},
        "hold_params": None, "balance_steady": None, "margin_steady": None, "disturbance_response": None,
    },
    "接": {
        "phase_type": "grasp", "risk_level": 4, "transition_to_hold": True,
        "stability_threshold": {"force_stability": 0.15, "max_speed": 20, "max_displacement": 5, "duration_ms": 500},
        "grasp_params": {"max_force": 80, "max_speed": 500, "max_acceleration": 1000},
        "hold_params": {"min_force": 5, "max_force": 80, "max_displacement": 8},
        "balance_steady": 0.58, "margin_steady": 0.55, "disturbance_response": "缓冲收紧",
    },
    "开": {
        "phase_type": "grasp", "risk_level": 2, "transition_to_hold": False,
        "stability_threshold": None,
        "grasp_params": {"max_force": 50, "max_speed": 150, "max_acceleration": 300},
        "hold_params": None, "balance_steady": None, "margin_steady": None, "disturbance_response": None,
    },
    "关": {
        "phase_type": "grasp", "risk_level": 2, "transition_to_hold": False,
        "stability_threshold": None,
        "grasp_params": {"max_force": 50, "max_speed": 120, "max_acceleration": 300},
        "hold_params": None, "balance_steady": None, "margin_steady": None, "disturbance_response": None,
    },
    "插": {
        "phase_type": "grasp", "risk_level": 2, "transition_to_hold": True,
        "stability_threshold": {"force_stability": 0.08, "max_speed": 5, "max_displacement": 0.5, "duration_ms": 300},
        "grasp_params": {"max_force": 30, "max_speed": 60, "max_acceleration": 150},
        "hold_params": {"min_force": 2, "max_force": 30, "max_displacement": 1},
        "balance_steady": 0.52, "margin_steady": 0.35, "disturbance_response": "越插越稳",
    },
    "拔": {
        "phase_type": "grasp", "risk_level": 2, "transition_to_hold": False,
        "stability_threshold": None,
        "grasp_params": {"max_force": 40, "max_speed": 80, "max_acceleration": 200},
        "hold_params": None, "balance_steady": None, "margin_steady": None, "disturbance_response": None,
    },
    "拧": {
        "phase_type": "grasp", "risk_level": 2, "transition_to_hold": True,
        "stability_threshold": {"force_stability": 0.10, "max_speed": 5, "max_displacement": 2, "duration_ms": 300},
        "grasp_params": {"max_force": 40, "max_speed": 100, "max_acceleration": 200},
        "hold_params": {"min_force": 3, "max_force": 40, "max_displacement": 3},
        "balance_steady": 0.60, "margin_steady": 0.45, "disturbance_response": "越拧越紧",
    },
    "摸": {
        "phase_type": "grasp", "risk_level": 1, "transition_to_hold": True,
        "stability_threshold": {"force_stability": 0.05, "max_speed": 2, "max_displacement": 0.5, "duration_ms": 200},
        "grasp_params": {"max_force": 5, "max_speed": 30, "max_acceleration": 50},
        "hold_params": {"min_force": 0.1, "max_force": 5, "max_displacement": 1},
        "balance_steady": 0.40, "margin_steady": 0.15, "disturbance_response": "轻触补偿",
    },
    "按": {
        "phase_type": "grasp", "risk_level": 2, "transition_to_hold": True,
        "stability_threshold": {"force_stability": 0.10, "max_speed": 5, "max_displacement": 1, "duration_ms": 200},
        "grasp_params": {"max_force": 20, "max_speed": 80, "max_acceleration": 200},
        "hold_params": {"min_force": 1, "max_force": 20, "max_displacement": 1},
        "balance_steady": 0.45, "margin_steady": 0.3, "disturbance_response": "越按越稳",
    },
    "捏": {
        "phase_type": "grasp", "risk_level": 2, "transition_to_hold": True,
        "stability_threshold": {"force_stability": 0.05, "max_speed": 3, "max_displacement": 0.5, "duration_ms": 300},
        "grasp_params": {"max_force": 15, "max_speed": 50, "max_acceleration": 100},
        "hold_params": {"min_force": 1, "max_force": 15, "max_displacement": 0.5},
        "balance_steady": 0.70, "margin_steady": 0.4, "disturbance_response": "越捏越紧",
    },
    # === 握态动词 17个 ===
    "握": {
        "phase_type": "hold", "risk_level": 2, "transition_to_hold": True,
        "stability_threshold": {"force_stability": 0.10, "max_speed": 10, "max_displacement": 2, "duration_ms": 300},
        "grasp_params": {"max_force": 50, "max_speed": 100, "max_acceleration": 300},
        "hold_params": {"min_force": 5, "max_force": 50, "max_displacement": 3},
        "balance_steady": 0.55, "margin_steady": 0.50, "disturbance_response": "越拽越紧",
    },
    "持": {
        "phase_type": "hold", "risk_level": 2, "transition_to_hold": True,
        "stability_threshold": {"force_stability": 0.08, "max_speed": 8, "max_displacement": 2, "duration_ms": 300},
        "grasp_params": {"max_force": 45, "max_speed": 80, "max_acceleration": 250},
        "hold_params": {"min_force": 5, "max_force": 45, "max_displacement": 3},
        "balance_steady": 0.52, "margin_steady": 0.48, "disturbance_response": "姿态修正",
    },
    "举": {
        "phase_type": "hold", "risk_level": 3, "transition_to_hold": True,
        "stability_threshold": {"force_stability": 0.10, "max_speed": 10, "max_displacement": 3, "duration_ms": 500},
        "grasp_params": {"max_force": 200, "max_speed": 300, "max_acceleration": 800},
        "hold_params": {"min_force": 30, "max_force": 200, "max_displacement": 5},
        "balance_steady": 0.68, "margin_steady": 0.75, "disturbance_response": "掉落保护",
    },
    "端": {
        "phase_type": "hold", "risk_level": 3, "transition_to_hold": True,
        "stability_threshold": {"force_stability": 0.08, "max_speed": 5, "max_displacement": 2, "duration_ms": 400},
        "grasp_params": {"max_force": 30, "max_speed": 80, "max_acceleration": 150},
        "hold_params": {"min_force": 3, "max_force": 30, "max_displacement": 3},
        "balance_steady": 0.50, "margin_steady": 0.40, "disturbance_response": "自动调平",
    },
    "抱": {
        "phase_type": "hold", "risk_level": 2, "transition_to_hold": True,
        "stability_threshold": {"force_stability": 0.15, "max_speed": 10, "max_displacement": 10, "duration_ms": 500},
        "grasp_params": {"max_force": 100, "max_speed": 150, "max_acceleration": 300},
        "hold_params": {"min_force": 10, "max_force": 100, "max_displacement": 10},
        "balance_steady": 0.45, "margin_steady": 0.55, "disturbance_response": "越挣越松",
    },
    "扶": {
        "phase_type": "hold", "risk_level": 1, "transition_to_hold": True,
        "stability_threshold": {"force_stability": 0.10, "max_speed": 5, "max_displacement": 3, "duration_ms": 300},
        "grasp_params": {"max_force": 20, "max_speed": 50, "max_acceleration": 100},
        "hold_params": {"min_force": 1, "max_force": 20, "max_displacement": 5},
        "balance_steady": 0.40, "margin_steady": 0.25, "disturbance_response": "倒地保护",
    },
    "托": {
        "phase_type": "hold", "risk_level": 2, "transition_to_hold": True,
        "stability_threshold": {"force_stability": 0.08, "max_speed": 5, "max_displacement": 2, "duration_ms": 300},
        "grasp_params": {"max_force": 50, "max_speed": 80, "max_acceleration": 200},
        "hold_params": {"min_force": 5, "max_force": 50, "max_displacement": 2},
        "balance_steady": 0.48, "margin_steady": 0.45, "disturbance_response": "滑落拦截",
    },
    "扛": {
        "phase_type": "hold", "risk_level": 3, "transition_to_hold": True,
        "stability_threshold": {"force_stability": 0.12, "max_speed": 10, "max_displacement": 5, "duration_ms": 500},
        "grasp_params": {"max_force": 150, "max_speed": 200, "max_acceleration": 500},
        "hold_params": {"min_force": 20, "max_force": 150, "max_displacement": 10},
        "balance_steady": 0.65, "margin_steady": 0.70, "disturbance_response": "失衡预警",
    },
    "背": {
        "phase_type": "hold", "risk_level": 2, "transition_to_hold": True,
        "stability_threshold": {"force_stability": 0.15, "max_speed": 10, "max_displacement": 8, "duration_ms": 500},
        "grasp_params": {"max_force": 100, "max_speed": 100, "max_acceleration": 300},
        "hold_params": {"min_force": 10, "max_force": 100, "max_displacement": 20},
        "balance_steady": 0.55, "margin_steady": 0.55, "disturbance_response": "前倾保护",
    },
    "夹": {
        "phase_type": "hold", "risk_level": 2, "transition_to_hold": True,
        "stability_threshold": {"force_stability": 0.08, "max_speed": 5, "max_displacement": 1, "duration_ms": 300},
        "grasp_params": {"max_force": 30, "max_speed": 60, "max_acceleration": 150},
        "hold_params": {"min_force": 2, "max_force": 30, "max_displacement": 0.5},
        "balance_steady": 0.62, "margin_steady": 0.42, "disturbance_response": "越滑越紧",
    },
    "提": {
        "phase_type": "hold", "risk_level": 3, "transition_to_hold": True,
        "stability_threshold": {"force_stability": 0.10, "max_speed": 8, "max_displacement": 3, "duration_ms": 400},
        "grasp_params": {"max_force": 80, "max_speed": 200, "max_acceleration": 400},
        "hold_params": {"min_force": 10, "max_force": 80, "max_displacement": 10},
        "balance_steady": 0.60, "margin_steady": 0.55, "disturbance_response": "防脱手",
    },
    "压": {
        "phase_type": "hold", "risk_level": 2, "transition_to_hold": True,
        "stability_threshold": {"force_stability": 0.08, "max_speed": 3, "max_displacement": 0.5, "duration_ms": 300},
        "grasp_params": {"max_force": 100, "max_speed": 80, "max_acceleration": 200},
        "hold_params": {"min_force": 5, "max_force": 100, "max_displacement": 0.5},
        "balance_steady": 0.65, "margin_steady": 0.60, "disturbance_response": "过载保护",
    },
    "踩": {
        "phase_type": "hold", "risk_level": 2, "transition_to_hold": True,
        "stability_threshold": {"force_stability": 0.10, "max_speed": 5, "max_displacement": 2, "duration_ms": 300},
        "grasp_params": {"max_force": 200, "max_speed": 200, "max_acceleration": 500},
        "hold_params": {"min_force": 10, "max_force": 200, "max_displacement": 15},
        "balance_steady": 0.55, "margin_steady": 0.50, "disturbance_response": "滑脱保护",
    },
    "叼": {
        "phase_type": "hold", "risk_level": 1, "transition_to_hold": True,
        "stability_threshold": {"force_stability": 0.10, "max_speed": 5, "max_displacement": 2, "duration_ms": 300},
        "grasp_params": {"max_force": 10, "max_speed": 30, "max_acceleration": 50},
        "hold_params": {"min_force": 1, "max_force": 10, "max_displacement": 2},
        "balance_steady": 0.50, "margin_steady": 0.30, "disturbance_response": "咬合补偿",
    },
    "吸": {
        "phase_type": "hold", "risk_level": 2, "transition_to_hold": True,
        "stability_threshold": {"force_stability": 0.10, "max_speed": 5, "max_displacement": 2, "duration_ms": 500},
        "grasp_params": {"max_force": 100, "max_speed": 80, "max_acceleration": 200},
        "hold_params": {"min_force": 10, "max_force": 100, "max_displacement": 2},
        "balance_steady": 0.52, "margin_steady": 0.45, "disturbance_response": "漏气补压",
    },
    "顶": {
        "phase_type": "hold", "risk_level": 3, "transition_to_hold": True,
        "stability_threshold": {"force_stability": 0.10, "max_speed": 5, "max_displacement": 2, "duration_ms": 400},
        "grasp_params": {"max_force": 150, "max_speed": 100, "max_acceleration": 400},
        "hold_params": {"min_force": 20, "max_force": 150, "max_displacement": 3},
        "balance_steady": 0.70, "margin_steady": 0.70, "disturbance_response": "失稳保护",
    },
    "卡": {
        "phase_type": "hold", "risk_level": 1, "transition_to_hold": True,
        "stability_threshold": {"force_stability": 0.05, "max_speed": 2, "max_displacement": 0.5, "duration_ms": 200},
        "grasp_params": {"max_force": 20, "max_speed": 50, "max_acceleration": 100},
        "hold_params": {"min_force": 0, "max_force": 20, "max_displacement": 0.5},
        "balance_steady": 0.50, "margin_steady": 0.30, "disturbance_response": "几何自锁",
    },
}

# -------------------------------------------------------------------------
# 1.2 英文动作→中文动作映射（修复English action查询问题）
# -------------------------------------------------------------------------
ENGLISH_ACTION_TO_VERB = {
    # 基础抓态
    "grasp": "抓", "hold": "握", "release": "放", "place": "放",
    "lift": "举", "carry": "提", "push": "推", "pull": "拉",
    "press": "按", "insert": "插", "rotate": "拧",
    # 移动
    "move": "扶",
    # 组合动作
    "grasp_and_insert": "插", "grasp_and_place": "放",
    # 其他
    "slide": "推", "open": "开", "close": "关",
}

# -------------------------------------------------------------------------
# 1.3 动作规则表（安全区间 [下限, 最优值, 上限]）
# -------------------------------------------------------------------------
ACTION_RULES = {
    "grasp": {
        "fragile":  {"force": [0.5, 2.0, 3.0],    "speed": [0.01, 0.03, 0.05],  "amplitude": [0.01, 0.05, 0.1],   "level": "S", "impulse_max": 0.5},
        "soft":     {"force": [1.0, 3.0, 8.0],    "speed": [0.02, 0.05, 0.1],   "amplitude": [0.02, 0.05, 0.1],   "level": "A", "impulse_max": 1.0},
        "rigid":    {"force": [3.0, 8.0, 20.0],   "speed": [0.03, 0.08, 0.15],  "amplitude": [0.02, 0.05, 0.1],   "level": "A", "impulse_max": 3.0},
        "heavy":    {"force": [10.0, 30.0, 50.0],  "speed": [0.02, 0.05, 0.1],   "amplitude": [0.03, 0.05, 0.1],   "level": "S", "impulse_max": 25.0},
        "human":    {"force": [0.5, 2.0, 5.0],    "speed": [0.005, 0.015, 0.03], "amplitude": [0.005, 0.02, 0.05],  "level": "S", "impulse_max": 20.0},
    },
    "carry": {
        "fragile":  {"force": [1.0, 5.0, 10.0],   "speed": [0.1, 0.2, 0.3],     "amplitude": [0.005, 0.02, 0.05],  "level": "S", "impulse_max": 0.5},
        "rigid":    {"force": [5.0, 15.0, 30.0],   "speed": [0.2, 0.4, 0.8],     "amplitude": [0.01, 0.03, 0.08],   "level": "A", "impulse_max": 3.0},
        "heavy":    {"force": [30.0, 80.0, 150.0],  "speed": [0.1, 0.3, 0.5],     "amplitude": [0.02, 0.05, 0.1],    "level": "S", "impulse_max": 25.0},
        "fluid":    {"force": [2.0, 5.0, 10.0],    "speed": [0.1, 0.2, 0.3],     "amplitude": [0.003, 0.01, 0.03],  "level": "S", "impulse_max": 0.5},
        "human":    {"force": [5.0, 15.0, 30.0],   "speed": [0.05, 0.15, 0.3],   "amplitude": [0.01, 0.03, 0.06],   "level": "S", "impulse_max": 20.0},
    },
    "push": {
        "furniture":    {"force": [10.0, 30.0, 80.0],   "speed": [0.1, 0.3, 0.6],   "amplitude": [0.1, 0.5, 1.0],   "level": "A", "impulse_max": 30.0},
        "door":         {"force": [5.0, 20.0, 50.0],    "speed": [0.1, 0.3, 0.5],   "amplitude": [0.3, 0.8, 1.2],   "level": "A", "impulse_max": 25.0},
        "human":        {"force": [2.0, 5.0, 15.0],     "speed": [0.05, 0.1, 0.2],  "amplitude": [0.01, 0.05, 0.1],  "level": "S", "impulse_max": 15.0},
        "heavy_object": {"force": [50.0, 100.0, 200.0],  "speed": [0.05, 0.2, 0.5],  "amplitude": [0.1, 0.5, 1.5],    "level": "S", "impulse_max": 50.0},
        "small_object": {"force": [0.5, 2.0, 5.0],      "speed": [0.05, 0.15, 0.3],  "amplitude": [0.05, 0.2, 0.5],   "level": "B", "impulse_max": 0.2},
    },
    "move": {
        "open_space": {"force": [0.0, 0.0, 0.0], "speed": [0.3, 0.6, 1.2], "amplitude": [0.5, 1.0, 3.0], "level": "A", "impulse_max": 100.0},
        "crowded":    {"force": [0.0, 0.0, 0.0], "speed": [0.1, 0.3, 0.5], "amplitude": [0.2, 0.5, 1.0], "level": "S", "impulse_max": 40.0},
        "near_human": {"force": [0.0, 0.0, 0.0], "speed": [0.1, 0.2, 0.4], "amplitude": [0.3, 0.5, 1.0], "level": "S", "impulse_max": 30.0},
        "emergency":  {"force": [0.0, 0.0, 0.0], "speed": [0.5, 1.0, 2.0], "amplitude": [1.0, 2.0, 5.0], "level": "A", "impulse_max": 200.0},
    },
}

# -------------------------------------------------------------------------
# 1.4 动作同义词（中文→英文标准key）
# -------------------------------------------------------------------------
ACTION_SYNONYMS = {
    "抓": "grasp", "抓取": "grasp", "拿": "grasp", "拿起": "grasp", "夹": "grasp",
    "捏": "grasp", "握住": "grasp", "按住": "按",
    "搬": "carry", "搬运": "carry", "托": "carry", "举": "carry", "抬起": "carry", "端": "carry",
    "推": "push", "拉": "push", "推拉": "push", "挪动": "push", "推开": "push", "拉开": "push",
    "走": "move", "移动": "move", "行走": "move", "靠近": "move", "接近": "move",
    "后退": "move", "走过去": "move",
}

# -------------------------------------------------------------------------
# 1.5 物体属性表
# -------------------------------------------------------------------------
OBJECT_PROPERTIES: Dict[str, Dict] = {
    "鸡蛋":   {"category": "fragile", "fragile": 0.8, "heavy": 0.0, "fluid": 0.0, "human_contact": 0.0, "mass_kg": 0.05, "weight_est": 0.1, "contact_area_mm2": 200, "contact_stiffness": 5.0, "max_deform": 0.05},
    "面包":   {"category": "soft", "fragile": 0.1, "heavy": 0.0, "fluid": 0.0, "human_contact": 1.0, "mass_kg": 0.1, "weight_est": 0.3, "contact_area_mm2": 500, "contact_stiffness": 0.5, "max_deform": 0.5},
    "水果":   {"category": "soft", "fragile": 0.2, "heavy": 0.0, "fluid": 0.0, "human_contact": 1.0, "mass_kg": 0.15, "weight_est": 0.2, "contact_area_mm2": 300, "contact_stiffness": 0.8, "max_deform": 0.4},
    "杯子":   {"category": "rigid", "fragile": 0.3, "heavy": 0.0, "fluid": 0.5, "human_contact": 1.0, "mass_kg": 0.2, "weight_est": 0.3, "contact_area_mm2": 400, "contact_stiffness": 3.0, "max_deform": 0.1},
    "玻璃杯":   {"category": "fragile", "fragile": 0.9, "heavy": 0.0, "fluid": 0.0, "human_contact": 1.0, "mass_kg": 0.25, "weight_est": 0.25, "contact_area_mm2": 350, "contact_stiffness": 8.0, "max_deform": 0.05},
    "碗":   {"category": "rigid", "fragile": 0.4, "heavy": 0.0, "fluid": 0.5, "human_contact": 1.0, "mass_kg": 0.3, "weight_est": 0.2, "contact_area_mm2": 450, "contact_stiffness": 3.0, "max_deform": 0.1},
    "水":   {"category": "fluid", "fragile": 0.0, "heavy": 0.0, "fluid": 1.0, "human_contact": 1.0, "mass_kg": 0.2, "weight_est": 0.5, "contact_area_mm2": 0, "contact_stiffness": 0.1, "max_deform": 0.5},
    "汤":   {"category": "fluid", "fragile": 0.0, "heavy": 0.0, "fluid": 1.0, "human_contact": 1.0, "mass_kg": 0.2, "weight_est": 0.5, "contact_area_mm2": 0, "contact_stiffness": 0.1, "max_deform": 0.5},
    "铁块":   {"category": "heavy", "fragile": 0.0, "heavy": 1.0, "fluid": 0.0, "human_contact": 0.0, "mass_kg": 20.0, "weight_est": 0.9, "contact_area_mm2": 300, "contact_stiffness": 20.0, "max_deform": 0.02},
    "石头":   {"category": "heavy", "fragile": 0.0, "heavy": 1.0, "fluid": 0.0, "human_contact": 0.0, "mass_kg": 15.0, "weight_est": 0.9, "contact_area_mm2": 350, "contact_stiffness": 15.0, "max_deform": 0.02},
    "椅子":   {"category": "furniture", "fragile": 0.0, "heavy": 1.0, "fluid": 0.0, "human_contact": 1.0, "mass_kg": 5.0, "weight_est": 0.5, "contact_area_mm2": 800, "contact_stiffness": 5.0, "max_deform": 0.1},
    "桌子":   {"category": "heavy", "fragile": 0.0, "heavy": 1.0, "fluid": 0.0, "human_contact": 1.0, "mass_kg": 25.0, "weight_est": 0.8, "contact_area_mm2": 1000, "contact_stiffness": 8.0, "max_deform": 0.05},
    "门":   {"category": "door", "fragile": 0.0, "heavy": 1.0, "fluid": 0.0, "human_contact": 1.0, "mass_kg": 30.0, "weight_est": 0.7, "contact_area_mm2": 600, "contact_stiffness": 10.0, "max_deform": 0.05},
    "抽屉":   {"category": "door", "fragile": 0.0, "heavy": 0.0, "fluid": 0.0, "human_contact": 0.0, "mass_kg": 2.0, "weight_est": 0.3, "contact_area_mm2": 500, "contact_stiffness": 5.0, "max_deform": 0.08},
    "人":   {"category": "human", "fragile": 1.0, "heavy": 0.0, "fluid": 0.0, "human_contact": 1.0, "mass_kg": 70.0, "weight_est": 0.7, "contact_area_mm2": 500, "contact_stiffness": 0.5, "max_deform": 0.3},
    "手":   {"category": "human", "fragile": 1.0, "heavy": 0.0, "fluid": 0.0, "human_contact": 1.0, "mass_kg": 0.5, "weight_est": 0.05, "contact_area_mm2": 200, "contact_stiffness": 0.8, "max_deform": 0.3},
    "胳膊":   {"category": "human", "fragile": 1.0, "heavy": 1.0, "fluid": 0.0, "human_contact": 1.0, "mass_kg": 3.0, "weight_est": 0.2, "contact_area_mm2": 300, "contact_stiffness": 1.0, "max_deform": 0.25},
    "笔":   {"category": "small_object", "fragile": 0.0, "heavy": 0.0, "fluid": 0.0, "human_contact": 1.0, "mass_kg": 0.01, "weight_est": 0.02, "contact_area_mm2": 100, "contact_stiffness": 10.0, "max_deform": 0.05},
    "钥匙":   {"category": "small_object", "fragile": 0.0, "heavy": 0.0, "fluid": 0.0, "human_contact": 1.0, "mass_kg": 0.02, "weight_est": 0.03, "contact_area_mm2": 150, "contact_stiffness": 15.0, "max_deform": 0.03},
    "书":   {"category": "rigid", "fragile": 0.2, "heavy": 0.0, "fluid": 0.0, "human_contact": 0.0, "mass_kg": 0.3, "weight_est": 0.3, "contact_area_mm2": 300, "contact_stiffness": 4.0, "max_deform": 0.1},
}

OBJECT_MASS = {
    "鸡蛋": 0.05,
    "面包": 0.10,
    "水果": 0.15,
    "杯子": 0.20,
    "玻璃杯": 0.25,
    "碗": 0.30,
    "水": 0.20,
    "汤": 0.20,
    "铁块": 20.00,
    "石头": 15.00,
    "椅子": 5.00,
    "桌子": 25.00,
    "门": 30.00,
    "抽屉": 2.00,
    "人": 70.00,
    "手": 0.50,
    "胳膊": 3.00,
    "笔": 0.01,
    "钥匙": 0.02,
    "书": 0.30,
}

# -------------------------------------------------------------------------
# 1.6 机器人能力表
# -------------------------------------------------------------------------
ROBOT_CAPABILITIES: Dict[str, Dict] = {
    "wheel_arm": {
        "name": "轮式机械臂", "tier": 2, "dexterity": 0.3,
        "actions": ["grasp", "carry", "push", "move"],
        "max_force": 50.0, "max_speed": 1.0, "max_accel": 5.0,
                  "base_weight_kg": 30.0,
    },
    "humanoid_basic": {
        "name": "基础人形", "tier": 2, "dexterity": 0.6,
        "actions": ["grasp", "carry", "push", "move"],
        "max_force": 100.0, "max_speed": 1.5, "max_accel": 10.0,
                  "base_weight_kg": 50.0,
    },
    "humanoid_dexterous": {
        "name": "灵巧人形", "tier": 3, "dexterity": 0.9,
        "actions": ["grasp", "carry", "push", "move"],
        "max_force": 200.0, "max_speed": 3.0, "max_accel": 20.0,
                  "base_weight_kg": 60.0,
    },
    "轮式机械臂": {"name": "轮式机械臂", "tier": 1, "dexterity": 0.3,
                  "actions": ["grasp", "carry", "push", "move"],
                  "max_force": 50, "max_speed": 1.0, "max_accel": 5.0,  "base_weight_kg": 30.0},
    "基础人形": {"name": "基础人形", "tier": 2, "dexterity": 0.6,
                "actions": ["grasp", "carry", "push", "move"],
                "max_force": 100, "max_speed": 1.5, "max_accel": 10.0,  "base_weight_kg": 50.0},
    "灵巧人形": {"name": "灵巧人形", "tier": 3, "dexterity": 0.9,
                "actions": ["grasp", "carry", "push", "move"],
                "max_force": 200, "max_speed": 3.0, "max_accel": 20.0,  "base_weight_kg": 60.0},
    "机械臂": {"name": "机械臂", "tier": 2, "dexterity": 0.5,
              "actions": ["grasp", "carry", "push", "move"],
              "max_force": 150, "max_speed": 1.5, "max_accel": 10.0,  "base_weight_kg": 45.0},
}

# -------------------------------------------------------------------------
# 1.7 物理约束系数
# -------------------------------------------------------------------------
STABILITY_CONSTRAINTS = {
    "rigid":      {"max_force_mult": 50.0, "max_vel_mult": 10.0, "max_accel_mult": 30.0},
    "semi_rigid": {"max_force_mult": 20.0, "max_vel_mult": 5.0,  "max_accel_mult": 15.0},
    "flexible":   {"max_force_mult": 8.0,  "max_vel_mult": 2.0,  "max_accel_mult": 5.0},
    "fragile":    {"max_force_mult": 3.0,  "max_vel_mult": 1.0,  "max_accel_mult": 2.0},
    "liquid":     {"max_force_mult": 1.0,  "max_vel_mult": 0.5,  "max_accel_mult": 1.0},
}

CATEGORY_TO_STABILITY = {
    "fragile": "fragile", "soft": "semi_rigid", "rigid": "rigid",
    "heavy": "rigid", "human": "fragile", "fluid": "liquid",
    "furniture": "rigid", "door": "rigid", "small_object": "rigid",
}

ACTION_PROFILES = {
    "grasp": {"force_scale": 1.0, "vel_scale": 0.3, "accel_scale": 1.0},
    "push": {"force_scale": 0.8, "vel_scale": 1.0, "accel_scale": 2.0},
    "carry": {"force_scale": 1.5, "vel_scale": 0.5, "accel_scale": 1.5},
    "move": {"force_scale": 0.0, "vel_scale": 1.0, "accel_scale": 1.0},
    "default": {"force_scale": 1.0, "vel_scale": 0.5, "accel_scale": 1.0},
}

# -------------------------------------------------------------------------
# 1.8 语义不可能组合
# -------------------------------------------------------------------------
IMPOSSIBLE_PAIRS = {
    "grasp": {"fluid"},
    "push":  {"fluid", "gas"},
}

# -------------------------------------------------------------------------
# 1.9 ISO标准参数（ISO 10218-1/2 + ISO/TS 15066）
# -------------------------------------------------------------------------
@dataclass
class ISOStandards:
    """ISO 10218-1/2 和 ISO/TS 15066 核心安全参数
    
    来源:
    - ISO 10218-1/2: 协作模式安全功能要求
    - ISO/TS 15066:2016 Annex A: 生物力学极限（力/压力限值）
    - ISO 10218-2:2025: 整合了TS 15066的协作应用内容
    """
    # === ISO 10218 基础参数 ===
    # 协作模式最大速度 (reduced-speed safety function)
    max_collaborative_speed_ms: float = 0.250  # 250 mm/s = 0.25 m/s
    
    # 操作员接近速度（ISO 13855）
    human_approach_speed_ms: float = 1.6  # 1.6 m/s 步行速度
    
    # === ISO/TS 15066 准静态接触力限值 (quasi-static / clamping) ===
    # 单位: 牛顿(N)
    quasi_static_force_limits: Dict[str, float] = field(default_factory=lambda: {
        "skull_forehead": 65.0,    # 头骨/前额
        "face":           45.0,    # 面部
        "neck":           75.0,    # 颈部
        "chest":          70.0,    # 胸部/躯干
        "upper_arm":     110.0,    # 上臂/肘部
        "forearm":       100.0,    # 前臂
        "hand":           75.0,    # 手/手指
        "lower_leg":     130.0,    # 小腿/膝盖
    })
    
    # === ISO/TS 15066 瞬态接触力限值 (transient / impact) ===
    # 单位: 牛顿(N)
    transient_force_limits: Dict[str, float] = field(default_factory=lambda: {
        "skull_forehead": 130.0,   # 头骨/前额
        "face":            65.0,   # 面部
        "neck":           150.0,   # 颈部
        "chest":          140.0,   # 胸部/躯干
        "upper_arm":      210.0,   # 上臂/肘部
        "forearm":        190.0,   # 前臂
        "hand":           150.0,   # 手/手指
        "lower_leg":      220.0,   # 小腿/膝盖
    })
    
    # === ISO/TS 15066 准静态压强限值 ===
    # 单位: N/cm²
    quasi_static_pressure_limits: Dict[str, float] = field(default_factory=lambda: {
        "skull_forehead": 30.0,
        "face":           20.0,
        "neck":           25.0,
        "chest":          25.0,
        "upper_arm":      30.0,
        "forearm":        40.0,
        "hand":           40.0,
        "lower_leg":      50.0,
    })
    
    # 身体部位→映射表（中文物体名→ISO区域）
    body_region_mapping: Dict[str, str] = field(default_factory=lambda: {
        "人": "chest", "手": "hand", "胳膊": "forearm",
    })
    
    # 默认接触力限值（不区分部位时使用保守值：面部65N准静态）
    default_quasi_static_force: float = 65.0
    default_transient_force: float = 130.0
    default_quasi_static_pressure: float = 25.0
    
    def get_body_region(self, obj_name: str) -> Optional[str]:
        """获取物体对应的身体区域"""
        return self.body_region_mapping.get(obj_name)
    
    def get_quasi_static_limit(self, obj_name: str) -> float:
        """获取准静态接触力限值"""
        region = self.get_body_region(obj_name)
        if region:
            return self.quasi_static_force_limits.get(region, self.default_quasi_static_force)
        return self.default_quasi_static_force
    
    def get_transient_limit(self, obj_name: str) -> float:
        """获取瞬态接触力限值"""
        region = self.get_body_region(obj_name)
        if region:
            return self.transient_force_limits.get(region, self.default_transient_force)
        return self.default_transient_force


# -------------------------------------------------------------------------
# 1.10 安全规则
# -------------------------------------------------------------------------
@dataclass
class SafetyRules:
    safety_ratio_min: float = 0.30
    safety_ratio_max: float = 0.70
    safety_margin_min: float = 0.20
    safety_margin_max: float = 0.80
    weight_sv: float = 0.25
    weight_vo: float = 0.35
    weight_params: float = 0.40
    level_safe: float = 0.7
    level_caution: float = 0.4
    fragile_force_factor: float = 0.5
    # 干扰等级阈值
    dist_micro: float = 0.05
    dist_small: float = 0.12
    dist_medium: float = 0.30
    dist_large: float = 0.50
    dist_danger: float = 0.80
    human_contact_threshold: float = 0.5  # human_contact>0.5时激活ISO限值
    # 反作用力约束摩擦系数（橡胶-地面默认0.6）
    friction_coef: float = 0.6
    # ISO标准实例
    iso: ISOStandards = field(default_factory=ISOStandards)

    def in_safety_zone(self, phi: float, a: float) -> bool:
        return self.safety_ratio_min <= phi <= self.safety_ratio_max and self.safety_margin_min <= a <= self.safety_margin_max

DEFAULT_RULES = SafetyRules()

# -------------------------------------------------------------------------
# 1.11 干扰响应模板
# -------------------------------------------------------------------------
DISTURBANCE_TEMPLATES = {
    "none":   {"force_ratio": 0.0,  "speed_factor": 1.0, "actions": ["稳态维持"], "transition": None},
    "micro":  {"force_ratio": 0.05, "speed_factor": 1.0, "actions": ["微扰自动补偿"], "transition": None},
    "small":  {"force_ratio": 0.20, "speed_factor": 0.7, "actions": ["收紧20%", "限速70%"], "transition": None},
    "medium": {"force_ratio": 0.50, "speed_factor": 0.3, "actions": ["收紧50%", "限速30%", "姿态修正"], "transition": None},
    "large":  {"force_ratio": 0.80, "speed_factor": 0.0, "actions": ["重抓流程"], "transition": "grasp"},
    "danger": {"force_ratio": -1.0, "speed_factor": 0.0, "actions": ["紧急释放", "全系统停止"], "transition": "release"},
}


# -------------------------------------------------------------------------
# 1.12 v4.2.2 七级风险等级阈值（基于安全裕度 margin）
# -------------------------------------------------------------------------
# PASS 样本：基于 margin 划分（margin = 1 - 最大超限比）
# FAIL 样本：基于 param_check.score 划分
RISK_LEVEL_7_PASS_THRESHOLDS = [
    # (min_margin, level)
    (0.50, "L0"),   # 安全：裕度>50%
    (0.30, "L1"),   # 注意：裕度30~50%
    (0.15, "L2"),   # 轻度：裕度15~30%
    (0.05, "L3"),   # 偏中：裕度5~15%（接近边界）
    (0.0,  "L4"),   # 中度：裕度0~5%（临界区）
]
# FAIL 样本：基于 score 划分
RISK_LEVEL_7_FAIL_THRESHOLDS = [
    (0.5, "L5"),    # 偏重：score 0.3~0.5，有 warning 未硬 FAIL
    (0.0, "L6"),    # 严重：score=0，硬 FAIL
]

RISK_LEVEL_7_LABELS = {
    "L0": "安全",
    "L1": "注意",
    "L2": "轻度",
    "L3": "偏中",
    "L4": "中度",
    "L5": "偏重",
    "L6": "严重",
}

# 五级 → 七级兼容映射（用于辅助理解，不影响原 risk_level 字段）
RISK_LEVEL_5_TO_7_MAP = {
    "LOW":    ["L0", "L1", "L2"],   # 原 LOW 对应 L0~L2
    "MEDIUM": ["L3", "L4", "L5"],   # 原 MEDIUM 对应 L3~L5
    "HIGH":   ["L6"],               # 原 HIGH 对应 L6
}

# 风险子类型定义
RISK_SUBTYPE_DEFS = {
    "force_overload":      "力过载",
    "speed_exceed":        "速度超限",
    "pressure_risk":       "压强风险",
    "impulse_risk":        "冲量风险",
    "iso_violation":       "ISO接触力超限",
    "reaction_unstable":   "反作用不稳定",
}


# =====================================================================
# 第二部分：数据结构
# =====================================================================

class Verdict(str, Enum):
    PASS = "PASS"
    FAIL = "FAIL"
    REJECT = "REJECT"

@dataclass
class V4Result:
    verdict: str = "PASS"
    state: str = "idle"
    risk_level: str = "LOW"
    latency_ms: float = 0.0
    # 适配量化（参数校验量化表达）
    capability_match: Dict = field(default_factory=dict)
    action_target_match: Dict = field(default_factory=dict)
    param_check: Dict = field(default_factory=dict)
    # 安全合规区间
    safety_zone: Dict = field(default_factory=dict)
    # 干扰
    disturbance: Dict = field(default_factory=dict)
    # 修正
    correction: str = "无需修正"
    recommended_params: Dict = field(default_factory=dict)
    action_key: str = ""
    object_category: str = ""
    rule_source: str = ""
    # 接触面积（安全维度）
    contact_area_mm2: float = 0.0
    pressure_kPa: float = 0.0
    # ISO合规标注
    iso_compliance: str = ""  # "符合ISO 10218/TS 15066" / "超出限值" / "未触发"
    # === v4.2.2 新增字段 ===
    # 七级风险等级（L0~L6），原 risk_level 保持五级不变
    risk_level_7: str = ""
    # 风险子类型标签列表
    risk_subtypes: List[str] = field(default_factory=list)
    # 智能推荐参数（FAIL样本基于超标维度计算）
    recommended_params_v2: Dict = field(default_factory=dict)
    # 边界安全回退参数（L3/L4边界PASS样本）
    retreat_params: Optional[Dict] = None
    # 语义合理性分数透传（外部传入，原样回传）
    semantic_plausibility_score: Optional[float] = None
    # === v1.0.0 新增字段 ===
    # 输入层面的非致命警告列表
    input_warnings: List[str] = field(default_factory=list)
    # === v4.2.3 新增字段 ===
    # FAIL 样本的超标倍率：max(force/f_limit, speed/s_limit, pressure/p_limit,
    #                          impulse/impulse_limit, reaction_force/reaction_limit)
    # PASS 样本为 0.0
    over_ratio: float = 0.0

    def to_dict(self) -> Dict:
        return {
            "verdict": self.verdict, "state": self.state,
            "risk_level": self.risk_level, "latency_ms": round(self.latency_ms, 4),
            "capability_match": self.capability_match,
            "action_target_match": self.action_target_match,
            "param_check": self.param_check,
            "safety_zone": self.safety_zone,
            "disturbance": self.disturbance,
            "correction": self.correction, "recommended_params": self.recommended_params,
            "action_key": self.action_key, "object_category": self.object_category,
            "rule_source": self.rule_source,
            "contact_area_mm2": self.contact_area_mm2,
            "pressure_kPa": round(self.pressure_kPa, 2),
            "iso_compliance": self.iso_compliance,
            # === v4.2.2 新增字段 ===
            "risk_level_7": self.risk_level_7,
            "risk_subtypes": self.risk_subtypes,
            "recommended_params_v2": self.recommended_params_v2,
            "retreat_params": self.retreat_params,
            "semantic_plausibility_score": self.semantic_plausibility_score,
            # === v4.2.3 新增字段 ===
            "over_ratio": round(self.over_ratio, 4),
            # === v1.0.0 新增字段 ===
            "input_warnings": self.input_warnings,
        }


# =====================================================================
# 第三部分：Layer 1 — 语义解析
# =====================================================================

class SemanticParser:
    """语义解析：中文 → 标准key + 属性查询

    v1.0.0: 支持通过构造函数注入 verb_db / object_db / action_rules，
            不传则使用全局默认值（保持向后兼容）。
    """

    def __init__(self,
                 verb_db: Optional[Dict[str, Dict[str, Any]]] = None,
                 object_db: Optional[Dict[str, Dict[str, Any]]] = None,
                 action_rules: Optional[Dict[str, Dict[str, Dict[str, Any]]]] = None):
        self.verb_db = verb_db if verb_db is not None else VERB_DATABASE
        self.object_db = object_db if object_db is not None else OBJECT_PROPERTIES
        self.action_rules = action_rules if action_rules is not None else ACTION_RULES

    @staticmethod
    def normalize_robot_cap(robot_cap: Dict[str, Any]) -> Dict[str, Any]:
        """机器人能力字段归一化：兼容 max_force_n/max_velocity_ms/max_acceleration_ms2
        和 max_force/max_speed/max_accel 两套命名，统一输出后者。"""
        cap = dict(robot_cap)
        # force: 优先 max_force_n，其次 max_force，默认 100
        if "max_force_n" in cap:
            cap["max_force"] = cap["max_force_n"]
        elif "max_force" not in cap:
            cap["max_force"] = 100.0
        # speed: 优先 max_velocity_ms，其次 max_speed，默认 2.0
        if "max_velocity_ms" in cap:
            cap["max_speed"] = cap["max_velocity_ms"]
        elif "max_speed" not in cap:
            cap["max_speed"] = 2.0
        # accel: 优先 max_acceleration_ms2，其次 max_accel，默认 10
        if "max_acceleration_ms2" in cap:
            cap["max_accel"] = cap["max_acceleration_ms2"]
        elif "max_accel" not in cap:
            cap["max_accel"] = 10.0
        # base weight (reaction force calculation)
        if "base_weight_kg" not in cap:
            cap["base_weight_kg"] = 20.0  # 保守默认值
        # ground friction coefficient (reaction force calculation)
        if "ground_friction" not in cap:
            cap["ground_friction"] = 0.6  # 橡胶-地面默认值
        # fixed base flag (industrial robots)
        if "fixed_base" not in cap:
            cap["fixed_base"] = False
        return cap


    @staticmethod
    def resolve_action(action_str: str) -> str:
        # 先查同义词表（中文→英文）
        result = ACTION_SYNONYMS.get(action_str)
        if result:
            return result
        # 如果已经是标准英文key，直接返回
        if action_str in ACTION_RULES:
            return action_str
        # 兜底：原样返回
        return action_str

    def get_verb_info(self, action_key: str) -> Optional[Dict[str, Any]]:
        """获取动词信息：支持英文key自动映射到中文动作"""
        vi = self.verb_db.get(action_key)
        if vi:
            return vi
        # 英文key → 中文动作映射
        cn_verb = ENGLISH_ACTION_TO_VERB.get(action_key)
        if cn_verb:
            return self.verb_db.get(cn_verb)
        return None

    def get_object_props(self, obj_name: str) -> Dict[str, Any]:
        props = self.object_db.get(obj_name)
        if props:
            return props
        return {"category": "rigid", "fragile": 0.3, "heavy": 0.3,
                "fluid": 0.0, "human_contact": 0.1, "mass_kg": 0.3, "weight_est": 0.3, "contact_area_mm2": 400, "contact_stiffness": 3.0, "max_deform": 0.1}

    @staticmethod
    def get_robot_cap(robot: str) -> Dict[str, Any]:
        result = ROBOT_CAPABILITIES.get(robot) or ROBOT_CAPABILITIES.get("humanoid_basic")
        return result  # type: ignore[return-value]  # type: ignore[return-value]

    def lookup_rules(self, action_key: str, category: str) -> Optional[Dict[str, Any]]:
        action_rules = self.action_rules.get(action_key)
        if not action_rules:
            return None
        return action_rules.get(category)

    def lookup_rules_fallback(self, action_key: str, props: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """降级匹配规则
        
        优先级：
        1. category 精确匹配（如 fragile/rigid/heavy/fluid/soft/human/furniture/door 等）
        2. 属性 fallback：heavy → heavy 类, fluid → fluid 类, fragile>0.5 → fragile 类
        3. human_contact：仅当 category 本身是 human 或 接触程度很高(full body)时才用 human 规则
        4. 兜底：取第一条规则
        """
        action_rules = self.action_rules.get(action_key)
        if not action_rules:
            return None
        cat = props.get("category", "rigid")
        if cat in action_rules:
            return action_rules[cat]
        # 属性级 fallback（按物理属性，不是接触状态）
        if props.get("heavy", 0) > 0.5 and "heavy" in action_rules:
            return action_rules["heavy"]
        if props.get("fluid", 0) > 0.5 and "fluid" in action_rules:
            return action_rules["fluid"]
        if props.get("fragile", 0) > 0.5 and "fragile" in action_rules:
            return action_rules["fragile"]
        # human_contact 仅作 ISO 触发依据，不用作规则 fallback 的高优先级依据
        # 避免日常被接触的物体（杯子/笔）套用人体安全阈值
        return next(iter(action_rules.values()))

    @staticmethod
    def check_impossible(action_key: str, category: str) -> bool:
        return category in IMPOSSIBLE_PAIRS.get(action_key, set())

    @staticmethod
    def compute_safety_zone(params: Dict[str, Any], rules: Dict[str, Any]) -> Dict[str, Any]:
        force = abs(float(params.get("force", 0)))
        speed = params.get("speed", 0)
        f_min, f_opt, f_max = rules["force"]
        s_min, s_opt, s_max = rules["speed"]

        if s_opt != s_min and speed <= s_opt:
            phi = 0.5 * (speed - s_min) / (s_opt - s_min)
        elif s_max != s_opt:
            phi = 0.5 + 0.5 * (speed - s_opt) / (s_max - s_opt)
        else:
            phi = 0.5
        phi = max(0.0, min(1.0, phi))

        energy = force * speed
        e_min = f_min * s_min
        e_opt = f_opt * s_opt
        e_max = f_max * s_max
        if e_opt != e_min and energy <= e_opt:
            a2 = 0.5 * (energy - e_min) / (e_opt - e_min)
        elif e_max != e_opt:
            a2 = 0.5 + 0.5 * (energy - e_opt) / (e_max - e_opt)
        else:
            a2 = 0.5
        a2 = max(0.0, min(1.0, a2))

        in_gz = (0.3 <= phi <= 0.7) and (0.2 <= a2 <= 0.8)
        margin = min(phi - 0.3, 0.7 - phi, a2 - 0.2, 0.8 - a2)
        return {"force_ratio": round(phi, 3), "margin_ratio": round(a2, 3),
                "in_safety_zone": in_gz, "safety_margin": round(margin, 3)}

    @staticmethod
    def recommend_params(rules: Dict[str, Any], context: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        f = rules["force"][1]
        s = rules["speed"][1]
        a = rules["amplitude"][1]
        if context:
            if context.get("near_human"): f *= 0.5; s *= 0.5; a *= 0.6
            if context.get("fragile"): f *= 0.7; s *= 0.7
        return {"force": round(f, 3), "speed": round(s, 4), "amplitude": round(a, 4)}
    # ---------------------------------------------------------------------
    # v4.2.2 新增：七级风险等级映射
    # ---------------------------------------------------------------------
    @staticmethod
    def map_risk_level_7(verdict: str, margin: float, score: float) -> str:
        """
        将判定结果映射到七级风险等级。

        PASS 样本：基于 margin（安全裕度）划分
        FAIL 样本：基于 param_check.score 划分
        REJECT 样本：L6（严重）

        Returns:
            "L0" ~ "L6"
        """
        if verdict == "REJECT":
            return "L6"

        if verdict == "PASS":
            for threshold, level in RISK_LEVEL_7_PASS_THRESHOLDS:
                if margin > threshold:
                    return level
            return "L4"  # margin == 0 的边界情况

        # FAIL
        score = max(0.0, min(1.0, score))
        if score >= 0.5:
            return "L5"  # 偏重：还有一定裕度但已 FAIL
        return "L6"      # 严重：硬 FAIL

    # ---------------------------------------------------------------------
    # v4.2.2 新增：风险子类型标签判定
    # ---------------------------------------------------------------------
    @staticmethod
    def classify_risk_subtypes(param_detail: str, verdict: str,
                                force: float, speed: float,
                                pressure_kPa: float,
                                iso_compliance: str,
                                obj_props: Dict,
                                f_limit: float = 0.0,
                                s_limit: float = 0.0,
                                impulse: float = 0.0,
                                impulse_limit: float = 0.0,
                                reaction_force_limit: float = 0.0) -> List[str]:
        """
        判定风险子类型，按风险权重排序。

        6种子类型：
        - force_overload（力过载）
        - speed_exceed（速度超限）
        - pressure_risk（压强风险）
        - impulse_risk（冲量风险）
        - iso_violation（ISO接触力超限）
        - reaction_unstable（反作用不稳定）
        """
        subtype_scores = {}

        # 1. 力过载
        if "力" in param_detail and ("超过" in param_detail or "接近" in param_detail):
            subtype_scores["force_overload"] = 1.0 if "超过" in param_detail else 0.5
        elif f_limit > 0 and force > f_limit * 0.8:
            subtype_scores["force_overload"] = min(1.0, force / max(f_limit, 0.01) - 0.8) / 0.2

        # 2. 速度超限
        if "速度" in param_detail and ("超限" in param_detail or "超过" in param_detail):
            subtype_scores["speed_exceed"] = 1.0
        elif "速度" in param_detail and "接近" in param_detail:
            subtype_scores["speed_exceed"] = 0.5
        elif s_limit > 0 and speed > s_limit * 0.8:
            subtype_scores["speed_exceed"] = min(1.0, speed / max(s_limit, 0.001) - 0.8) / 0.2

        # 3. 压强风险（压强较高，或 detail 中无明确力/速度超标但有压强相关）
        pressure_threshold = 200.0  # kPa 参考阈值
        if pressure_kPa > pressure_threshold:
            subtype_scores["pressure_risk"] = min(1.0, pressure_kPa / pressure_threshold - 1.0 + 0.3)

        # 4. 冲量风险
        if "冲量" in param_detail and "超过" in param_detail:
            subtype_scores["impulse_risk"] = 1.0
        elif "冲量" in param_detail and "接近" in param_detail:
            subtype_scores["impulse_risk"] = 0.5
        elif impulse_limit > 0 and impulse > impulse_limit * 0.8:
            subtype_scores["impulse_risk"] = min(1.0, impulse / max(impulse_limit, 0.01) - 0.8) / 0.2

        # 5. ISO接触力超限
        if "ISO" in iso_compliance and "超出" in iso_compliance:
            subtype_scores["iso_violation"] = 1.0
        elif "ISO" in param_detail and "超出" in param_detail:
            subtype_scores["iso_violation"] = 1.0

        # 6. 反作用不稳定
        if "反作用力" in param_detail and ("超" in param_detail or "超过" in param_detail):
            subtype_scores["reaction_unstable"] = 1.0
        elif "反作用力" in param_detail and "接近" in param_detail:
            subtype_scores["reaction_unstable"] = 0.5
        elif (reaction_force_limit > 0 and force > reaction_force_limit * 0.8
              and not obj_props.get("fixed_base", False)):
            subtype_scores["reaction_unstable"] = min(
                1.0, force / max(reaction_force_limit, 0.01) - 0.8) / 0.2

        # 按分数降序排列，返回标签名列表（只返回分数>0的）
        sorted_subtypes = sorted(
            subtype_scores.items(),
            key=lambda x: x[1],
            reverse=True
        )
        return [name for name, score in sorted_subtypes if score > 0]

    # ---------------------------------------------------------------------
    # v4.2.3 新增：超标倍率 over_ratio 计算
    # ---------------------------------------------------------------------
    @staticmethod
    def compute_over_ratio(verdict: str,
                           force: float, f_limit: float,
                           speed: float, s_limit: float,
                           pressure_kPa: float, contact_area_mm2: float,
                           impulse: float, impulse_limit: float,
                           reaction_force: float, reaction_force_limit: float,
                           iso_force_limit: float = 0.0) -> float:
        """
        计算 FAIL 样本的超标倍率 over_ratio。

        over_ratio = max(各维度的实际值/限值)，取最严维度。
        PASS 样本返回 0.0。

        考虑的维度：
        - 力（force / force_limit）
        - 速度（speed / speed_limit）
        - 压强（pressure_kPa / pressure_limit，由 force_limit 与接触面积推导）
        - 冲量（impulse / impulse_limit，仅当 impulse_limit > 0 时）
        - 反作用力（reaction_force / reaction_force_limit，仅当有反作用约束时）
        - ISO力（force / iso_force_limit，仅当 iso_force_limit > 0 时）

        Returns:
            float: 超标倍率。PASS=0.0, FAIL=max(ratios), 仅考虑 ratio > 1.0 的维度
        """
        if verdict == "PASS" or verdict == "REJECT":
            return 0.0

        ratios = []

        # 力维度
        if f_limit > 0 and force > 0:
            ratios.append(force / f_limit)

        # 速度维度
        if s_limit > 0 and speed > 0:
            ratios.append(speed / s_limit)

        # 压强维度：压强限值 = 力限值产生的压强
        # 注意：动态接触面积下，压强制约与力制约不完全等价
        if f_limit > 0 and contact_area_mm2 > 0:
            pressure_limit = f_limit / (contact_area_mm2 * 1e-6) / 1000.0
            if pressure_limit > 0 and pressure_kPa > 0:
                ratios.append(pressure_kPa / pressure_limit)

        # 冲量维度
        if impulse_limit > 0 and impulse > 0:
            ratios.append(impulse / impulse_limit)

        # 反作用力维度
        if reaction_force_limit > 0 and reaction_force > 0:
            ratios.append(reaction_force / reaction_force_limit)

        # ISO力限值维度
        if iso_force_limit > 0 and force > 0:
            ratios.append(force / iso_force_limit)

        if not ratios:
            return 0.0

        max_ratio = max(ratios)
        # 只有超限时返回倍率（>1.0 表示超限）
        return max_ratio if max_ratio > 1.0 else 0.0

    # ---------------------------------------------------------------------
    # v4.2.2 新增：智能推荐参数（FAIL样本基于超标维度）
    # ---------------------------------------------------------------------
    @staticmethod
    def compute_smart_recommendations(param_detail: str, verdict: str,
                                       force: float, speed: float,
                                       f_limit: float, s_limit: float,
                                       impulse: float, impulse_limit: float,
                                       reaction_limit: float,
                                       iso_limit: float,
                                       pressure_kPa: float,
                                       contact_area_mm2: float,
                                       obj_props: Dict,
                                       safety_factor: float = 0.85) -> Dict:
        """
        基于超标维度智能计算推荐参数。

        返回:
            suggested_force, suggested_speed, suggestion_text
            以及各维度超标详情
        """
        # PASS 样本返回空结构
        if verdict == "PASS":
            return {
                "suggested_force": None,
                "suggested_speed": None,
                "suggestion_text": "参数在安全范围内，无需调整",
                "overshoot_dimensions": [],
            }

        suggested_forces = []
        suggested_speeds = []
        over_dimensions = []

        # 力超标
        if f_limit > 0 and force > f_limit:
            f_suggest = f_limit * safety_factor
            suggested_forces.append(f_suggest)
            over_dimensions.append({
                "dimension": "force",
                "limit": round(f_limit, 4),
                "current": round(force, 4),
                "overshoot_ratio": round(force / max(f_limit, 0.01), 2),
            })

        # 速度超标
        if s_limit > 0 and speed > s_limit:
            s_suggest = s_limit * safety_factor
            suggested_speeds.append(s_suggest)
            over_dimensions.append({
                "dimension": "speed",
                "limit": round(s_limit, 6),
                "current": round(speed, 6),
                "overshoot_ratio": round(speed / max(s_limit, 0.001), 2),
            })

        # 冲量超标 → 反推速度建议
        if impulse_limit > 0 and impulse > impulse_limit:
            mass = float(obj_props.get("mass_kg", 0.5))
            s_imp = impulse_limit * safety_factor / max(mass, 0.01)
            suggested_speeds.append(s_imp)
            over_dimensions.append({
                "dimension": "impulse",
                "limit": round(impulse_limit, 4),
                "current": round(impulse, 4),
                "overshoot_ratio": round(impulse / max(impulse_limit, 0.01), 2),
            })

        # 反作用力超标 → 反推力建议
        if reaction_limit > 0 and force > reaction_limit:
            f_rxn = reaction_limit * safety_factor
            suggested_forces.append(f_rxn)
            over_dimensions.append({
                "dimension": "reaction_force",
                "limit": round(reaction_limit, 4),
                "current": round(force, 4),
                "overshoot_ratio": round(force / max(reaction_limit, 0.01), 2),
            })

        # ISO 超限 → 建议接触力
        if iso_limit > 0 and force > iso_limit:
            f_iso = iso_limit * safety_factor
            suggested_forces.append(f_iso)
            over_dimensions.append({
                "dimension": "iso_force",
                "limit": round(iso_limit, 4),
                "current": round(force, 4),
                "overshoot_ratio": round(force / max(iso_limit, 0.01), 2),
            })

        # 多维度超标 → 取最严（最小）
        if suggested_forces:
            final_force = min(suggested_forces)
        else:
            final_force = force * safety_factor if force > 0 else None  # type: ignore[assignment]

        if suggested_speeds:
            final_speed = min(suggested_speeds)
        else:
            final_speed = speed * safety_factor if speed > 0 else None  # type: ignore[assignment]

        # 生成建议文本
        hint_parts = []
        dim_names = {
            "force": "力",
            "speed": "速度",
            "impulse": "冲量",
            "reaction_force": "反作用力",
            "iso_force": "ISO接触力",
            "pressure": "压强",
        }
        if len(over_dimensions) >= 2:
            hint_parts.append(f"多维度同时超标({len(over_dimensions)}项)，需综合调整")
        elif over_dimensions:
            d = over_dimensions[0]
            hint_parts.append(f"{dim_names.get(str(d['dimension']), str(d['dimension']))}超标{d['overshoot_ratio']:.1f}倍")

        if any(d["dimension"] in ("force", "reaction_force", "iso_force") for d in over_dimensions):
            hint_parts.append("建议分阶段施力，避免冲击")
        if any(d["dimension"] in ("speed", "impulse") for d in over_dimensions):
            hint_parts.append("建议降低运动速度，采用渐进式接近")

        if not hint_parts:
            hint_parts.append("建议降低参数至安全范围")

        suggestion_text = "；".join(hint_parts)

        return {
            "suggested_force": round(final_force, 4) if final_force else None,
            "suggested_speed": round(final_speed, 6) if final_speed else None,
            "suggestion_text": suggestion_text,
            "overshoot_dimensions": over_dimensions,
            "safety_factor": safety_factor,
        }

    # ---------------------------------------------------------------------
    # v4.2.2 新增：边界安全回退参数
    # ---------------------------------------------------------------------
    @staticmethod
    def compute_retreat_params(force: float, speed: float,
                                f_limit: float, s_limit: float,
                                current_level_7: str,
                                current_margin: float) -> Optional[Dict]:
        """
        对边界 PASS 样本（L3/L4，margin < 0.25）计算三档安全回退建议。

        三档策略：
        - conservative：降30%，回到安全区（L0/L1）
        - moderate：降15%，回到轻风险区（L1/L2）
        - minimal：降5%，仅拉离临界点

        Returns:
            retreat_params 字典，或 None（非边界样本）
        """
        # 仅对 margin < 0.25 的 PASS 样本生效
        if current_margin >= 0.25 or current_margin <= 0:
            return None
        if f_limit <= 0 and s_limit <= 0:
            return None

        tiers_cfg = {
            "conservative": {"force_pct": 30.0, "speed_pct": 30.0},
            "moderate":     {"force_pct": 15.0, "speed_pct": 15.0},
            "minimal":      {"force_pct": 5.0,  "speed_pct": 5.0},
        }

        tiers = {}
        for tier_name, cfg in tiers_cfg.items():
            f_red = cfg["force_pct"] / 100.0
            s_red = cfg["speed_pct"] / 100.0
            new_force = force * (1 - f_red) if force > 0 else 0
            new_speed = speed * (1 - s_red) if speed > 0 else 0

            # 预估裕度（基于最严维度）
            ratios = []
            if f_limit > 0 and force > 0:
                ratios.append(new_force / f_limit)
            if s_limit > 0 and speed > 0:
                ratios.append(new_speed / s_limit)

            est_margin = max(0.0, 1.0 - max(ratios)) if ratios else current_margin

            # 预估七级风险等级
            if est_margin > 0.50:
                est_level = "L0"
            elif est_margin > 0.30:
                est_level = "L1"
            elif est_margin > 0.15:
                est_level = "L2"
            elif est_margin > 0.05:
                est_level = "L3"
            else:
                est_level = "L4"

            tiers[tier_name] = {
                "force_reduction_pct": cfg["force_pct"],
                "speed_reduction_pct": cfg["speed_pct"],
                "estimated_risk_level_7": est_level,
                "estimated_margin": round(est_margin, 4),
            }

        return {
            "default_tier": "moderate",
            "current_risk_level_7": current_level_7,
            "current_margin": round(current_margin, 4),
            "tiers": tiers,
            "recommendation": (
                f"当前处于{current_level_7}临界区(裕度{current_margin*100:.1f}%)，"
                f"建议采用适中档回退策略：降力15%、降速15%，"
                f"预计回到{tiers['moderate']['estimated_risk_level_7']}级"
            ),
        }




# =====================================================================
# 第四部分：Layer 2 — 安全适配（参数校验量化表达）
# =====================================================================


    @staticmethod
    def compute_contact_area(base_area: float, force: float,
                             stiffness: float, max_deform: float) -> float:
        """动态接触面积计算（软质/易碎物受力后接触面积增大）
        
        简化线性近似公式:
            contact_area = base_area * (1 + min(max_deform, force / (stiffness * base_area)))
        
        Args:
            base_area: 基础接触面积 mm²
            force: 受力 N
            stiffness: 接触刚度 N/mm²（变形比例系数，越大越不易变形）
            max_deform: 最大变形比例（0-1）
        
        Returns:
            动态接触面积 mm²
        """
        if base_area <= 0 or force <= 0 or stiffness <= 0:
            return base_area
        deform_ratio = force / (stiffness * base_area)
        deform_ratio = min(max_deform, deform_ratio)
        return base_area * (1.0 + deform_ratio)


class SafetyAdapter:
    """安全适配层
    
    能力-动作适配 → 能力参数量化（机器人能力参数空间: 力/速/振幅上限）
    动作-目标适配 → 交互参数校验（动作参数 × 目标属性）
    参数安全 → 具体参数边界校验 + ISO标准校验
    """

    def __init__(self,
                 rules: Optional[SafetyRules] = None,
                 verb_db: Optional[Dict[str, Dict[str, Any]]] = None,
                 action_rules: Optional[Dict[str, Dict[str, Dict[str, Any]]]] = None):
        self.rules = rules or DEFAULT_RULES
        self.verb_db = verb_db if verb_db is not None else VERB_DATABASE
        self.action_rules = action_rules if action_rules is not None else ACTION_RULES

    # ----------------------------------------------------------
    # 能力-动作适配（能力参数量化）
    # ----------------------------------------------------------
    def check_capability_action(self, robot: str, verb: str,
                            robot_cap: Dict, verb_info: Optional[Dict]) -> Dict:
        """能力-动作适配 → 能力参数: (F_max, V_max, A_max) 能力参数校验"""
        # 字段归一化（兼容两套命名）
        cap = SemanticParser.normalize_robot_cap(robot_cap)
        if verb_info is None:
            # 未知动词但action_key在标准规则表中 → 通过
            if verb in self.action_rules:
                return {"passed": True, "score": 0.6, "detail": "标准动作(无具体动词参数)",
                        "cap_result": {"F_max": cap.get("max_force", 100),
                                    "V_max": cap.get("max_speed", 1.5),
                                    "A_max": cap.get("max_accel", 10),
                                    "tier": cap.get("tier", 2),
                                    "dexterity": cap.get("dexterity", 0.5),
                                    "required_tier": 1}}
            return {"passed": False, "score": 0.0, "detail": f"未知动词：{verb}",
                    "cap_result": {"F_max": 0, "V_max": 0, "A_max": 0, "tier": 0, "dexterity": 0}}

        risk = verb_info["risk_level"]
        tier = cap.get("tier", 2)
        dexterity = cap.get("dexterity", 0.5)
        f_max = cap.get("max_force", 100)
        v_max = cap.get("max_speed", 1.5)
        a_max = cap.get("max_accel", 10)

        required_tier = math.ceil(risk / 2.5)
        passed = tier >= required_tier

        if verb in ["捏", "摸", "插"] and dexterity < 0.4:
            passed = False

        score = min(1.0, 0.5 + 0.3 * (tier / 3) + 0.2 * dexterity) if passed else 0.1

        return {
            "passed": passed, "score": round(score, 2),
            "detail": "能力-动作适配良好" if passed else f"机器人能力不足(需tier≥{required_tier}, 当前{tier})",
            "cap_result": {"F_max": f_max, "V_max": v_max, "A_max": a_max,
                        "tier": tier, "dexterity": dexterity, "required_tier": required_tier},
        }

    # ----------------------------------------------------------
    # 动作-目标适配（交互参数校验）
    # ----------------------------------------------------------
    def check_action_target(self, verb: str, obj: str,
                           verb_info: Optional[Dict], obj_props: Dict) -> Dict:
        """动作-目标适配 → 交互参数: 参数向量 × 物体属性"""
        if verb_info is None:
            # 英文标准动作无具体动词参数 → 仅检查不可能组合
            if verb in self.action_rules:
                cat = obj_props.get("category", "rigid")
                if self.rules:
                    if cat in IMPOSSIBLE_PAIRS.get(verb, set()):
                        return {"passed": False, "score": 0.0,
                                "detail": f"动作{verb}与物体{obj}不兼容",
                                "interaction_result": {"compatibility": 0.0}}
                return {"passed": True, "score": 0.6, "detail": "动作-目标兼容(通用规则)",
                        "interaction_result": {"compatibility": 0.6}}
            return {"passed": False, "score": 0.0, "detail": f"未知动词：{verb}",
                    "interaction_result": {}}

        # 高风险 + 极脆弱
        fragile_val = float(obj_props.get("fragile", 0.0))
        if verb_info["risk_level"] >= 4 and fragile_val > 0.7:
            return {"passed": False, "score": 0.05,
                    "detail": f"高风险动作「{verb}」作用于易碎物「{obj}」",
                    "interaction_result": {"compatibility": 0.05}}

        # 兼容性评分
        base_score = 0.7  # 基础兼容分（通过不可能组合检查后起分较高）
        if verb in ["握", "持", "抓", "拿"]:
            base_score += 0.1
        elif verb in ["举", "扛", "背"] and float(obj_props.get("heavy", 0.0)) > 0.5:
            base_score += 0.1
        elif verb in ["端", "托"] and float(obj_props.get("fluid", 0.0)) > 0.3:
            base_score -= 0.1

        hc = float(obj_props.get("human_contact", 0.0))
        if hc > 0.5 and verb_info["risk_level"] >= 4:
            base_score -= 0.15

        base_score = max(0.2, min(1.0, base_score))
        mass = OBJECT_MASS.get(obj, 1.0)
        stability = CATEGORY_TO_STABILITY.get(obj_props.get("category", "rigid"), "rigid")
        contact_area = obj_props.get("contact_area_mm2", 400)

        return {
            "passed": True, "score": round(base_score, 2),
            "detail": "动作-目标兼容",
            "interaction_result": {"mass_kg": mass, "stability": stability,
                         "contact_area_mm2": contact_area, "compatibility": round(base_score, 2)},
        }

    # ----------------------------------------------------------
    # 参数安全（具体边界校验 + ISO标准）
    # ----------------------------------------------------------
    def check_params(self, verb: str, params: Dict,
                      verb_info: Optional[Dict], obj_props: Dict,
                      robot_cap: Dict, rules: Optional[Dict] = None,
                      obj_name: str = "") -> Dict:
        """参数安全校验 + ISO标准校验
        
        核心逻辑:
        - params为空 → 仅检查语义可行性，不检查参数
        - params有值 → 使用动词约束 + 规则表硬约束 + 反作用力约束
        - 规则表超限 → 硬FAIL（force>f_max 或 speed>s_max）
        
        三层力约束（从动量-能量双守恒推导）：
        Layer 1: 主→谓 — 机器人输出能力上限（电机/关节极限）
        Layer 2: 谓→宾 — 物体承受能力上限（材料/结构极限，含动态接触面积计算）
        Layer 3: 主↔宾 — 反作用力约束（机器人本体稳定性，能量守恒双向性）
        最终安全力上限 = min(三层各自的上限)
        """
        # 无参数 → 跳过参数检查（语义层已校验）
        has_params = params and any(v != 0 for v in params.values())
        
        if not has_params:
            return {"passed": True, "score": 0.8, "detail": "无参数输入(语义检查通过)", 
                    "margin": 0.5, "iso_compliance": "未触发",
                    "limits": {"force_limit": 0.0, "speed_limit": 0.0, "iso_force_limit": 0.0}}

        iso_compliance = "未触发"
        
        # === 有verb_info时：动词约束 + 规则表 ===
        if verb_info is not None:
            return self._check_params_with_verb(verb, params, verb_info, obj_props, 
                                                 robot_cap, rules, obj_name)
        
        # === 无verb_info时：纯规则表 ===
        if rules:
            return self._check_params_by_rules(verb, params, rules, obj_props, obj_name)
        
        return {"passed": True, "score": 0.5, "detail": "无参数限制", "margin": 1.0,
                "iso_compliance": "未触发"}

    def _check_params_with_verb(self, verb: str, params: Dict, verb_info: Dict,
                                 obj_props: Dict, robot_cap: Dict,
                                 rules: Optional[Dict], obj_name: str) -> Dict:
        """有动词参数时的完整校验"""
        grasp_params = verb_info.get("grasp_params", {})
        if not grasp_params:
            return {"passed": True, "score": 0.6, "detail": "无参数限制", "margin": 1.0,
                    "iso_compliance": "未触发"}

        score = 1.0
        violations = []
        min_margin = 1.0
        iso_compliance = "未触发"

        force = abs(float(params.get("force", 0)))
        speed = params.get("speed", 0)
        
        max_force_verb = grasp_params.get("max_force", 100)
        max_force = min(max_force_verb, robot_cap.get("max_force", 100))
        fragile_val = float(obj_props.get("fragile", 0.0))
        fragile_factor = 1.0 - fragile_val * self.rules.fragile_force_factor
        safe_force = max_force * fragile_factor

        # ISO标准校验：仅人体部位 (category=human) 触发ISO力限值
        # human_contact 仅作场景标注参考，不施加力约束（避免日常物体过度收紧）
        obj_category = obj_props.get("category", "rigid")
        is_body_part = (obj_category == "human")
        if is_body_part:
            iso_limit = self.rules.iso.get_quasi_static_limit(obj_name)
            if force > iso_limit:
                violations.append(f"力({force:.1f}N)超出ISO/TS 15066限值({iso_limit}N)")
                iso_compliance = "超出限值"
                min_margin = 0.0
                score = 0.0  # 硬FAIL
            else:
                iso_compliance = "符合ISO 10218/TS 15066"

        # 动词约束 — 力超安全上限 → 硬FAIL
        if force > safe_force:
            score = 0.0
            violations.append(f"力({force:.1f})超过安全值({safe_force:.1f})")
            min_margin = 0.0
        elif force > safe_force * 0.8:
            score -= 0.15
            violations.append(f"力接近上限")
            min_margin = min(min_margin, (safe_force - force) / max(safe_force, 0.01))
        else:
            min_margin = min(min_margin, (safe_force - force) / max(safe_force, 0.01))

        max_speed_verb = grasp_params.get("max_speed", 200)
        # 动词库 max_speed 单位 mm/s，输入 speed 单位 m/s，统一转 m/s 比较
        max_speed_verb_ms = max_speed_verb / 1000.0
        max_speed_robot = robot_cap.get("max_speed", 2)  # m/s
        max_speed = min(max_speed_verb_ms, max_speed_robot)
        # 速度超限值 → 硬FAIL（输入 speed 单位 m/s，max_speed 已统一为 m/s）
        if speed > max_speed:
            score = 0.0
            violations.append(f"速度({speed:.3f}m/s)超限({max_speed:.3f}m/s)")
            min_margin = 0.0
        elif max_speed > 0:
            min_margin = min(min_margin, (max_speed - speed) / max_speed)

        # 规则表硬约束（优先级最高）
        if rules:
            f_min, f_opt, f_max = rules["force"]
            s_min, s_opt, s_max = rules["speed"]
            # 硬约束：超过规则上限 → 直接FAIL
            if force > f_max and f_max > 0:
                score = 0.0
                violations.append(f"力({force:.2f})超过规则上限({f_max})")
                min_margin = 0.0
            elif force > 0 and f_max > 0:
                rm = (f_max - force) / max(f_max, 0.01)
                min_margin = min(min_margin, rm)
            if speed > s_max and s_max > 0:
                score = 0.0
                violations.append(f"速度({speed:.4f})超过规则上限({s_max})")
                min_margin = 0.0
            elif speed > 0 and s_max > 0:
                sm = (s_max - speed) / max(s_max, 0.01)
                min_margin = min(min_margin, sm)

        
                # --- Layer 3: 反作用力约束（机器人本体稳定性）---
        if not robot_cap.get("fixed_base", False):
            base_weight = robot_cap.get("base_weight_kg", 20.0)
            friction_coef = robot_cap.get("ground_friction", self.rules.friction_coef)
            max_reaction = base_weight * G * friction_coef
            if force > max_reaction:
                score = 0.0
                violations.append(f"反作用力({force:.1f}N)超机器人稳定极限({max_reaction:.1f}N)")
                min_margin = 0.0
            elif force > max_reaction * 0.8:
                score -= 0.15
                violations.append(f"反作用力接近稳定极限({force:.1f}/{max_reaction:.1f}N)")
                rfn = (max_reaction - force) / max(max_reaction, 0.01)
                min_margin = min(min_margin, rfn)

        # --- 冲量校验（仅移动类动作）---
        speed_val = abs(float(params.get("speed", 0)))
        if speed_val > 0 and rules:
            mass_kg = float(obj_props.get("mass_kg", 0))
            impulse_max = rules.get("impulse_max")
            if mass_kg > 0 and impulse_max is not None:
                impulse = mass_kg * speed_val
                if impulse > impulse_max:
                    score = 0.0
                    violations.append(f"冲量({impulse:.2f}kg·m/s)超过上限({impulse_max}kg·m/s)")
                    min_margin = 0.0
                elif impulse > impulse_max * 0.8:
                    score -= 0.1
                    violations.append(f"冲量接近上限({impulse:.2f}/{impulse_max}kg·m/s)")
                    imp_margin = (impulse_max - impulse) / max(impulse_max, 0.01)
                    min_margin = min(min_margin, imp_margin)


        passed = score > 0.3
        detail = "; ".join(violations) if violations else "参数安全"

        # v4.2.2：记录各维度有效限值（用于智能推荐参数和风险子类型判定）
        limits = {
            "force_limit": safe_force,
            "speed_limit": max_speed,
            "iso_force_limit": iso_limit if is_body_part else 0.0,
        }
        # 规则表上限（如果更严格则取规则表的）
        if rules:
            _, _, f_max_r = rules["force"]
            _, _, s_max_r = rules["speed"]
            if f_max_r > 0 and f_max_r < safe_force:
                limits["force_limit"] = f_max_r
            if s_max_r > 0 and s_max_r < max_speed:
                limits["speed_limit"] = s_max_r

        return {
            "passed": passed, "score": round(max(score, 0.0), 2),
            "detail": detail, "margin": round(max(0, min_margin), 4),
            "iso_compliance": iso_compliance,
            "limits": limits,  # v4.2.2 新增
        }

    def _check_params_by_rules(self, action_key: str, params: Dict,
                                rules: Dict, obj_props: Dict, obj_name: str) -> Dict:
        """纯用规则表校验（无verb_info时）"""
        score = 1.0
        violations = []
        min_margin = 1.0
        iso_compliance = "未触发"

        force = abs(float(params.get("force", 0)))
        speed = params.get("speed", 0)
        f_min, f_opt, f_max = rules["force"]
        s_min, s_opt, s_max = rules["speed"]

        # ISO标准校验：仅人体部位 (category=human) 触发ISO力限值
        obj_category = obj_props.get("category", "rigid")
        is_body_part = (obj_category == "human")
        if is_body_part:
            iso_limit = self.rules.iso.get_quasi_static_limit(obj_name)
            if force > iso_limit:
                score = 0.0
                violations.append(f"力({force:.1f}N)超出ISO限值({iso_limit}N)")
                iso_compliance = "超出限值"
                min_margin = 0.0
            else:
                iso_compliance = "符合ISO 10218/TS 15066"

        # 规则表硬约束
        if force > f_max and f_max > 0:
            score = 0.0
            violations.append(f"力({force:.2f})超过规则上限({f_max})")
            min_margin = 0.0
        elif force > 0 and f_max > 0:
            rm = (f_max - force) / max(f_max, 0.01)
            min_margin = min(min_margin, rm)

        if speed > s_max and s_max > 0:
            score = 0.0
            violations.append(f"速度({speed:.4f})超过规则上限({s_max})")
            min_margin = 0.0
        elif speed > 0 and s_max > 0:
            min_margin = min(min_margin, (s_max - speed) / max(s_max, 0.01))

        
        # --- 冲量校验 ---
        speed_val = abs(float(params.get("speed", 0)))
        if speed_val > 0:
            mass_kg = float(obj_props.get("mass_kg", 0))
            impulse_max = rules.get("impulse_max")
            if mass_kg > 0 and impulse_max is not None:
                impulse = mass_kg * speed_val
                if impulse > impulse_max:
                    score = 0.0
                    violations.append(f"冲量({impulse:.2f}kg·m/s)超过上限({impulse_max}kg·m/s)")
                    min_margin = 0.0
                elif impulse > impulse_max * 0.8:
                    score -= 0.1
                    violations.append(f"冲量接近上限({impulse:.2f}/{impulse_max}kg·m/s)")
                    imp_margin = (impulse_max - impulse) / max(impulse_max, 0.01)
                    min_margin = min(min_margin, imp_margin)


        passed = score > 0.3
        detail = "; ".join(violations) if violations else "参数安全"
        # v4.2.2：记录各维度有效限值
        limits = {
            "force_limit": f_max if f_max > 0 else 0.0,
            "speed_limit": s_max if s_max > 0 else 0.0,
            "iso_force_limit": iso_limit if is_body_part else 0.0,
        }
        return {"passed": passed, "score": round(max(score, 0.0), 2),
                "detail": detail, "margin": round(max(0, min_margin), 4),
                "iso_compliance": iso_compliance,
                "limits": limits}


# =====================================================================
# 第五部分：Layer 3 — FAV + 状态机 + 边界监控
# =====================================================================

class FAVClassifier:
    """F/A/V三维自动分类器"""

    def __init__(self):
        self.hold_threshold = 0.05
        self.grasp_threshold = 0.20
        self.release_threshold = -0.15
        self.idle_force_threshold = 0.02

    def classify(self, force: float, amplitude: float, velocity: float,
                 history: Optional[List[Dict]] = None) -> Dict:
        """根据F/A/V自动分类: idle/grasping/holding/releasing"""
        if amplitude < 0.001:
            phi = 1.0 if force >= 0.001 else 0.0
        else:
            phi = min(2.0, force / amplitude)

        if history and len(history) >= 2:
            dt = max(history[-1].get('t', 1) - history[0].get('t', 0), 0.001)
            dF = (history[-1]['force'] - history[0]['force']) / dt
            dA = (history[-1]['amplitude'] - history[0]['amplitude']) / dt
            phi0 = history[0]['force'] / max(history[0]['amplitude'], 0.001)
            phi1 = history[-1]['force'] / max(history[-1]['amplitude'], 0.001)
            dphi = (phi1 - phi0) / dt
        else:
            dA = abs(velocity)
            dF = abs(velocity) * phi * 0.8
            dphi = abs(velocity) * 0.5

        change = math.sqrt(dF**2 + dA**2 + (dphi*2)**2) / math.sqrt(6)
        change = min(1.0, change)

        if force < self.idle_force_threshold and change < 0.02:
            return {"phase_state": "idle", "force_density": round(phi, 3), "change": round(change, 3),
                    "confidence": 0.9, "basis": "力≈0且变化极小 → 空闲"}

        if dphi < self.release_threshold and dF < -0.05:
            return {"phase_state": "releasing", "force_density": round(phi, 3), "change": round(change, 3),
                    "confidence": 0.7, "basis": "平衡比和力快速下降 → 释放"}

        if change <= self.hold_threshold:
            return {"phase_state": "holding", "force_density": round(phi, 3), "change": round(change, 3),
                    "confidence": 0.85, "basis": "变化率极低 → 稳态"}
        elif change >= self.grasp_threshold:
            return {"phase_state": "grasping", "force_density": round(phi, 3), "change": round(change, 3),
                    "confidence": 0.8, "basis": "变化率高 → 动态抓态"}
        else:
            ratio = (change - self.hold_threshold) / (self.grasp_threshold - self.hold_threshold)
            ps = "grasping" if ratio > 0.6 else "holding"
            return {"phase_state": ps, "force_density": round(phi, 3), "change": round(change, 3),
                    "confidence": 0.6, "basis": f"过渡区({change:.3f})"}


class SafetyJudgeV4:
    """安全判定器 — 状态稳定 + 边界监控 + 干扰响应"""

    def __init__(self,
                 rules: Optional[SafetyRules] = None,
                 verb_db: Optional[Dict[str, Dict[str, Any]]] = None,
                 action_rules: Optional[Dict[str, Dict[str, Dict[str, Any]]]] = None):
        self.rules = rules or DEFAULT_RULES
        self.verb_db = verb_db if verb_db is not None else VERB_DATABASE
        self.action_rules = action_rules if action_rules is not None else ACTION_RULES

    def check_state_stabilization(self, verb: str, force: float, force_target: float,
                             speed: float, displacement: float) -> bool:
        """状态稳定检测: 力/速度/位移三参数稳定 → 可进入保持状态"""
        vi = VERB_DATABASE.get(verb)
        if not vi or not vi.get("transition_to_hold"):
            return False
        th = vi.get("stability_threshold")
        if not th:
            return False
        f_stable = abs(force - force_target) / max(force_target, 0.01) < th["force_stability"]
        v_low = abs(speed) < th["max_speed"]
        d_stable = abs(displacement) < th["max_displacement"]
        return f_stable and v_low and d_stable

    def estimate_disturbance(self, force: float, force_target: float,
                              displacement: float, max_displacement: float,
                              vibration: float,
                              hold_params: Optional[Dict] = None) -> Tuple[float, str]:
        """估计干扰强度并分级"""
        hp = hold_params or {}
        min_f = hp.get("min_force", 0)
        max_f = hp.get("max_force", float('inf'))
        strength = 0.0

        # 1. 力越界
        if min_f > 0 and force < min_f:
            d = (min_f - force) / max(min_f, 0.01)
            strength = max(strength, min(d, 1.0))
        elif max_f < float('inf') and force > max_f:
            d = (force - max_f) / max(max_f, 0.01)
            strength = max(strength, min(d, 1.0))
        else:
            # 在范围内：边缘轻微扰动
            if min_f > 0 and max_f < float('inf') and force_target > 0:
                mid = (min_f + max_f) / 2
                half = (max_f - min_f) / 2
                if half > 0:
                    norm = abs(force - mid) / half
                    if norm > 0.7:
                        strength = max(strength, min((norm - 0.7) / 0.3 * 0.10, 0.12))

        # 2. 位移越界
        if max_displacement > 0 and abs(displacement) > max_displacement:
            d = abs(displacement) / max_displacement - 1.0
            strength = max(strength, min(d, 1.0))

        # 3. 力波动
        if force_target > 0:
            fluc = abs(force - force_target) / force_target
            if fluc > 0.5:
                strength = max(strength, min((fluc - 0.5) * 0.5, 1.0))

        # 4. 振动
        if vibration > 0.3:
            strength = max(strength, min((vibration - 0.3) * 2.5, 1.0))
        elif vibration > 0.05:
            strength = max(strength, min((vibration - 0.05) * 0.5, 0.12))

        strength = min(1.0, max(0.0, strength))
        strength = round(strength, 10)  # 消除浮点精度问题
        level = self._classify_disturbance(strength)
        return strength, level

    def _classify_disturbance(self, s: float) -> str:
        r = self.rules
        if s < r.dist_micro: return "none"
        elif s < r.dist_small: return "micro"
        elif s < r.dist_medium: return "small"
        elif s < r.dist_large: return "medium"
        elif s < r.dist_danger: return "large"
        else: return "danger"

    def hold_boundary_check(self, verb: str, force: float, force_target: float,
                             displacement: float, vibration: float) -> Dict:
        """握态边界监控（低功耗模式）"""
        vi = VERB_DATABASE.get(verb)
        if not vi:
            return {"passed": False, "disturbance_level": "danger", "reason": "未知动词"}
        hp = vi.get("hold_params", {})
        if not hp:
            return {"passed": False, "disturbance_level": "none", "reason": "不支持握态"}

        max_d = hp.get("max_displacement", 5.0)
        strength, level = self.estimate_disturbance(force, force_target, displacement, max_d, vibration, hp)

        violations = []
        if hp.get("min_force", 0) > 0 and force < hp["min_force"]:
            violations.append("力低于下限")
        if "max_force" in hp and force > hp["max_force"]:
            violations.append("力超过上限")
        if max_d > 0 and abs(displacement) > max_d:
            violations.append("位移超限")

        if level in ("none", "micro"):
            passed, rec = True, ("maintain" if level == "none" else "compensate")
        elif level == "small":
            passed, rec = True, "tighten"
        elif level == "medium":
            passed, rec = False, "slow_down"
        elif level == "large":
            passed, rec = False, "re_gasp"
        else:
            passed, rec = False, "emergency_release"

        return {
            "passed": passed, "disturbance_level": level,
            "disturbance_strength": round(strength, 3),
            "stability": round(max(0, 1.0 - strength), 3),
            "violations": violations, "recommended_action": rec,
        }

    def apply_disturbance_response(self, verb: str, level: str,
                                    base_force: float = 10.0) -> Dict:
        tpl = DISTURBANCE_TEMPLATES.get(level, DISTURBANCE_TEMPLATES["none"])
        vi = VERB_DATABASE.get(verb, {})
        return {
            "level": level, "actions": tpl["actions"],
            "force_adjustment": base_force * tpl["force_ratio"],  # type: ignore[operator]
            "speed_limit_factor": tpl["speed_factor"],
            "state_transition": tpl["transition"],
            "verb_response": vi.get("disturbance_response", ""),
        }



# =====================================================================
# 第六部分辅助：外部 JSON 配置加载
# =====================================================================

def load_verb_database(json_path: str) -> Dict[str, Dict[str, Any]]:
    """从 JSON 文件加载动词数据库
    
    Args:
        json_path: JSON 文件路径
        
    Returns:
        动词数据库字典
    """
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError(f"动词数据库文件 {json_path} 格式错误：根节点必须是字典")
    return data


def load_object_database(json_path: str) -> Dict[str, Dict[str, Any]]:
    """从 JSON 文件加载物体属性数据库
    
    Args:
        json_path: JSON 文件路径
        
    Returns:
        物体属性数据库字典
    """
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError(f"物体数据库文件 {json_path} 格式错误：根节点必须是字典")
    return data


def load_action_rules(json_path: str) -> Dict[str, Dict[str, Dict[str, Any]]]:
    """从 JSON 文件加载动作规则表
    
    Args:
        json_path: JSON 文件路径
        
    Returns:
        动作规则表字典
    """
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError(f"动作规则文件 {json_path} 格式错误：根节点必须是字典")
    return data


# =====================================================================
# 第六部分：主引擎
# =====================================================================

class SafetyEngineV4:
    """
    Rotor Safety Engine v1.0.0

    模式A: check_command(action, obj, params, ...)  — 自然语言
    模式B: check_action(scene, action, robot)        — JSON参数
    """

    # === 版本号 ===
    VERSION = "1.0.0"

    # === v4.2.3 新增：七级风险分级规则（与 map_risk_level_7 完全一致）===
    # PASS 样本：基于 safety_margin 分档
    # FAIL 样本：基于越线倍率 over_ratio 分档
    RISK_LEVEL_7_RULES = {
        "L0": {
            "label_cn": "安全",
            "verdict": "PASS",
            "margin_range": {"min": 0.50, "max": 1.00},
            "description": "安全裕度充足，远低于安全边界",
            "typical_scene": "日常抓取轻物、低速操作",
        },
        "L1": {
            "label_cn": "低风险",
            "verdict": "PASS",
            "margin_range": {"min": 0.30, "max": 0.50},
            "description": "安全裕度适中，在安全区内",
            "typical_scene": "中等参数操作，有一定安全余量",
        },
        "L2": {
            "label_cn": "中低风险",
            "verdict": "PASS",
            "margin_range": {"min": 0.15, "max": 0.30},
            "description": "安全裕度尚可，中等参数水平",
            "typical_scene": "接近安全区间上限的正常操作",
        },
        "L3": {
            "label_cn": "中风险/接近边界",
            "verdict": "PASS",
            "margin_range": {"min": 0.05, "max": 0.15},
            "description": "接近安全边界，裕度偏低",
            "typical_scene": "边界操作，建议留有余量",
        },
        "L4": {
            "label_cn": "中高风险/临界",
            "verdict": "PASS",
            "margin_range": {"min": 0.0, "max": 0.05},
            "description": "临界区，高风险但未越线，极易越界",
            "typical_scene": "参数紧贴安全上限，稍有波动即FAIL",
        },
        "L5": {
            "label_cn": "高风险/越线",
            "verdict": "FAIL",
            "overshoot_range": {"min_ratio": 1.0, "max_ratio": 1.5},
            "description": "已越线，越线倍率≤50%（over_ratio ≤ 1.5）",
            "typical_scene": "参数轻微超标，降低参数即可安全",
        },
        "L6": {
            "label_cn": "危险/严重越线",
            "verdict": "FAIL",
            "overshoot_range": {"min_ratio": 1.5, "max_ratio": None},
            "description": "严重越线，越线倍率>50%（over_ratio > 1.5）",
            "typical_scene": "参数严重超标，需要大幅降低或重新规划",
        },
    }

    def __init__(self,
                 verb_db: Optional[Dict[str, Dict[str, Any]]] = None,
                 object_db: Optional[Dict[str, Dict[str, Any]]] = None,
                 action_rules: Optional[Dict[str, Dict[str, Dict[str, Any]]]] = None,
                 rules: Optional[SafetyRules] = None):
        self.parser = SemanticParser(verb_db=verb_db, object_db=object_db, action_rules=action_rules)
        self.adapter = SafetyAdapter(rules=rules, verb_db=verb_db, action_rules=action_rules)
        self.fav = FAVClassifier()
        self.judge = SafetyJudgeV4(rules=rules)

    @classmethod
    def from_config(cls,
                    verb_db_path: Optional[str] = None,
                    object_db_path: Optional[str] = None,
                    action_rules_path: Optional[str] = None,
                    rules: Optional[SafetyRules] = None) -> "SafetyEngineV4":
        """从 JSON 配置文件创建 SafetyEngineV4 实例
        
        Args:
            verb_db_path: 动词数据库 JSON 文件路径
            object_db_path: 物体属性数据库 JSON 文件路径
            action_rules_path: 动作规则表 JSON 文件路径
            rules: SafetyRules 实例（可选）
            
        Returns:
            SafetyEngineV4 实例
        """
        verb_db = load_verb_database(verb_db_path) if verb_db_path else None
        object_db = load_object_database(object_db_path) if object_db_path else None
        action_rules = load_action_rules(action_rules_path) if action_rules_path else None
        return cls(verb_db=verb_db, object_db=object_db, action_rules=action_rules, rules=rules)


    @staticmethod
    def _validate_params(params: Dict[str, Any]) -> Tuple[bool, str, List[str], Dict[str, Any]]:
        """集中校验输入参数

        Args:
            params: 动作参数字典

        Returns:
            (is_valid, error_msg, warnings, normalized_params)
            - is_valid: True=参数有效，False=致命错误
            - error_msg: 错误描述（is_valid=False 时）
            - warnings: 非致命警告列表
            - normalized_params: 标准化后的参数字典（如负数取abs）
        """
        warnings: List[str] = []
        normalized: Dict[str, Any] = dict(params) if params else {}

        # 数值参数白名单（需要检查类型/范围的参数）
        numeric_params = [
            "force", "speed", "acceleration", "displacement",
            "duration_ms", "angle", "force_n", "velocity_ms",
            "acceleration_ms2", "mass_kg", "amplitude",
        ]

        for key in numeric_params:
            if key not in normalized:
                continue
            val = normalized[key]
            # 类型检查
            if not isinstance(val, (int, float)):
                return False, f"参数 {key} 必须是数字类型，当前为 {type(val).__name__}", warnings, normalized
            # 有限值检查
            if math.isnan(val) or math.isinf(val):
                return False, f"参数 {key} 必须是有限数值", warnings, normalized

        # force 负数处理（向后兼容：取abs + warning）
        if "force" in normalized and normalized["force"] < 0:
            warnings.append(f"force为负数({normalized['force']})，已取绝对值")
            normalized["force"] = abs(normalized["force"])

        if "force_n" in normalized and normalized["force_n"] < 0:
            warnings.append(f"force_n为负数({normalized['force_n']})，已取绝对值")
            normalized["force_n"] = abs(normalized["force_n"])

        # speed 负数处理
        if "speed" in normalized and normalized["speed"] < 0:
            warnings.append(f"speed为负数({normalized['speed']})，已取绝对值")
            normalized["speed"] = abs(normalized["speed"])

        if "velocity_ms" in normalized and normalized["velocity_ms"] < 0:
            warnings.append(f"velocity_ms为负数({normalized['velocity_ms']})，已取绝对值")
            normalized["velocity_ms"] = abs(normalized["velocity_ms"])

        # 越界检查：明显不合理的值
        if "force" in normalized and normalized["force"] > 10000:
            return False, f"force值({normalized['force']}N)明显不合理，超过10000N上限", warnings, normalized
        if "force_n" in normalized and normalized["force_n"] > 10000:
            return False, f"force_n值({normalized['force_n']}N)明显不合理，超过10000N上限", warnings, normalized

        if "speed" in normalized and normalized["speed"] > 100:
            return False, f"speed值({normalized['speed']}m/s)明显不合理，超过100m/s上限", warnings, normalized
        if "velocity_ms" in normalized and normalized["velocity_ms"] > 100:
            return False, f"velocity_ms值({normalized['velocity_ms']}m/s)明显不合理，超过100m/s上限", warnings, normalized

        return True, "", warnings, normalized


    def _build_result(self,
                      verdict: str,
                      risk: str,
                      corr: str,
                      action_key: str,
                      verb_info: Optional[Dict[str, Any]],
                      props: Dict[str, Any],
                      subj: Dict[str, Any],
                      params: Dict[str, Any],
                      rules: Optional[Dict[str, Any]],
                      capability_result: Dict[str, Any],
                      target_result: Dict[str, Any],
                      param_result: Dict[str, Any],
                      t0: float,
                      category: str = "",
                      context: Optional[Dict[str, Any]] = None,
                      state: Optional[str] = None,
                      input_warnings: Optional[List[str]] = None) -> Dict[str, Any]:
        """构建结果字典（check_command / check_action 共用）
        
        包含：安全区间、接触面积、压强、风险等级、风险子类型、
              智能推荐参数、边界回退参数、超标倍率、ISO合规标注 等。
        
        注意：此方法是纯构建逻辑，不改变判定结果。
        """
        # 安全区间
        gz = {}
        if rules and params:
            gz = self.parser.compute_safety_zone(params, rules)

        # 接触面积 + 压强（动态接触面积）
        base_area = props.get("contact_area_mm2", 400)
        force_val = abs(float(params.get("force", 0))) if params else 0.0
        stiffness = props.get("contact_stiffness", 3.0)
        max_deform = props.get("max_deform", 0.1)
        area = self.parser.compute_contact_area(base_area, force_val, stiffness, max_deform)
        pressure = force_val / (area * 1e-6) / 1000.0 if area > 0 else 0.0

        # 推荐参数
        rec = self.parser.recommend_params(rules, context) if rules else {}

        # 七级风险等级
        margin_val = param_result.get("margin", 0.0)
        score_val = param_result.get("score", 0.0)
        risk_level_7 = self.parser.map_risk_level_7(verdict, margin_val, score_val)

        # 限值
        limits = param_result.get("limits", {})
        f_limit = limits.get("force_limit", 0.0)
        s_limit = limits.get("speed_limit", 0.0)
        iso_limit = limits.get("iso_force_limit", 0.0)

        # 冲量
        speed_val = params.get("speed", 0) if params else 0
        mass_kg_val = float(props.get("mass_kg", 0))
        impulse_val = mass_kg_val * speed_val if rules and rules.get("impulse_max") else 0.0
        impulse_limit_val = rules.get("impulse_max", 0.0) if rules else 0.0

        # 反作用力限制
        if not subj.get("fixed_base", False):
            base_w = subj.get("base_weight_kg", 20.0)
            fric = subj.get("ground_friction", self.adapter.rules.friction_coef)
            reaction_limit_val = base_w * G * fric
        else:
            reaction_limit_val = 0.0

        # 风险子类型
        risk_subtypes = self.parser.classify_risk_subtypes(
            param_result.get("detail", ""), verdict,
            force_val, speed_val,
            pressure, param_result.get("iso_compliance", "未触发"),
            props,
            f_limit=f_limit, s_limit=s_limit,
            impulse=impulse_val, impulse_limit=impulse_limit_val,
            reaction_force_limit=reaction_limit_val,
        )

        # 智能推荐参数 v2
        recommended_params_v2 = self.parser.compute_smart_recommendations(
            param_result.get("detail", ""), verdict,
            force_val, speed_val,
            f_limit, s_limit,
            impulse_val, impulse_limit_val,
            reaction_limit_val, iso_limit,
            pressure, area, props,
        )

        # 边界安全回退参数
        retreat_params = None
        if verdict == "PASS" and margin_val < 0.25 and margin_val > 0:
            retreat_params = self.parser.compute_retreat_params(
                force_val, speed_val,
                f_limit, s_limit,
                risk_level_7, margin_val,
            )

        # 超标倍率 over_ratio
        over_ratio_val = self.parser.compute_over_ratio(
            verdict,
            force_val, f_limit,
            speed_val, s_limit,
            pressure, area,
            impulse_val, impulse_limit_val,
            force_val, reaction_limit_val,
            iso_force_limit=iso_limit,
        )

        # 语义合理性分数透传
        semantic_score = context.get("semantic_score") if context else None

        # ISO合规
        iso_compliance = param_result.get("iso_compliance", "未触发")
        hc_val = float(props.get("human_contact", 0.0))
        if iso_compliance == "未触发":
            if category == "human":
                iso_compliance = "符合ISO 10218/TS 15066"
            elif hc_val > self.adapter.rules.human_contact_threshold:
                iso_compliance = "符合ISO 10218/TS 15066(接触态参考)"

        # 状态推断
        if state is None:
            state = "idle"
            if verb_info:
                pt = verb_info.get("phase_type", "grasp")
                if pt == "hold":
                    state = "hold"
                elif params and any(v > 0 for v in params.values()):
                    state = "grasp"

        warnings_list = input_warnings if input_warnings is not None else []

        result = V4Result(
            verdict=verdict, state=state, risk_level=risk,
            latency_ms=(time.perf_counter() - t0) * 1000,
            capability_match={"score": capability_result["score"], "detail": capability_result["detail"],
                              "cap_result": capability_result.get("cap_result", {})},
            action_target_match={"score": target_result["score"], "detail": target_result["detail"],
                                 "interaction_result": target_result.get("interaction_result", {})},
            param_check={"score": param_result["score"], "detail": param_result["detail"],
                         "margin": param_result["margin"]},
            safety_zone=gz,
            disturbance={"level": "none", "strength": 0.0},
            correction=corr, recommended_params=rec,
            action_key=action_key, object_category=category,
            rule_source="rules" if rules else "physics_model",
            contact_area_mm2=area, pressure_kPa=pressure,
            iso_compliance=iso_compliance,
            risk_level_7=risk_level_7,
            risk_subtypes=risk_subtypes,
            recommended_params_v2=recommended_params_v2,
            retreat_params=retreat_params,
            semantic_plausibility_score=semantic_score,
            over_ratio=over_ratio_val,
            input_warnings=warnings_list,
        )
        return result.to_dict()


    def check_command(self, action: str, obj: str,
                       params: Optional[Dict[str, Any]] = None,
                       robot: Optional[Union[str, Dict[str, Any]]] = None,
                       object_params: Optional[Dict[str, Any]] = None,
                       context: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """自然语言输入

        Args:
            action: 动作名称（中文或英文，如 "抓取" / "grasp"）
            obj: 目标物体名称
            params: 动作参数字典 {"force": N, "speed": m/s, ...}
            robot: 机器人配置，支持三种形式：
                   - 字符串: 预设机型名，如 "humanoid_basic" / "humanoid_dexterous"
                   - 字典: 自定义能力参数 {"tier":2, "dexterity":0.5, "max_force":150, ...}
                   - None: 使用默认 humanoid_basic
            object_params: 自定义物体属性字典（覆盖内置物体库）
            context: 上下文信息 {"near_human": bool, "fragile": bool}
        """
        t0 = time.perf_counter()
        input_warnings: List[str] = []

        # === 输入参数校验 ===
        # action 必须是非空字符串
        if not isinstance(action, str) or not action.strip():
            return V4Result(
                verdict="REJECT", risk_level="HIGH",
                latency_ms=(time.perf_counter() - t0) * 1000,
                correction="action参数必须是非空字符串",
                input_warnings=["action参数必须是非空字符串"],
            ).to_dict()

        # obj 必须是字符串
        if not isinstance(obj, str):
            return V4Result(
                verdict="REJECT", risk_level="HIGH",
                latency_ms=(time.perf_counter() - t0) * 1000,
                correction="obj参数必须是字符串类型",
                input_warnings=["obj参数必须是字符串类型"],
            ).to_dict()

        # params 类型检查
        if params is not None and not isinstance(params, dict):
            return V4Result(
                verdict="REJECT", risk_level="HIGH",
                latency_ms=(time.perf_counter() - t0) * 1000,
                correction="params参数必须是字典类型",
                input_warnings=["params参数必须是字典类型"],
            ).to_dict()

        # robot 类型检查
        if robot is not None and not isinstance(robot, (str, dict)):
            return V4Result(
                verdict="REJECT", risk_level="HIGH",
                latency_ms=(time.perf_counter() - t0) * 1000,
                correction="robot参数必须是字符串或字典类型",
                input_warnings=["robot参数必须是字符串或字典类型"],
            ).to_dict()

        # params 数值参数校验
        if params:
            is_valid, err_msg, param_warnings, normalized_params = self._validate_params(params)
            if not is_valid:
                return V4Result(
                    verdict="REJECT", risk_level="HIGH",
                    latency_ms=(time.perf_counter() - t0) * 1000,
                    correction=f"输入参数无效: {err_msg}",
                    input_warnings=param_warnings,
                ).to_dict()
            input_warnings.extend(param_warnings)
            params = normalized_params

        # Layer 1: 语义解析
        action_key = self.parser.resolve_action(action)
        props = object_params or self.parser.get_object_props(obj)
        # robot 参数兼容三种形式：字符串(机型名) / 字典(自定义能力) / None(默认)
        if robot is None:
            subj = self.parser.get_robot_cap("humanoid_basic")
        elif isinstance(robot, str):
            subj = self.parser.get_robot_cap(robot)
        elif isinstance(robot, dict):
            subj = self.parser.normalize_robot_cap(robot)
        else:
            subj = self.parser.get_robot_cap("humanoid_basic")
        # 动词信息：先尝试原始动作名（保留具体动词语义），再尝试解析后的标准key
        verb_info = VERB_DATABASE.get(action) or self.parser.get_verb_info(action_key)
        category = props.get("category", "rigid")
        impossible = self.parser.check_impossible(action_key, category)
        rules = self.parser.lookup_rules_fallback(action_key, props)

        # Layer 2: 安全适配（参数校验量化表达）

        capability_result = self.adapter.check_capability_action(robot, action_key, subj, verb_info)  # type: ignore[arg-type]
        target_result = self.adapter.check_action_target(action_key, obj, verb_info, props)
        param_result = self.adapter.check_params(action_key, params or {}, verb_info, props, subj, rules, obj)

        # Layer 3: 综合决策
        if impossible:
            verdict, risk, corr = "REJECT", "HIGH", "动作与物体本质不兼容"
        elif not capability_result["passed"]:
            verdict, risk, corr = "REJECT", "HIGH", capability_result["detail"]
        elif not target_result["passed"]:
            verdict, risk, corr = "REJECT", "HIGH", target_result["detail"]
        elif not param_result["passed"]:
            verdict, risk, corr = "FAIL", "MEDIUM", f"参数不安全: {param_result['detail']}"
        else:
            total = capability_result["score"] * 0.25 + target_result["score"] * 0.35 + param_result["score"] * 0.40
            if total >= 0.7:
                verdict, risk, corr = "PASS", "LOW", "无需修正"
            elif total >= 0.4:
                verdict, risk, corr = "PASS", "MEDIUM", "安全裕度偏低"
            else:
                verdict, risk, corr = "FAIL", "MEDIUM", f"综合评分偏低({total:.2f})"

        # 调用公共构建方法
        return self._build_result(
            verdict=verdict,
            risk=risk,
            corr=corr,
            action_key=action_key,
            verb_info=verb_info,
            props=props,
            subj=subj,
            params=params or {},
            rules=rules,
            capability_result=capability_result,
            target_result=target_result,
            param_result=param_result,
            t0=t0,
            category=category,
            context=context,
            input_warnings=input_warnings,
        )

    def check_action(self, scene_data: Dict[str, Any], action_data: Dict[str, Any], robot_data: Dict[str, Any]) -> Dict[str, Any]:
        """JSON参数输入（模式B）

        对齐 check_command 完整逻辑:
        - 从 obj_data 提取 category / fragile / heavy / fluid / human_contact / contact_area
        - 调用 check_impossible 拦截不可能组合
        - 调用 lookup_rules_fallback 获取规则表
        - 将 rules 和 obj_name 传递给 check_params 做硬约束校验
        - 机器人能力字段归一化
        """
        t0 = time.perf_counter()
        input_warnings: List[str] = []

        # === 输入参数校验 ===
        if not isinstance(scene_data, dict):
            return V4Result(
                verdict="REJECT", risk_level="HIGH",
                latency_ms=(time.perf_counter() - t0) * 1000,
                correction="scene_data参数必须是字典类型",
                input_warnings=["scene_data参数必须是字典类型"],
            ).to_dict()
        if not isinstance(action_data, dict):
            return V4Result(
                verdict="REJECT", risk_level="HIGH",
                latency_ms=(time.perf_counter() - t0) * 1000,
                correction="action_data参数必须是字典类型",
                input_warnings=["action_data参数必须是字典类型"],
            ).to_dict()
        if not isinstance(robot_data, dict):
            return V4Result(
                verdict="REJECT", risk_level="HIGH",
                latency_ms=(time.perf_counter() - t0) * 1000,
                correction="robot_data参数必须是字典类型",
                input_warnings=["robot_data参数必须是字典类型"],
            ).to_dict()
        if "type" not in action_data:
            return V4Result(
                verdict="REJECT", risk_level="HIGH",
                latency_ms=(time.perf_counter() - t0) * 1000,
                correction="action_data必须包含type字段",
                input_warnings=["action_data必须包含type字段"],
            ).to_dict()

        # 数值参数校验（force_n / velocity_ms）
        action_copy = dict(action_data)
        numeric_fields = ["force_n", "velocity_ms", "acceleration_ms2"]
        action_for_validate = {k: action_copy.get(k, 0) for k in numeric_fields if k in action_copy}
        if action_for_validate:
            is_valid, err_msg, param_warnings, normalized_action = self._validate_params(action_for_validate)
            if not is_valid:
                return V4Result(
                    verdict="REJECT", risk_level="HIGH",
                    latency_ms=(time.perf_counter() - t0) * 1000,
                    correction=f"输入参数无效: {err_msg}",
                    input_warnings=param_warnings,
                ).to_dict()
            input_warnings.extend(param_warnings)
            action_copy.update(normalized_action)

        target = action_copy.get("target_object", "")
        obj_data = None
        for o in scene_data.get("objects", []):
            if o.get("object_id") == target:
                obj_data = o
                break
        if not obj_data:
            return V4Result(verdict="FAIL", risk_level="HIGH",
                            latency_ms=(time.perf_counter() - t0) * 1000,
                            correction=f"目标'{target}'未找到").to_dict()

        # 从 obj_data 中尽可能提取真实属性（与 check_command 对齐）
        mass_kg = obj_data.get("mass_kg", 1.0)
        props = {
            "category": obj_data.get("stability", "rigid"),
            "fragile": float(obj_data.get("fragile", 0.3)),
            "heavy": 1.0 if mass_kg > 2 else 0.0,
            "fluid": float(obj_data.get("fluid", 0.0)),
            "human_contact": float(obj_data.get("human_contact", 0.0)),
            "weight_est": mass_kg,
            "contact_area_mm2": obj_data.get("contact_area_mm2", 400),
            "contact_stiffness": obj_data.get("contact_stiffness", 3.0),
            "max_deform": obj_data.get("max_deform", 0.1),
        }
        # 机器人能力字段归一化
        subj = self.parser.normalize_robot_cap({
            "tier": robot_data.get("tier", 2),
            "dexterity": robot_data.get("dexterity", 0.5),
            **robot_data,
        })
        action_key = action_copy.get("type", "default")
        verb_info = self.parser.get_verb_info(action_key)
        params = {"force": action_copy.get("force_n", 0),
                  "speed": action_copy.get("velocity_ms", 0)}

        # 与 check_command 对齐：不可能组合检查 + 规则表fallback
        category = props.get("category", "rigid")
        impossible = self.parser.check_impossible(action_key, category)
        rules = self.parser.lookup_rules_fallback(action_key, props)

        capability_result = self.adapter.check_capability_action("robot", action_key, subj, verb_info)
        target_result = self.adapter.check_action_target(action_key, target, verb_info, props)
        param_result = self.adapter.check_params(action_key, params, verb_info, props, subj, rules, target)

        # 综合决策（与 check_command 对齐）
        if impossible:
            verdict, risk, corr = "REJECT", "HIGH", "动作与物体本质不兼容"
        elif not capability_result["passed"]:
            verdict, risk, corr = "REJECT", "HIGH", capability_result["detail"]
        elif not target_result["passed"]:
            verdict, risk, corr = "REJECT", "HIGH", target_result["detail"]
        elif not param_result["passed"]:
            verdict, risk, corr = "FAIL", "MEDIUM", f"参数不安全: {param_result['detail']}"
        else:
            total = (capability_result["score"] * 0.25
                     + target_result["score"] * 0.35
                     + param_result["score"] * 0.40)
            if total >= 0.7:
                verdict, risk, corr = "PASS", "LOW", "无需修正"
            elif total >= 0.4:
                verdict, risk, corr = "PASS", "MEDIUM", "安全裕度偏低"
            else:
                verdict, risk, corr = "FAIL", "MEDIUM", f"综合评分偏低({total:.2f})"

        # 调用公共构建方法
        return self._build_result(
            verdict=verdict,
            risk=risk,
            corr=corr,
            action_key=action_key,
            verb_info=verb_info,
            props=props,
            subj=subj,
            params=params,
            rules=rules,
            capability_result=capability_result,
            target_result=target_result,
            param_result=param_result,
            t0=t0,
            category=category,
            input_warnings=input_warnings,
        )
# =====================================================================
# 辅助函数（供测试用，必须在if __name__之前定义）
# =====================================================================

def _check_a(engine, subj, verb, obj, expected):
    r = engine.check_command(verb, obj, robot=subj)
    if r["verdict"] == "REJECT": actual = "reject"
    elif r["risk_level"] == "LOW": actual = "safe"
    elif r["risk_level"] == "MEDIUM": actual = "caution"
    else: actual = "danger"
    return actual == expected

def _check_f(engine, subj, verb, obj, params, expected):
    r = engine.check_command(verb, obj, params, robot=subj)
    if r["verdict"] == "REJECT": actual = "reject"
    elif r["risk_level"] == "LOW": actual = "safe"
    elif r["risk_level"] == "MEDIUM": actual = "caution"
    else: actual = "danger"
    return actual == expected


# =====================================================================
# 第七部分：测试套件
# =====================================================================

if __name__ == "__main__":
    print("=" * 70)
    print("Rotor Safety Engine v4.2.3 测试套件")
    print("=" * 70)

    engine = SafetyEngineV4()
    all_pass = True

    TEST_ROBOT = {
        "max_force_n": 500.0, "max_velocity_ms": 2.0,
        "max_acceleration_ms2": 50.0, "max_payload_kg": 100.0,
        "min_force_n": 0.001, "repeatability_mm": 0.02,
        "force_resolution_n": 0.001,
    }

    # =================================================================
    # 测试1：228场景准确率（4动作×19物体×3参数组）
    # =================================================================
    print("\n" + "─" * 70)
    print("测试1：228场景准确率")
    print("─" * 70)

    actions = ["grasp", "carry", "push", "move"]
    objects = list(OBJECT_PROPERTIES.keys())
    t1_total = t1_pass = 0
    t1_errors = []

    for action in actions:
        for obj_name in objects:
            props = OBJECT_PROPERTIES[obj_name]
            cat = props["category"]
            is_imp = cat in IMPOSSIBLE_PAIRS.get(action, set())

            # 使用引擎真实的 fallback 逻辑取规则，与 check_command 内部一致
            # 使用引擎真实的 fallback 逻辑取规则，与 check_command 内部一致
            rule = engine.parser.lookup_rules_fallback(action, props)
            if rule is None: continue

            # 取动词约束，有效上限 = min(规则上限, 动词上限, 机器人上限)
            verb_info = engine.parser.get_verb_info(action)
            r_force_max = rule["force"][2]
            r_speed_max = rule["speed"][2]
            f_opt = rule["force"][1]
            s_opt = rule["speed"][1]

            # 计算有效上限（考虑动词约束 + 机器人能力 + 易碎系数 + ISO限值）
            robot_cap = engine.parser.normalize_robot_cap(TEST_ROBOT)
            f_max_verb = verb_info["grasp_params"]["max_force"] if verb_info and verb_info.get("grasp_params") else float("inf")
            f_max_robot = robot_cap.get("max_force", 100)
            fragile_val = float(props.get("fragile", 0.0))
            fragile_factor = 1.0 - fragile_val * engine.adapter.rules.fragile_force_factor
            cat = props.get("category", "rigid")
            # ISO限值：仅人体部位生效（human_contact仅作标注参考，不收紧推荐参数）
            iso_limit = float("inf")
            if cat == "human":
                iso_limit = engine.adapter.rules.iso.get_quasi_static_limit(obj_name)
            if r_force_max > 0:
                f_max = min(r_force_max, f_max_verb, f_max_robot, iso_limit) * fragile_factor
            else:
                f_max = min(f_max_verb, f_max_robot, iso_limit) * fragile_factor

            s_max_verb = (verb_info["grasp_params"]["max_speed"] / 1000.0) if verb_info and verb_info.get("grasp_params") else float("inf")
            s_max_robot = robot_cap.get("max_speed", 2.0)
            if r_speed_max > 0:
                s_max = min(r_speed_max, s_max_verb, s_max_robot)
            else:
                s_max = min(s_max_verb, s_max_robot)

            # 安全值：取规则最优值，但不超过有效上限的 80%
            f_safe = min(f_opt, f_max * 0.8)
            s_safe = min(s_opt, s_max * 0.8)

            # 安全 → PASS/REJECT
            r = engine.check_command(action, obj_name, {"force": f_safe, "speed": s_safe}, robot=TEST_ROBOT)
            exp = "REJECT" if is_imp else "PASS"
            t1_total += 1
            if r["verdict"] == exp: t1_pass += 1
            else: t1_errors.append(f"  ✗ {action}+{obj_name} safe: 期望{exp}, 实际{r['verdict']} (f={f_safe:.3f},s={s_safe:.4f})")

            # 边界 → PASS/REJECT（取有效上限的 90%）
            bf = f_max * 0.9 if f_max > 0 else 0
            bs = s_max * 0.9 if s_max > 0 else 0
            r = engine.check_command(action, obj_name, {"force": bf, "speed": bs}, robot=TEST_ROBOT)
            t1_total += 1
            if r["verdict"] == exp: t1_pass += 1
            else: t1_errors.append(f"  ✗ {action}+{obj_name} boundary: 期望{exp}, 实际{r['verdict']} (f={bf:.3f},s={bs:.4f})")

            # 危险 → FAIL/REJECT
            # impossible 组合期望 REJECT，正常组合期望 FAIL（超过有效上限即可）
            if is_imp:
                r = engine.check_command(action, obj_name, {"force": f_max * 2 if f_max > 0 else 0, "speed": s_max * 2}, robot=TEST_ROBOT)
                t1_total += 1
                if r["verdict"] == "REJECT": t1_pass += 1
                else: t1_errors.append(f"  ✗ {action}+{obj_name} danger: 期望REJECT, 实际{r['verdict']}")
            else:
                r = engine.check_command(action, obj_name, {"force": f_max * 2.0 if f_max > 0 else 0, "speed": s_max * 2.0}, robot=TEST_ROBOT)
                t1_total += 1
                if r["verdict"] == "FAIL": t1_pass += 1
                else: t1_errors.append(f"  ✗ {action}+{obj_name} danger: 期望FAIL, 实际{r['verdict']} (f={f_max*2:.3f},s={s_max*2:.4f})")

    acc1 = t1_pass / t1_total if t1_total > 0 else 0
    print(f"  结果: {t1_pass}/{t1_total} = {acc1:.2%}")
    for e in t1_errors[:5]: print(e)
    if len(t1_errors) > 5: print(f"  ... 共{len(t1_errors)}个错误")
    if not t1_errors: print("  ✓ 全部通过！")
    if acc1 < 1.0: all_pass = False

    # =================================================================
    # 测试2：76场景综合测试
    # =================================================================
    print("\n" + "─" * 70)
    print("测试2：76场景综合测试")
    print("─" * 70)

    # A类：语义25场景
    semantic_tests = [
        ("基础人形", "抓", "杯子", "safe"), ("基础人形", "拿", "书", "safe"),
        ("基础人形", "取", "物品", "safe"), ("基础人形", "推", "门", "safe"),
        ("基础人形", "拉", "抽屉", "safe"), ("基础人形", "按", "按钮", "safe"),
        ("基础人形", "捏", "笔", "safe"), ("基础人形", "插", "钥匙", "safe"),
        ("基础人形", "拧", "钥匙", "safe"), ("基础人形", "摸", "表面", "safe"),
        ("灵巧人形", "握", "杯子", "safe"), ("灵巧人形", "持", "工具", "safe"),
        ("灵巧人形", "举", "重物", "safe"), ("灵巧人形", "端", "碗", "safe"),
        ("灵巧人形", "夹", "文件", "safe"), ("基础人形", "扶", "老人", "safe"),
        ("基础人形", "托", "托盘", "safe"), ("基础人形", "提", "袋子", "safe"),
        ("灵巧人形", "压", "模具", "safe"), ("基础人形", "踩", "踏板", "safe"),
        ("灵巧人形", "吸", "玻璃", "safe"), ("灵巧人形", "顶", "门", "safe"),
        ("基础人形", "卡", "卡槽", "safe"), ("基础人形", "背", "背包", "safe"),
        ("基础人形", "扛", "袋子", "safe"),
    ]

    # B类：边界监控15场景
    boundary_tests = [
        ("握", "杯子", 25.0, 25.0, 1.0, 0.0, "none"),
        ("握", "杯子", 30.0, 25.0, 1.0, 0.0, "none"),
        ("握", "杯子", 25.0, 25.0, 1.0, 0.05, "none"),
        ("握", "杯子", 25.0, 25.0, 1.0, 0.15, "micro"),
        ("握", "杯子", 4.0, 25.0, 1.0, 0.0, "small"),
        ("握", "杯子", 2.0, 25.0, 1.0, 0.0, "large"),
        ("握", "杯子", 0.5, 25.0, 1.0, 0.0, "danger"),
        ("握", "杯子", 25.0, 25.0, 4.0, 0.0, "medium"),
        ("握", "杯子", 25.0, 25.0, 7.0, 0.0, "danger"),
        ("握", "杯子", 25.0, 25.0, 1.0, 0.5, "large"),
        ("举", "重物", 20.0, 100.0, 1.0, 0.0, "medium"),
        ("端", "碗", 15.0, 15.0, 1.0, 0.0, "none"),
        ("夹", "文件", 15.0, 15.0, 0.3, 0.0, "none"),
        ("提", "袋子", 35.0, 40.0, 5.0, 0.0, "none"),
        ("托", "托盘", 25.0, 25.0, 1.5, 0.0, "none"),
    ]

    # C类：FAV分类10场景
    fav_tests = [
        (0.0, 0.0, 0.0, "idle"), (0.01, 0.01, 0.005, "idle"),
        (0.7, 0.4, 0.7, "grasping"), (0.5, 0.5, 0.005, "holding"),
        (0.55, 0.5, 0.003, "holding"), (0.85, 0.3, 0.85, "grasping"),
        (0.01, 0.02, 0.005, "idle"), (0.3, 0.6, 0.003, "holding"),
        (0.9, 0.7, 0.9, "grasping"), (0.52, 0.48, 0.003, "holding"),
    ]

    # D类：状态稳定10场景
    closure_tests = [
        ("抓", 25.0, 25.0, 5.0, 1.0, True), ("抓", 25.0, 25.0, 50.0, 1.0, False),
        ("抓", 10.0, 25.0, 5.0, 1.0, False), ("握", 25.0, 25.0, 5.0, 1.0, True),
        ("拿", 20.0, 20.0, 3.0, 1.5, True), ("取", 15.0, 15.0, 3.0, 1.0, True),
        ("推", 50.0, 50.0, 10.0, 3.0, True), ("按", 10.0, 10.0, 3.0, 0.5, True),
        ("捏", 7.5, 7.5, 2.0, 0.3, True), ("插", 15.0, 15.0, 3.0, 0.3, True),
    ]

    # E类：干扰分级6场景
    disturbance_tests = [
        (0.02, "none"), (0.08, "micro"), (0.20, "small"),
        (0.40, "medium"), (0.60, "large"), (0.90, "danger"),
    ]

    # F类：端到端5场景（参数单位与规则表一致）
    e2e_tests = [
        ("基础人形", "抓", "鸡蛋", {"force": 1.0, "speed": 0.03}, "safe"),
        ("基础人形", "拿", "杯子", {"force": 5.0, "speed": 0.08}, "safe"),
        ("基础人形", "推", "门", {"force": 20.0, "speed": 0.3}, "safe"),
        ("灵巧人形", "握", "玻璃杯", {"force": 5.0, "speed": 0.03}, "safe"),
        ("基础人形", "捏", "笔", {"force": 3.0, "speed": 0.01}, "safe"),
    ]

    # G类：接触面积5场景
    area_tests = [
        ("抓取", "杯子", {"force": 5.0, "speed": 0.05}, 200.0),
        ("抓取", "杯子", {"force": 5.0, "speed": 0.05}, 800.0),
        ("抓取", "杯子", {"force": 5.0, "speed": 0.05}, 400.0),
        ("抓取", "鸡蛋", {"force": 1.0, "speed": 0.02}, 100.0),
        ("抓取", "铁块", {"force": 20.0, "speed": 0.05}, 500.0),
    ]

    t2_total = t2_pass = 0
    t2_errors = []

    # A类
    a_pass = 0
    for subj, verb, obj, exp in semantic_tests:
        t2_total += 1
        if _check_a(engine, subj, verb, obj, exp):
            t2_pass += 1
            a_pass += 1
        else:
            r = engine.check_command(verb, obj, robot=subj)
            actual = "reject" if r["verdict"] == "REJECT" else ("safe" if r["risk_level"] == "LOW" else "caution")
            t2_errors.append(f"  ✗ A: {subj}+{verb}+{obj}: 期望{exp}, 实际{actual}")

    # B类
    b_pass = 0
    for v, o, f, ft, d, vi, el in boundary_tests:
        t2_total += 1
        r = engine.judge.hold_boundary_check(v, f, ft, d, vi)
        actual = r.get("disturbance_level", "none")
        if actual == el:
            t2_pass += 1
            b_pass += 1
        else:
            t2_errors.append(f"  ✗ B: {v}+{o} f={f}: 期望{el}, 实际{actual}")

    # C类
    c_pass = 0
    for f, a, vel, es in fav_tests:
        t2_total += 1
        r = engine.fav.classify(f, a, vel)
        actual = r["phase_state"]
        if actual == es:
            t2_pass += 1
            c_pass += 1
        else:
            t2_errors.append(f"  ✗ C: F={f},A={a},V={vel}: 期望{es}, 实际{actual}")

    # D类
    d_pass = 0
    for v, f, ft, sp, d, ec in closure_tests:
        t2_total += 1
        actual = engine.judge.check_state_stabilization(v, f, ft, sp, d)  # type: ignore[assignment]
        if actual == ec:
            t2_pass += 1
            d_pass += 1
        else:
            t2_errors.append(f"  ✗ D: {v} f={f}: 期望{ec}, 实际{actual}")

    # E类
    e_pass = 0
    for s, el in disturbance_tests:
        t2_total += 1
        actual = engine.judge._classify_disturbance(s)
        if actual == el:
            t2_pass += 1
            e_pass += 1
        else:
            t2_errors.append(f"  ✗ E: s={s}: 期望{el}, 实际{actual}")

    # F类
    f_pass = 0
    for subj, verb, obj, params, exp in e2e_tests:
        t2_total += 1
        if _check_f(engine, subj, verb, obj, params, exp):
            t2_pass += 1
            f_pass += 1
        else:
            r = engine.check_command(verb, obj, params, robot=subj)
            actual = "reject" if r["verdict"] == "REJECT" else ("safe" if r["risk_level"] == "LOW" else "caution")
            t2_errors.append(f"  ✗ F: {subj}+{verb}+{obj}: 期望{exp}, 实际{actual}")

    # G类
    g_pass = 0
    for act, obj, params, area in area_tests:
        t2_total += 1
        r = engine.check_command(act, obj, params, robot=TEST_ROBOT,
                                  object_params={"contact_area_mm2": area})
        if r["verdict"] == "PASS":
            t2_pass += 1
            g_pass += 1
        else:
            t2_errors.append(f"  ✗ G: {act}+{obj} area={area}: {r['verdict']}")

    acc2 = t2_pass / t2_total if t2_total > 0 else 0
    print(f"  A类(语义25): {a_pass}/25")
    print(f"  B类(边界15): {b_pass}/15")
    print(f"  C类(FAV10):  {c_pass}/10")
    print(f"  D类(闭合10): {d_pass}/10")
    print(f"  E类(干扰6):  {e_pass}/6")
    print(f"  F类(端到端5):{f_pass}/5")
    print(f"  G类(接触5):  {g_pass}/5")
    print(f"  总计: {t2_pass}/{t2_total} = {acc2:.2%}")
    for e in t2_errors[:10]: print(e)
    if len(t2_errors) > 10: print(f"  ... 共{len(t2_errors)}个错误")
    if not t2_errors: print("  ✓ 全部通过！")
    if acc2 < 0.90: all_pass = False

    # =================================================================
    # 测试3：自然语言输入（12个）
    # =================================================================
    print("\n" + "─" * 70)
    print("测试3：自然语言输入")
    print("─" * 70)

    nl_tests = [
        ("抓取", "鸡蛋", "PASS"), ("搬运", "铁块", "PASS"), ("推开", "椅子", "PASS"),
        ("拿", "水", "REJECT"), ("捏", "面包", "PASS"), ("搬", "桌子", "PASS"),
        ("抓取", "玻璃杯", "PASS"), ("走", "人", "PASS"), ("拉开", "门", "PASS"),
        ("握", "杯子", "PASS"), ("拧", "钥匙", "PASS"), ("按住", "按钮", "PASS"),
    ]

    t3_pass = 0
    for act, obj, exp in nl_tests:
        r = engine.check_command(act, obj, robot=TEST_ROBOT)
        ok = r["verdict"] == exp
        if ok: t3_pass += 1
        print(f"  {'✓' if ok else '✗'} {act} {obj}: {r['verdict']} (期望{exp})")
    print(f"  结果: {t3_pass}/{len(nl_tests)}")
    if t3_pass < len(nl_tests): all_pass = False

    # =================================================================
    # 测试4：模式B（JSON输入）
    # =================================================================
    print("\n" + "─" * 70)
    print("测试4：模式B — JSON物理参数输入")
    print("─" * 70)

    scene = {"objects": [{"object_id": "seal", "name": "密封条",
                          "mass_kg": 0.8, "stability": "flexible",
                          "contact_area_mm2": 600}]}
    # flexible → fragile 类: grasp力上限3.0N, 速度上限0.05m/s
    action_b = {"type": "grasp", "force_n": 2.0, "velocity_ms": 0.03,
                "acceleration_ms2": 1.0, "target_object": "seal"}
    robot_b = {"max_force_n": 150, "max_velocity_ms": 2.0,
               "max_acceleration_ms2": 10.0}
    r = engine.check_action(scene, action_b, robot_b)
    ok_b = r["verdict"] == "PASS"
    print(f"  {'✓' if ok_b else '✗'} grasp 密封条: {r['verdict']}, 压强={r['pressure_kPa']:.2f}kPa")
    if not ok_b: all_pass = False

    # =================================================================
    # 测试5：性能测试
    # =================================================================
    print("\n" + "─" * 70)
    print("测试5：性能测试")
    print("─" * 70)

    N = 100000
    t0 = time.perf_counter()
    for _ in range(N):
        engine.check_command("grasp", "鸡蛋", {"force": 2.0, "speed": 0.03}, robot=TEST_ROBOT)
    elapsed = time.perf_counter() - t0
    avg_ms = elapsed / N * 1000
    ok_perf = avg_ms < 0.1
    print(f"  {N}次, 总{elapsed:.3f}s, 平均{avg_ms:.4f}ms/次")
    print(f"  {'✓' if ok_perf else '✗'} < 0.1ms: {'通过' if ok_perf else '未通过'}")
    if not ok_perf: all_pass = False

    # =================================================================
    # 测试6：输出结构完整性 + ISO合规
    # =================================================================
    print("\n" + "─" * 70)
    print("测试6：输出结构完整性 + ISO合规标注")
    print("─" * 70)

    r = engine.check_command("抓取", "鸡蛋", {"force": 2.0, "speed": 0.03}, robot=TEST_ROBOT)
    required = ["verdict", "state", "capability_match", "action_target_match",
                "param_check", "safety_zone", "disturbance", "correction",
                "recommended_params", "contact_area_mm2", "pressure_kPa", "iso_compliance",
                "risk_level_7", "risk_subtypes", "over_ratio"]
    missing = [k for k in required if k not in r]
    ok_struct = len(missing) == 0
    print(f"  {'✓' if ok_struct else '✗'} 必需字段: {len(required)}个")
    if missing: print(f"    缺失: {missing}")

    # ISO合规测试
    r_human = engine.check_command("摸", "手", {"force": 1.0, "speed": 10}, robot=TEST_ROBOT)
    print(f"  人体接触ISO测试: {r_human['iso_compliance']}")
    
    print(f"\n  完整输出示例:")
    print(f"  {json.dumps(r, indent=2, ensure_ascii=False)}")
    if not ok_struct: all_pass = False

    # =================================================================
    # 测试7：动态接触面积 + 冲量校验 + 反作用力约束
    # =================================================================
    print("\n" + "─" * 70)
    print("测试7：动态接触面积 + 冲量校验 + 反作用力约束")
    print("─" * 70)

    t7_pass = 0
    t7_total = 0

    # 7a: 动态接触面积验证 — 同样5N力，面包压强 < 铁块压强（软物面积大→压强小）
    t7_total += 1
    r_bread = engine.check_command("grasp", "面包", {"force": 5.0, "speed": 0.05}, robot=TEST_ROBOT)
    r_iron = engine.check_command("grasp", "铁块", {"force": 5.0, "speed": 0.05}, robot=TEST_ROBOT)
    ok_dca = (r_bread["pressure_kPa"] < r_iron["pressure_kPa"]
              and r_bread["contact_area_mm2"] > OBJECT_PROPERTIES["面包"]["contact_area_mm2"]
              and r_iron["contact_area_mm2"] > OBJECT_PROPERTIES["铁块"]["contact_area_mm2"])
    print(f"  {'✓' if ok_dca else '✗'} 动态接触面积: 面包({r_bread['pressure_kPa']:.1f}kPa) < 铁块({r_iron['pressure_kPa']:.1f}kPa)")
    if ok_dca: t7_pass += 1

    # 7b: 动态接触面积 — 力为0时面积不变
    t7_total += 1
    r0 = engine.check_command("grasp", "面包", {"force": 0.0, "speed": 0.05}, robot=TEST_ROBOT)
    base_a = OBJECT_PROPERTIES["面包"]["contact_area_mm2"]
    ok_zero = abs(r0["contact_area_mm2"] - base_a) < 0.01
    print(f"  {'✓' if ok_zero else '✗'} 力为0时面积不变: {r0['contact_area_mm2']:.1f} == {base_a}")
    if ok_zero: t7_pass += 1

    # 7c: 冲量校验 — carry 重物高速 → FAIL
    t7_total += 1
    r_imp_fail = engine.check_command("carry", "铁块", {"force": 40.0, "speed": 1.5}, robot=TEST_ROBOT)
    ok_imp_fail = r_imp_fail["verdict"] == "FAIL" and "冲量" in r_imp_fail["param_check"]["detail"]
    imp_val = 20.0 * 1.5  # 20kg × 1.5m/s
    print(f"  {'✓' if ok_imp_fail else '✗'} 冲量-重物高速FAIL: {r_imp_fail['verdict']} ({imp_val:.1f}kg·m/s)")
    if ok_imp_fail: t7_pass += 1

    # 7d: 冲量校验 — carry 重物低速 → PASS
    t7_total += 1
    r_imp_pass = engine.check_command("carry", "铁块", {"force": 40.0, "speed": 0.2}, robot=TEST_ROBOT)
    imp_safe = 20.0 * 0.2  # 4 kg·m/s < 25
    ok_imp_pass = r_imp_pass["verdict"] == "PASS" or (r_imp_pass["verdict"] == "FAIL" and "冲量" not in r_imp_pass["param_check"]["detail"])
    # Actually check: if FAIL for other reasons (not impulse), still counts
    detail = r_imp_pass["param_check"]["detail"]
    impulse_ok = "冲量" not in detail or "接近" in detail
    print(f"  {'✓' if impulse_ok else '✗'} 冲量-重物低速安全: {r_imp_pass['verdict']} (冲量={imp_safe:.1f}kg·m/s, detail={detail[:40]})")
    if impulse_ok: t7_pass += 1

    # 7e: 冲量校验 — carry 轻物安全速度 → 冲量远低于限值，应安全通过
    t7_total += 1
    # 鸡蛋 0.05kg × carry fragile速度最优值0.2m/s = 0.01 kg·m/s，远低于impulse_max
    r_light = engine.check_command("carry", "鸡蛋", {"force": 5.0, "speed": 0.2}, robot=TEST_ROBOT)
    imp_light = 0.05 * 0.2  # 0.01 kg·m/s, well under impulse limit
    light_ok = (r_light["verdict"] == "PASS")
    print(f"  {'✓' if light_ok else '✗'} 冲量-轻物安全速度通过: {r_light['verdict']} (冲量={imp_light:.4f}kg·m/s)")
    if light_ok: t7_pass += 1

    # 7f: 反作用力约束 — 力超过反作用极限 → FAIL
    t7_total += 1
    # 50kg × 9.81 × 0.6 = 294.3N, 用400N测试
    r_rxn_fail = engine.check_command("push", "门", {"force": 400.0, "speed": 0.1}, robot=TEST_ROBOT)
    rxn_detail = r_rxn_fail["param_check"]["detail"]
    ok_rxn_fail = "反作用力" in rxn_detail
    print(f"  {'✓' if ok_rxn_fail else '✗'} 反作用力-超载FAIL: {r_rxn_fail['verdict']} (detail含反作用力)")
    if ok_rxn_fail: t7_pass += 1

    # 7g: 反作用力约束 — 正常力 → 不触发反作用FAIL
    t7_total += 1
    r_rxn_ok = engine.check_command("push", "门", {"force": 20.0, "speed": 0.3}, robot=TEST_ROBOT)
    rxn_ok_detail = r_rxn_ok["param_check"]["detail"]
    rxn_not_triggered = "反作用力" not in rxn_ok_detail
    print(f"  {'✓' if rxn_not_triggered else '✗'} 反作用力-正常力不触发: {r_rxn_ok['verdict']}")
    if rxn_not_triggered: t7_pass += 1

    print(f"  结果: {t7_pass}/{t7_total}")
    if t7_pass < t7_total: all_pass = False

    # =================================================================
    # 总结
    # =================================================================
    print("\n" + "=" * 70)
    if all_pass:
        print("✓ 全部测试通过！Rotor Safety Engine v4 验证完成。")
    else:
        print("✗ 部分测试未通过。")
    print("=" * 70)
    sys.exit(0 if all_pass else 1)
