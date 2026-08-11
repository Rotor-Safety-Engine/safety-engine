"""
Rotor Safety Engine - Demo 演示脚本

演示 10 个典型场景（5 PASS + 5 FAIL），展示引擎判断结果与性能统计。
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from safety_engine import SafetyEngineV4


# =============================================================================
# 10 个测试场景
# =============================================================================

SCENARIOS = [
    # --- PASS 场景 ---
    {
        "name": "场景 1：机器人抓取金属工件（正常操作）",
        "scene": {
            "objects": [{
                "object_id": "metal_part",
                "name": "铝合金工件",
                "mass_kg": 2.5,
                "stability": "rigid",
                "contact_area_mm2": 500,
            }]
        },
        "action": {
            "type": "grasp",
            "force_n": 12.0,
            "velocity_ms": 0.15,
            "acceleration_ms2": 3.0,
            "target_object": "metal_part",
        },
        "robot": {
            "max_force_n": 150, "max_velocity_ms": 2.0,
            "max_acceleration_ms2": 10.0,
        },
    },
    {
        "name": "场景 2：机器人轻放塑料件到工作台",
        "scene": {
            "objects": [{
                "object_id": "plastic_cover",
                "name": "塑料外壳",
                "mass_kg": 0.3,
                "stability": "semi_rigid",
                "contact_area_mm2": 800,
            }]
        },
        "action": {
            "type": "place",
            "force_n": 2.0,
            "velocity_ms": 0.1,
            "acceleration_ms2": 0.5,
            "target_object": "plastic_cover",
        },
        "robot": {
            "max_force_n": 50, "max_velocity_ms": 1.5,
            "max_acceleration_ms2": 8.0,
        },
    },
    {
        "name": "场景 3：机器人推动纸箱（力足够）",
        "scene": {
            "objects": [{
                "object_id": "box_01",
                "name": "纸箱",
                "mass_kg": 5.0,
                "stability": "semi_rigid",
                "contact_area_mm2": 1200,
            }]
        },
        "action": {
            "type": "push",
            "force_n": 30.0,
            "velocity_ms": 0.2,
            "acceleration_ms2": 1.0,
            "target_object": "box_01",
        },
        "robot": {
            "max_force_n": 200, "max_velocity_ms": 2.0,
            "max_acceleration_ms2": 12.0,
        },
    },
    {
        "name": "场景 4：机器人旋转控制旋钮",
        "scene": {
            "objects": [{
                "object_id": "knob_01",
                "name": "控制旋钮",
                "mass_kg": 0.05,
                "stability": "rigid",
                "contact_area_mm2": 100,
            }]
        },
        "action": {
            "type": "rotate",
            "force_n": 0.5,
            "velocity_ms": 0.02,
            "acceleration_ms2": 0.1,
            "target_object": "knob_01",
        },
        "robot": {
            "max_force_n": 100, "max_velocity_ms": 1.0,
            "max_acceleration_ms2": 5.0,
        },
    },
    {
        "name": "场景 5：机器人抬起中等重量零件",
        "scene": {
            "objects": [{
                "object_id": "gear_assembly",
                "name": "齿轮组件",
                "mass_kg": 3.0,
                "stability": "rigid",
                "contact_area_mm2": 400,
            }]
        },
        "action": {
            "type": "lift",
            "force_n": 35.0,
            "velocity_ms": 0.2,
            "acceleration_ms2": 2.0,
            "target_object": "gear_assembly",
        },
        "robot": {
            "max_force_n": 200, "max_velocity_ms": 3.0,
            "max_acceleration_ms2": 15.0,
        },
    },

    # --- FAIL 场景 ---
    {
        "name": "场景 6：机器人大力抓取玻璃面板（力过大，易碎）",
        "scene": {
            "objects": [{
                "object_id": "glass_panel",
                "name": "玻璃面板",
                "mass_kg": 1.5,
                "stability": "fragile",
                "contact_area_mm2": 600,
            }]
        },
        "action": {
            "type": "grasp",
            "force_n": 80.0,
            "velocity_ms": 0.5,
            "acceleration_ms2": 8.0,
            "target_object": "glass_panel",
        },
        "robot": {
            "max_force_n": 150, "max_velocity_ms": 2.0,
            "max_acceleration_ms2": 10.0,
        },
    },
    {
        "name": "场景 7：机器人高速插拔密封条（速度超限，动词约束）",
        "scene": {
            "objects": [{
                "object_id": "seal_strip",
                "name": "车门密封条",
                "mass_kg": 0.8,
                "stability": "flexible",
                "contact_area_mm2": 300,
            }]
        },
        "action": {
            "type": "insert",
            "force_n": 10.0,
            "velocity_ms": 1.0,
            "acceleration_ms2": 2.0,
            "target_object": "seal_strip",
        },
        "robot": {
            "max_force_n": 150, "max_velocity_ms": 5.0,
            "max_acceleration_ms2": 10.0,
        },
    },
    {
        "name": "场景 8：小型机器人尝试举重物（能力不足）",
        "scene": {
            "objects": [{
                "object_id": "engine_block",
                "name": "发动机缸体",
                "mass_kg": 45.0,
                "stability": "heavy",
                "contact_area_mm2": 2000,
            }]
        },
        "action": {
            "type": "lift",
            "force_n": 200.0,
            "velocity_ms": 0.1,
            "acceleration_ms2": 0.5,
            "target_object": "engine_block",
        },
        "robot": {
            "max_force_n": 100, "max_velocity_ms": 1.0,
            "max_acceleration_ms2": 5.0,
        },
    },
    {
        "name": "场景 9：机器人力过大操作易碎橡胶件（力超限）",
        "scene": {
            "objects": [{
                "object_id": "rubber_gasket",
                "name": "橡胶垫片",
                "mass_kg": 0.1,
                "stability": "flexible",
                "contact_area_mm2": 200,
            }]
        },
        "action": {
            "type": "press",
            "force_n": 50.0,
            "velocity_ms": 0.05,
            "acceleration_ms2": 1.0,
            "target_object": "rubber_gasket",
        },
        "robot": {
            "max_force_n": 200, "max_velocity_ms": 3.0,
            "max_acceleration_ms2": 30.0,
        },
    },
    {
        "name": "场景 10：机器人力 / 速度均超限（三重失败）",
        "scene": {
            "objects": [{
                "object_id": "pcb_board",
                "name": "电路板",
                "mass_kg": 0.2,
                "stability": "fragile",
                "contact_area_mm2": 150,
            }]
        },
        "action": {
            "type": "grasp",
            "force_n": 200.0,
            "velocity_ms": 5.0,
            "acceleration_ms2": 30.0,
            "target_object": "pcb_board",
        },
        "robot": {
            "max_force_n": 100, "max_velocity_ms": 2.0,
            "max_acceleration_ms2": 10.0,
        },
    },
]


def main():
    print("=" * 70)
    print("  Rotor Safety Engine v4 - Demo 演示")
    print("=" * 70)
    print()

    engine = SafetyEngineV4()
    pass_count = 0
    fail_count = 0
    latencies = []

    # 预热：避免首次调用初始化开销导致延迟偏高
    warmup_scene = SCENARIOS[0]["scene"]
    warmup_action = SCENARIOS[0]["action"]
    warmup_robot = SCENARIOS[0]["robot"]
    for _ in range(100):
        engine.check_action(warmup_scene, warmup_action, warmup_robot)

    for i, scenario in enumerate(SCENARIOS):
        print(f"\n{'─' * 60}")
        print(f"  {scenario['name']}")
        print(f"{'─' * 60}")

        result = engine.check_action(
            scenario["scene"],
            scenario["action"],
            scenario["robot"],
        )

        latencies.append(result["latency_ms"])

        verdict_icon = "✅ PASS" if result["verdict"] == "PASS" else "❌ FAIL"
        print(f"  判定: {verdict_icon}  |  风险: {result['risk_level']}  |  延迟: {result['latency_ms']:.3f}ms")
        print(f"  接触压强: {result.get('pressure_kPa', 0):.1f} kPa")
        print(f"  能力适配: {result['capability_match']['score']:.2f}")
        print(f"  动作目标: {result['action_target_match']['score']:.2f}")
        print(f"  参数安全: {result['param_check']['score']:.2f}")

        if result["correction"] and result["correction"] != "无需修正":
            print(f"  💡 修正建议: {result['correction']}")

        if result["verdict"] == "PASS":
            pass_count += 1
        else:
            fail_count += 1

    # 性能统计
    print(f"\n\n{'=' * 70}")
    print("  性能统计")
    print(f"{'=' * 70}")
    print(f"  总场景数: {len(SCENARIOS)}")
    print(f"  PASS: {pass_count}  |  FAIL: {fail_count}")
    print(f"  平均延迟: {sum(latencies) / len(latencies):.3f}ms")
    print(f"  最大延迟: {max(latencies):.3f}ms")
    print(f"  最小延迟: {min(latencies):.3f}ms")
    throughput = len(latencies) / (sum(latencies) / 1000)
    print(f"  估算吞吐量: {throughput:.0f} 次/秒")
    print()


if __name__ == "__main__":
    main()
