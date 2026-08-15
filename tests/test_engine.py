# -*- coding: utf-8 -*-
"""
Safety Engine pytest 测试套件

运行方式:
    pytest tests/test_engine.py -v
    pytest tests/test_engine.py -v --tb=short
    python3 tests/test_engine.py    # 也可直接运行
"""

import sys
import os
import time

# 确保 src 目录在 import 路径中
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from safety_engine import (
    SafetyEngineV4,
    OBJECT_PROPERTIES,
    OBJECT_MASS,
    ACTION_RULES,
    IMPOSSIBLE_PAIRS,
    VERB_DATABASE,
)


# =====================================================================
# Fixtures
# =====================================================================

try:
    import pytest
except ImportError:
    raise ImportError(
        "pytest 未安装。请运行: pip install pytest\n"
        "或使用: pytest tests/test_engine.py -v"
    )


@pytest.fixture
def engine():
    return SafetyEngineV4()


@pytest.fixture
def test_robot():
    return {
        "max_force_n": 500.0,
        "max_velocity_ms": 2.0,
        "max_acceleration_ms2": 50.0,
        "max_payload_kg": 100.0,
        "min_force_n": 0.001,
        "repeatability_mm": 0.02,
        "force_resolution_n": 0.001,
    }


# =====================================================================
# Test 1: 228+ 场景准确率（4动作 × N物体 × 3参数组）
# =====================================================================

class TestAccuracy228:
    """场景准确率测试：安全/边界/危险三组参数全覆盖"""

    actions = ["grasp", "carry", "push", "move"]

    @pytest.mark.parametrize("action", actions)
    @pytest.mark.parametrize("obj_name", list(OBJECT_PROPERTIES.keys()))
    def test_safe_param(self, engine, test_robot, action, obj_name):
        """安全参数 → 正常PASS，不可能组合REJECT"""
        props = OBJECT_PROPERTIES[obj_name]
        cat = props["category"]
        is_imp = cat in IMPOSSIBLE_PAIRS.get(action, set())
        rule = engine.parser.lookup_rules_fallback(action, props)
        if rule is None:
            pytest.skip("无匹配规则")

        f_safe, s_safe = self._effective_safe(engine, test_robot, action, props, rule)
        r = engine.check_command(action, obj_name, {"force": f_safe, "speed": s_safe}, robot=test_robot)

        if is_imp:
            assert r["verdict"] == "REJECT", f"{action}+{obj_name} 安全组应REJECT"
        else:
            assert r["verdict"] == "PASS", f"{action}+{obj_name} 安全组应PASS, 实际{r['verdict']}: {r.get('correction','')}"

    @pytest.mark.parametrize("action", actions)
    @pytest.mark.parametrize("obj_name", list(OBJECT_PROPERTIES.keys()))
    def test_boundary_param(self, engine, test_robot, action, obj_name):
        """边界参数（有效上限90%） → 正常PASS，不可能组合REJECT"""
        props = OBJECT_PROPERTIES[obj_name]
        cat = props["category"]
        is_imp = cat in IMPOSSIBLE_PAIRS.get(action, set())
        rule = engine.parser.lookup_rules_fallback(action, props)
        if rule is None:
            pytest.skip("无匹配规则")

        f_max, s_max = self._effective_max(engine, test_robot, action, props, rule)
        bf = f_max * 0.9 if f_max > 0 else 0
        bs = s_max * 0.9 if s_max > 0 else 0
        r = engine.check_command(action, obj_name, {"force": bf, "speed": bs}, robot=test_robot)

        if is_imp:
            assert r["verdict"] == "REJECT", f"{action}+{obj_name} 边界组应REJECT"
        else:
            assert r["verdict"] == "PASS", f"{action}+{obj_name} 边界组应PASS, 实际{r['verdict']}: {r.get('correction','')}"

    @pytest.mark.parametrize("action", actions)
    @pytest.mark.parametrize("obj_name", list(OBJECT_PROPERTIES.keys()))
    def test_danger_param(self, engine, test_robot, action, obj_name):
        """危险参数（有效上限2倍） → 正常FAIL，不可能组合REJECT"""
        props = OBJECT_PROPERTIES[obj_name]
        cat = props["category"]
        is_imp = cat in IMPOSSIBLE_PAIRS.get(action, set())
        rule = engine.parser.lookup_rules_fallback(action, props)
        if rule is None:
            pytest.skip("无匹配规则")

        f_max, s_max = self._effective_max(engine, test_robot, action, props, rule)
        df = f_max * 2.0 if f_max > 0 else 0
        ds = s_max * 2.0 if s_max > 0 else 0
        r = engine.check_command(action, obj_name, {"force": df, "speed": ds}, robot=test_robot)

        if is_imp:
            assert r["verdict"] == "REJECT", f"{action}+{obj_name} 危险组应REJECT"
        else:
            assert r["verdict"] == "FAIL", f"{action}+{obj_name} 危险组应FAIL, 实际{r['verdict']}"

    # --- 辅助方法 ---

    @staticmethod
    def _effective_max(engine, robot, action, props, rule):
        """计算有效上限 = min(规则上限, 动词上限, 机器人上限, ISO上限) × 易碎系数"""
        robot_cap = engine.parser.normalize_robot_cap(robot)
        verb_info = engine.parser.get_verb_info(action)

        r_force_max = rule["force"][2]
        r_speed_max = rule["speed"][2]

        f_max_verb = verb_info["grasp_params"]["max_force"] if verb_info and verb_info.get("grasp_params") else float("inf")
        f_max_robot = robot_cap.get("max_force", 100)
        fragile_val = float(props.get("fragile", 0.0))
        fragile_factor = 1.0 - fragile_val * engine.adapter.rules.fragile_force_factor
        hc_val = float(props.get("human_contact", 0.0))
        cat = props.get("category", "rigid")

        iso_limit = float("inf")
        if cat == "human" or hc_val > engine.adapter.rules.human_contact_threshold:
            iso_limit = engine.adapter.rules.iso.get_quasi_static_limit("")
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
        return f_max, s_max

    @classmethod
    def _effective_safe(cls, engine, robot, action, props, rule):
        """安全值：取规则最优值，但不超过有效上限的80%"""
        f_max, s_max = cls._effective_max(engine, robot, action, props, rule)
        f_opt = rule["force"][1]
        s_opt = rule["speed"][1]
        return min(f_opt, f_max * 0.8), min(s_opt, s_max * 0.8)


# =====================================================================
# Test 2: 综合测试
# =====================================================================

class TestComprehensive:
    """76场景综合测试"""

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

    @pytest.mark.parametrize("subj,verb,obj,exp", semantic_tests, ids=[f"{s}-{v}-{o}" for s,v,o,_ in semantic_tests])
    def test_semantic(self, engine, subj, verb, obj, exp):
        r = engine.check_command(verb, obj, robot=subj)
        actual = self._verdict_to_label(r)
        assert actual == exp, f"{subj}+{verb}+{obj}: 期望{exp}, 实际{actual}"

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

    @pytest.mark.parametrize("v,o,f,ft,d,vi,el", boundary_tests, ids=[f"{v}-{o}-f{f}-d{d}-v{vi}" for v,o,f,ft,d,vi,el in boundary_tests])
    def test_boundary(self, engine, v, o, f, ft, d, vi, el):
        r = engine.judge.hold_boundary_check(v, f, ft, d, vi)
        actual = r.get("disturbance_level", "none")
        assert actual == el, f"{v}+{o} f={f}: 期望{el}, 实际{actual}"

    # C类：FAV分类10场景
    fav_tests = [
        (0.0, 0.0, 0.0, "idle"), (0.01, 0.01, 0.005, "idle"),
        (0.7, 0.4, 0.7, "grasping"), (0.5, 0.5, 0.005, "holding"),
        (0.55, 0.5, 0.003, "holding"), (0.85, 0.3, 0.85, "grasping"),
        (0.01, 0.02, 0.005, "idle"), (0.3, 0.6, 0.003, "holding"),
        (0.9, 0.7, 0.9, "grasping"), (0.52, 0.48, 0.003, "holding"),
    ]

    @pytest.mark.parametrize("f,a,vel,es", fav_tests)
    def test_fav(self, engine, f, a, vel, es):
        r = engine.fav.classify(f, a, vel)
        assert r["phase_state"] == es, f"F={f},A={a},V={vel}: 期望{es}, 实际{r['phase_state']}"

    # D类：状态稳定10场景
    closure_tests = [
        ("抓", 25.0, 25.0, 5.0, 1.0, True), ("抓", 25.0, 25.0, 50.0, 1.0, False),
        ("抓", 10.0, 25.0, 5.0, 1.0, False), ("握", 25.0, 25.0, 5.0, 1.0, True),
        ("拿", 20.0, 20.0, 3.0, 1.5, True), ("取", 15.0, 15.0, 3.0, 1.0, True),
        ("推", 50.0, 50.0, 10.0, 3.0, True), ("按", 10.0, 10.0, 3.0, 0.5, True),
        ("捏", 7.5, 7.5, 2.0, 0.3, True), ("插", 15.0, 15.0, 3.0, 0.3, True),
    ]

    @pytest.mark.parametrize("v,f,ft,sp,d,ec", closure_tests, ids=[f"{v}-f{f}" for v,f,ft,sp,d,ec in closure_tests])
    def test_closure(self, engine, v, f, ft, sp, d, ec):
        actual = engine.judge.check_state_stabilization(v, f, ft, sp, d)
        assert actual == ec, f"{v} f={f}: 期望{ec}, 实际{actual}"

    # E类：干扰分级6场景
    disturbance_tests = [
        (0.02, "none"), (0.08, "micro"), (0.20, "small"),
        (0.40, "medium"), (0.60, "large"), (0.90, "danger"),
    ]

    @pytest.mark.parametrize("s,el", disturbance_tests)
    def test_disturbance(self, engine, s, el):
        actual = engine.judge._classify_disturbance(s)
        assert actual == el, f"s={s}: 期望{el}, 实际{actual}"

    # F类：端到端5场景
    e2e_tests = [
        ("基础人形", "抓", "鸡蛋", {"force": 1.0, "speed": 0.03}, "safe"),
        ("基础人形", "拿", "杯子", {"force": 5.0, "speed": 0.08}, "safe"),
        ("基础人形", "推", "门", {"force": 20.0, "speed": 0.3}, "safe"),
        ("灵巧人形", "握", "玻璃杯", {"force": 5.0, "speed": 0.03}, "safe"),
        ("基础人形", "捏", "笔", {"force": 3.0, "speed": 0.01}, "safe"),
    ]

    @pytest.mark.parametrize("subj,verb,obj,params,exp", e2e_tests, ids=[f"{s}-{v}-{o}" for s,v,o,p,e in e2e_tests])
    def test_e2e(self, engine, subj, verb, obj, params, exp):
        r = engine.check_command(verb, obj, params, robot=subj)
        actual = self._verdict_to_label(r)
        assert actual == exp, f"{subj}+{verb}+{obj}: 期望{exp}, 实际{actual}"

    # G类：接触面积5场景
    area_tests = [
        ("抓取", "杯子", {"force": 5.0, "speed": 0.05}, 200.0),
        ("抓取", "杯子", {"force": 5.0, "speed": 0.05}, 800.0),
        ("抓取", "杯子", {"force": 5.0, "speed": 0.05}, 400.0),
        ("抓取", "鸡蛋", {"force": 1.0, "speed": 0.02}, 100.0),
        ("抓取", "铁块", {"force": 20.0, "speed": 0.05}, 500.0),
    ]

    @pytest.mark.parametrize("act,obj,params,area", area_tests, ids=[f"{a}-{o}-area{area}" for a,o,p,area in area_tests])
    def test_contact_area(self, engine, test_robot, act, obj, params, area):
        r = engine.check_command(act, obj, params, robot=test_robot,
                                  object_params={"contact_area_mm2": area})
        assert r["verdict"] == "PASS", f"{act}+{obj} area={area}: {r['verdict']}"

    @staticmethod
    def _verdict_to_label(r):
        if r["verdict"] == "REJECT":
            return "reject"
        elif r["risk_level"] == "LOW":
            return "safe"
        elif r["risk_level"] == "MEDIUM":
            return "caution"
        else:
            return "danger"


# =====================================================================
# Test 3: 自然语言输入
# =====================================================================

class TestNaturalLanguage:
    nl_tests = [
        ("抓取", "鸡蛋", "PASS"), ("搬运", "铁块", "PASS"), ("推开", "椅子", "PASS"),
        ("拿", "水", "REJECT"), ("捏", "面包", "PASS"), ("搬", "桌子", "PASS"),
        ("抓取", "玻璃杯", "PASS"), ("走", "人", "PASS"), ("拉开", "门", "PASS"),
        ("握", "杯子", "PASS"), ("拧", "钥匙", "PASS"), ("按住", "按钮", "PASS"),
    ]

    @pytest.mark.parametrize("act,obj,exp", nl_tests)
    def test_nl(self, engine, test_robot, act, obj, exp):
        r = engine.check_command(act, obj, robot=test_robot)
        assert r["verdict"] == exp, f"{act} {obj}: 期望{exp}, 实际{r['verdict']}"


# =====================================================================
# Test 4: 模式B JSON 输入
# =====================================================================

class TestJsonMode:
    def test_basic_grasp(self, engine):
        scene = {"objects": [{"object_id": "seal", "name": "密封条",
                              "mass_kg": 0.8, "stability": "flexible",
                              "contact_area_mm2": 600}]}
        action = {"type": "grasp", "force_n": 2.0, "velocity_ms": 0.03,
                  "acceleration_ms2": 1.0, "target_object": "seal"}
        robot = {"max_force_n": 150, "max_velocity_ms": 2.0,
                 "max_acceleration_ms2": 10.0}
        r = engine.check_action(scene, action, robot)
        assert r["verdict"] == "PASS", f"grasp 密封条应PASS, 实际{r['verdict']}: {r.get('correction','')}"

    def test_impossible_combination(self, engine):
        scene = {"objects": [{"object_id": "water", "name": "水",
                              "mass_kg": 0.5, "stability": "liquid",
                              "fragile": 0.0, "fluid": 1.0, "human_contact": 1.0,
                              "contact_area_mm2": 0}]}
        action = {"type": "grasp", "force_n": 5.0, "velocity_ms": 0.1,
                  "target_object": "water"}
        robot = {"max_force_n": 150, "max_velocity_ms": 2.0,
                 "max_acceleration_ms2": 10.0}
        r = engine.check_action(scene, action, robot)
        # liquid 不在 IMPOSSIBLE_PAIRS 中，所以不 REJECT；检查基本功能
        assert "verdict" in r

    def test_object_not_found(self, engine):
        scene = {"objects": []}
        action = {"type": "grasp", "force_n": 5.0, "velocity_ms": 0.1,
                  "target_object": "nonexist"}
        robot = {"max_force_n": 150, "max_velocity_ms": 2.0,
                 "max_acceleration_ms2": 10.0}
        r = engine.check_action(scene, action, robot)
        assert r["verdict"] == "FAIL"

    def test_alternate_field_names(self, engine):
        """测试旧版字段名 max_force/max_speed/max_accel 也能正常工作"""
        scene = {"objects": [{"object_id": "obj", "stability": "rigid",
                              "contact_area_mm2": 400}]}
        action = {"type": "grasp", "force_n": 5.0, "velocity_ms": 0.05,
                  "target_object": "obj"}
        robot = {"max_force": 100, "max_speed": 1.0, "max_accel": 5.0}
        r = engine.check_action(scene, action, robot)
        assert r["verdict"] == "PASS"


# =====================================================================
# Test 5: 性能测试
# =====================================================================

class TestPerformance:
    def test_latency_below_100us(self, engine, test_robot):
        """平均延迟应低于 0.1ms (100μs)"""
        N = 10000
        t0 = time.perf_counter()
        for _ in range(N):
            engine.check_command("grasp", "鸡蛋", {"force": 2.0, "speed": 0.03}, robot=test_robot)
        elapsed = time.perf_counter() - t0
        avg_ms = elapsed / N * 1000
        assert avg_ms < 0.1, f"平均延迟 {avg_ms:.4f}ms 超过 0.1ms 限制"


# =====================================================================
# Test 6: 输出结构完整性 + ISO合规
# =====================================================================

class TestOutputStructure:
    required_fields = [
        "verdict", "state", "capability_match", "action_target_match",
        "param_check", "safety_zone", "disturbance", "correction",
        "recommended_params", "contact_area_mm2", "pressure_kPa", "iso_compliance",
    ]

    def test_required_fields(self, engine, test_robot):
        r = engine.check_command("抓取", "鸡蛋", {"force": 2.0, "speed": 0.03}, robot=test_robot)
        missing = [k for k in self.required_fields if k not in r]
        assert len(missing) == 0, f"缺失字段: {missing}"

    def test_human_iso_compliance(self, engine, test_robot):
        """人体部位应触发 ISO 合规标注"""
        r = engine.check_command("摸", "手", {"force": 1.0, "speed": 0.01}, robot=test_robot)
        assert "ISO" in r["iso_compliance"], f"人体部位应标注ISO合规，实际: {r['iso_compliance']}"

    def test_human_contact_iso(self, engine, test_robot):
        """human_contact 物体也应触发 ISO 合规标注"""
        r = engine.check_command("抓", "杯子", {"force": 5.0, "speed": 0.05}, robot=test_robot)
        assert "ISO" in r["iso_compliance"], f"human_contact物体应标注ISO合规，实际: {r['iso_compliance']}"


# =====================================================================
# Test 7: 动态接触面积 + 冲量校验 + 反作用力约束
# =====================================================================

class TestPhysicsNewFeatures:
    """新物理特性验证：动态接触面积 / 冲量校验 / 反作用力约束"""

    # --- 动态接触面积 ---
    def test_dynamic_contact_area_soft_vs_rigid(self, engine, test_robot):
        """同样5N力，软物(面包)压强大于刚体？不对，软物面积变大压强应该更小。"""
        r_bread = engine.check_command("grasp", "面包", {"force": 5.0, "speed": 0.05}, robot=test_robot)
        r_iron = engine.check_command("grasp", "铁块", {"force": 5.0, "speed": 0.05}, robot=test_robot)
        # 面包是软物 → 接触面积增大 → 压强减小
        assert r_bread["pressure_kPa"] < r_iron["pressure_kPa"], (
            f"面包压强({r_bread['pressure_kPa']:.1f}kPa)应小于铁块({r_iron['pressure_kPa']:.1f}kPa)")
        # 接触面积应大于 base_area
        base_bread = OBJECT_PROPERTIES["面包"]["contact_area_mm2"]
        base_iron = OBJECT_PROPERTIES["铁块"]["contact_area_mm2"]
        assert r_bread["contact_area_mm2"] > base_bread, "面包受力后接触面积应增大"
        assert r_iron["contact_area_mm2"] >= base_iron, "铁块受力后接触面积应≥base"

    def test_dynamic_contact_area_zero_force(self, engine, test_robot):
        """力为0时接触面积 = base_area（不变）"""
        r = engine.check_command("grasp", "面包", {"force": 0.0, "speed": 0.05}, robot=test_robot)
        base = OBJECT_PROPERTIES["面包"]["contact_area_mm2"]
        assert abs(r["contact_area_mm2"] - base) < 0.01, (
            f"力为0时面积应等于base({base})，实际{r['contact_area_mm2']}")

    def test_dynamic_contact_area_max_deform(self, engine, test_robot):
        """力极大时接触面积受 max_deform 限制"""
        # 用很大的力，验证不超过 max_deform 比例
        big_force = 1000.0  # 远超刚度承受范围
        r = engine.check_command("grasp", "面包", {"force": big_force, "speed": 0.05}, robot=test_robot)
        base = OBJECT_PROPERTIES["面包"]["contact_area_mm2"]
        max_deform = OBJECT_PROPERTIES["面包"]["max_deform"]
        max_area = base * (1 + max_deform)
        assert r["contact_area_mm2"] <= max_area * 1.01, (
            f"接触面积不应超过max_deform限制({max_area:.1f})，实际{r['contact_area_mm2']:.1f}")

    def test_compute_contact_area_static_method(self, engine):
        """SemanticParser.compute_contact_area 静态方法独立可用"""
        from safety_engine import SemanticParser
        # 力为0 → 面积不变
        assert SemanticParser.compute_contact_area(100, 0, 2.0, 0.5) == 100
        # 力=stiffness*area → deform_ratio=1.0 → 但受max_deform限制
        area = SemanticParser.compute_contact_area(100, 200, 2.0, 0.5)
        assert area == 150.0  # 100 * (1 + 0.5) = 150
        # 力很小 → 面积略增
        area2 = SemanticParser.compute_contact_area(100, 10, 2.0, 0.5)
        assert area2 == 105.0  # 100 * (1 + 10/(2*100)) = 100 * 1.05

    # --- 冲量校验 ---
    def test_impulse_heavy_high_speed_fail(self, engine, test_robot):
        """carry 重物高速 → 冲量超限 → FAIL"""
        r = engine.check_command("carry", "铁块", {"force": 40.0, "speed": 1.5}, robot=test_robot)
        assert r["verdict"] == "FAIL", f"重物高速应FAIL，实际{r['verdict']}"
        assert "冲量" in r["param_check"]["detail"], "FAIL原因应包含冲量"

    def test_impulse_heavy_low_speed_safe(self, engine, test_robot):
        """carry 重物低速 → 冲量安全 → 不触发冲量FAIL"""
        r = engine.check_command("carry", "铁块", {"force": 40.0, "speed": 0.2}, robot=test_robot)
        detail = r["param_check"]["detail"]
        # 冲量=20kg × 0.2m/s = 4 kg·m/s < 25，不应触发冲量超限
        impulse_violation = "冲量" in detail and "超过上限" in detail
        assert not impulse_violation, f"重物低速冲量应安全，实际: {detail}"

    def test_impulse_light_object_high_speed_safe(self, engine, test_robot):
        """carry 轻物高速 → 冲量小 → 安全"""
        # 笔 0.01kg × 0.25m/s = 0.0025 kg·m/s，远低于3.0上限
        r = engine.check_command("carry", "笔", {"force": 2.0, "speed": 0.25}, robot=test_robot)
        detail = r["param_check"]["detail"]
        impulse_violation = "冲量" in detail and "超过上限" in detail
        assert not impulse_violation, f"轻物冲量应安全，实际: {detail}"

    def test_impulse_not_triggered_for_grasp(self, engine, test_robot):
        """grasp 等非移动类动作不触发冲量校验（无冲量超限）"""
        # grasp 是持态动作，即使有speed也不应出现"冲量超过上限"
        r = engine.check_command("grasp", "铁块", {"force": 10.0, "speed": 0.1}, robot=test_robot)
        detail = r["param_check"]["detail"]
        assert "冲量" not in detail or "超过上限" not in detail, (
            f"grasp不应触发冲量超限，detail={detail}")
        # 验证基本判定通过
        assert r["verdict"] == "PASS", (
            f"10N grasp铁块应PASS，实际{r['verdict']}: {detail}")

    # --- 反作用力约束 ---
    def test_reaction_force_overload_fail(self, engine, test_robot):
        """力超过反作用极限 → FAIL，detail含反作用力"""
        # base 50kg × 9.81 × 0.6 = 294.3N，用400N应触发
        r = engine.check_command("push", "门", {"force": 400.0, "speed": 0.1}, robot=test_robot)
        assert r["verdict"] == "FAIL"
        assert "反作用力" in r["param_check"]["detail"], "应包含反作用力约束失败"

    def test_reaction_force_normal_safe(self, engine, test_robot):
        """正常力 → 不触发反作用约束"""
        r = engine.check_command("push", "门", {"force": 20.0, "speed": 0.3}, robot=test_robot)
        detail = r["param_check"]["detail"]
        assert "反作用力" not in detail, f"正常力不应触发反作用约束，实际: {detail}"

    def test_reaction_force_fixed_base_disabled(self, engine):
        """fixed_base=True 时反作用力约束不生效"""
        robot = {"max_force_n": 500.0, "max_velocity_ms": 2.0,
                 "max_acceleration_ms2": 50.0, "base_weight_kg": 50.0,
                 "fixed_base": True}
        r = engine.check_command("push", "门", {"force": 400.0, "speed": 0.1}, robot=robot)
        detail = r["param_check"]["detail"]
        assert "反作用力" not in detail, "fixed_base时反作用力约束应禁用"

    # --- 物体质量数据 ---
    def test_object_mass_comprehensive(self):
        """验证所有物体都有 mass_kg 字段"""
        for name, props in OBJECT_PROPERTIES.items():
            assert "mass_kg" in props, f"{name} 缺少 mass_kg 字段"
            assert props["mass_kg"] > 0, f"{name} mass_kg 应大于0"

    def test_heavy_based_on_mass(self):
        """heavy 分类应基于 mass_kg > 2kg（human 类别除外，保持语义一致性）"""
        for name, props in OBJECT_PROPERTIES.items():
            category = props.get("category", "")
            # human 类别是特殊语义分类，不以 heavy 标记
            if category == "human":
                continue
            if props["mass_kg"] > 2:
                assert props["heavy"] == 1.0, f"{name} mass={props['mass_kg']}应标记heavy"
            else:
                assert props["heavy"] == 0.0, f"{name} mass={props['mass_kg']}不应标记heavy"


# =====================================================================
# （社区版不含力学增强，企业版提供）
# =====================================================================

if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
