# Rotor Safety Engine

**The deterministic physics safety layer between VLA models and physical robots.**

> "We don't understand your task. We just make sure it won't break physics."

---

## Why you need it

VLA models generate actions from language and vision — but they don't _know physics_.
A model can decide to "grab the glass" without realizing its grip force would shatter it,
or "reach for the cup" at a speed that would break a hand on collision.

Rotor Safety Engine is the **runtime safety guard** that sits between your VLA model
and the robot controller. It doesn't try to understand the task — it computes whether
the action is physically safe, in **~17 microseconds**.

- ⚡ **~17μs latency** — 58,000+ checks/sec on a single CPU core
- 🛡️ **ISO 10218 / ISO/TS 15066 aligned** — designed with industrial safety standards in mind
- 🧠 **No neural nets, 100% deterministic** — pure Newtonian physics, same input → same output
- 📦 **Single file · Zero dependencies** — one Python file, stdlib only, drop in anywhere

---

## Quick Start

Install from PyPI:

```bash
pip install rotor-safety-engine
```

Or just copy the single file:

```bash
curl -O https://raw.githubusercontent.com/Rotor-Safety-Engine/safety-engine/main/src/safety_engine.py
```

### 30-second demo

```python
from safety_engine import SafetyEngineV4

engine = SafetyEngineV4()

# Safe grasp → PASS
r = engine.check_command("grasp", "egg", params={"force": 2.0, "speed": 0.03}, robot="humanoid_basic")
print(f"{r['verdict']} | risk={r['risk_level']} | pressure={r['physics']['pressure_kpa']:.1f}kPa")
# ✅ PASS | risk=LOW | pressure=12.3kPa

# Dangerous force → FAIL
r = engine.check_command("grasp", "glass_panel", params={"force": 80.0, "speed": 0.5}, robot="humanoid_basic")
print(f"{r['verdict']} | risk={r['risk_level']} | over_ratio={r['over_ratio']:.1f}x")
# ❌ FAIL | risk=HIGH | over_ratio=2.3x
```

---

## How is this different?

| | **Rotor Safety Engine** | ROS MoveIt Safety | Safety Gymnasium |
|---|---|---|---|
| **Latency** | **~17μs** | ~ms range | ~ms range |
| **Deployment** | Single Python file | Full ROS stack | RL env only |
| **VLA-ready** | ✅ drop-in guard layer | ❌ | ❌ |
| **Dynamic contact area** | ✅ pressure-based | ❌ force-only | ⚠️ limited |
| **Impulse boundary** | ✅ mass × velocity | ❌ | ⚠️ partial |
| **Reaction force stability** | ✅ base stability check | ❌ | ❌ |
| **7-level risk granularity** | ✅ L0–L6 + over_ratio | ❌ binary/3-level | ❌ 3-level |
| **Deterministic** | ✅ 100% | ✅ | ❌ statistical |

---

## 五层安全架构

```
输入（动作 + 物体 + 参数）
    │
    ▼
┌──────────────────────────┐
│ Layer 1  语义解析层       │
│  动词库 · 对象属性库       │
│  机器人能力表 · 规则匹配   │
│  安全区间计算 · 参数推荐   │
└───────────┬──────────────┘
            │
            ▼
┌──────────────────────────┐
│ Layer 2  安全适配层       │
│  机器人能力约束            │
│  动作-目标兼容性校验        │
│  参数安全阈值校验          │
└───────────┬──────────────┘
            │
            ▼
┌──────────────────────────┐
│ Layer 3  动作分类层       │
│  FAV 三维分类器           │
│  动作阶段自动识别          │
│  （idle / grasping / holding）
└───────────┬──────────────┘
            │
            ▼
┌──────────────────────────┐
│ Layer 4  综合决策层       │
│  PASS / FAIL / REJECT    │
│  全量输出 + 修正建议      │
│  ISO 合规标注            │
└───────────┬──────────────┘
            │
            ▼
┌──────────────────────────┐
│ Layer 5  力学增强层 v4.3  │
│  作用-反作用完整对         │
│  动量-冲量-时间链路        │
│  （能量守恒待后续版本）    │
└──────────────────────────┘
```

---

## 快速开始

### 安装

```bash
# 单文件，直接拷走即可
cp src/safety_engine.py your_project/
```

### 基本使用 — 自然语言模式

```python
from safety_engine import SafetyEngineV4

engine = SafetyEngineV4()

# 正常操作 → PASS
result = engine.check_command("抓取", "鸡蛋",
                              params={"force": 2.0, "speed": 0.03},
                              robot="humanoid_basic")
print(result["verdict"])      # PASS
print(result["risk_level"])   # LOW

# 力太大 → FAIL
result = engine.check_command("抓", "玻璃面板",
                              params={"force": 80.0, "speed": 0.5},
                              robot="humanoid_basic")
print(result["verdict"])      # FAIL
print(result["correction"])   # 参数不安全: ...
```

### 基本使用 — JSON 参数模式（生产推荐）

```python
scene = {
    "objects": [{
        "object_id": "metal_part",
        "name": "铝合金工件",
        "mass_kg": 2.5,
        "stability": "rigid",
        "contact_area_mm2": 500,
    }]
}

action = {
    "type": "grasp",
    "force_n": 25.0,
    "velocity_ms": 0.3,
    "acceleration_ms2": 3.0,
    "target_object": "metal_part",
}

robot = {
    "max_force_n": 150,
    "max_velocity_ms": 2.0,
    "max_acceleration_ms2": 10.0,
}

result = engine.check_action(scene, action, robot)
print(result["verdict"])       # PASS
print(result["pressure_kPa"])  # 接触压强
```

### 运行 Demo

```bash
python examples/demo.py
```

### 运行测试

```bash
# 方式 1：直接运行（零依赖，推荐快速验证）
python tests/test_engine.py

# 方式 2：pytest 运行（需安装 pytest）
pip install pytest
pytest tests/test_engine.py -v
pytest tests/test_engine.py -v --tb=short   # 失败时显示精简堆栈
```

测试覆盖：
- **测试 1**：4 动作 × 20 物体 × 3 参数组（安全/边界/危险）= 240 场景准确率
- **测试 2**：76 场景综合测试（语义 / 边界 / FAV / 闭合 / 干扰 / 端到端 / 接触面积）
- **测试 3**：12 个自然语言输入场景
- **测试 4**：JSON 参数模式（模式 B）端到端
- **测试 5**：性能基准（平均延迟 < 100μs）
- **测试 6**：输出结构完整性 + ISO 合规标注
- **测试 7**：动态接触面积 + 冲量校验 + 反作用力约束（7 项物理验证）

---

## API 说明

### `check_action(scene_data, action_data, robot_data) -> dict`

JSON 参数模式，生产环境推荐。零解析开销，延迟最低。

**scene_data（场景数据）**

| 字段 | 类型 | 说明 |
|------|------|------|
| objects | list | 场景中的物体列表，每个物体含 object_id / mass_kg / stability / contact_area_mm2 |

**action_data（动作数据）**

| 字段 | 类型 | 说明 |
|------|------|------|
| type | string | 动作类型：grasp / carry / push / pull / lift / place / press / insert / rotate / hold / release 等 |
| force_n | float | 作用力（N） |
| velocity_ms | float | 运动速度（m/s） |
| acceleration_ms2 | float | 加速度（m/s²） |
| target_object | string | 目标物体 ID |

**robot_data（机器人能力）**

| 字段 | 类型 | 说明 |
|------|------|------|
| max_force_n | float | 最大作用力（N） |
| max_velocity_ms | float | 最大速度（m/s） |
| max_acceleration_ms2 | float | 最大加速度（m/s²） |

### `check_command(action, obj, params=None, robot=None) -> dict`

自然语言模式，快速原型和调试用。

### 返回值字段

| 字段 | 类型 | 说明 |
|------|------|------|
| `verdict` | string | 最终判定：`PASS` / `FAIL` / `REJECT` |
| `state` | string | 动作状态：`idle` / `grasp` / `hold` |
| `risk_level` | string | 风险等级：`LOW` / `MEDIUM` / `HIGH` |
| `latency_ms` | float | 本次判定耗时（毫秒） |
| `capability_match` | object | 能力适配结果（得分 + 详情） |
| `action_target_match` | object | 动作-目标适配结果（得分 + 详情） |
| `param_check` | object | 参数安全校验结果（得分 + 详情 + 安全裕度） |
| `safety_zone` | object | 安全区间（最优值 / 上限 / 下限） |
| `disturbance` | object | 干扰等级评估 |
| `correction` | string | 判定原因 / 修正建议 |
| `recommended_params` | object | 推荐参数 |
| `contact_area_mm2` | float | 接触面积（mm²） |
| `pressure_kPa` | float | 接触压强（kPa） |
| `iso_compliance` | string | ISO 合规标注（人体接触场景自动触发） |
| `risk_level_7` | string | 七级风险等级（L0~L6），v4.2.2+ |
| `risk_subtypes` | array | 风险子类型标签列表，v4.2.2+ |
| `recommended_params_v2` | object | FAIL 样本智能推荐参数，v4.2.2+ |
| `retreat_params` | object/null | 边界 PASS 样本安全回退参数，v4.2.2+ |
| `semantic_plausibility_score` | float/null | 语义合理性分数透传，v4.2.2+ |
| `over_ratio` | float | 超标倍率（FAIL=最严维度实际值/限值之比，PASS=0.0），v4.2.3+ |
| `physics` | object | 力学分析输出（Layer1 + Layer2 两层封装），v4.3.0+，详见【力学增强】章节 |

---

## 力学增强（v4.3.0 新增）

v4.3 引入了力学计算层（Layer 5），将原先隐性的判定约束升级为**显性的、结构化的、可输出的力学计算结果**，为后续力学仿真引擎和世界模型物理先验打基础。

> **设计原则**：力学增强是"加输出"，不是"改判定"。所有判定逻辑（PASS/FAIL/REJECT、风险等级）与 v4.2.3 完全一致，仅新增 `physics` 字段。
> **向后兼容**：不传任何新参数时，使用默认值自动填充，零配置即可使用。

### 两层力学架构（核心两层，Layer 3 待后续版本）

```
physics
  ├── action_reaction       ← Layer 1: 作用-反作用完整对
  └── momentum_analysis     ← Layer 2: 动量-冲量-时间链路
  （energy_analysis         ← Layer 3: 能量守恒校验，下次版本再做）
```

### Layer 1：作用-反作用完整对（action_reaction）

| 字段 | 类型 | 说明 |
|------|------|------|
| `action_force_magnitude_N` | float | 作用力大小（N） |
| `action_force_direction` | list[float] | 作用力方向单位向量（3D，[x, y, z]） |
| `reaction_force_magnitude_N` | float | 反作用力大小（N），与作用力大小相等（牛顿第三定律） |
| `reaction_force_direction` | list[float] | 反作用力方向单位向量（3D），与作用力方向相反 |
| `normal_force_N` | float | 法向分量（N），垂直接触面 |
| `friction_force_N` | float | 切向摩擦力（N），μ × 法向力 |
| `contact_point_offset_m` | list[float] | 作用点相对于物体中心的偏移（m），3D [dx, dy, dz] |
| `action_torque_Nm` | float | 作用力矩（N·m），力 × 力臂 |
| `reaction_torque_Nm` | float | 反作用力矩（N·m），大小相等方向相反 |

**简化假设**（不追求仿真精度，要的是物理框架完整）：
- 对于标量力输入，默认作用力沿 x 轴正方向（推力）或 z 轴负方向（压力/抓取）
- 接触面默认在 y-z 平面（法向为 x 轴），法向力 = 作用力的 x 分量
- 接触点默认在物体表面中心，偏移 = [物体_radius, 0, 0]，物体半径从接触面积反推（面积 = πr²）
- 力矩简化为平面内计算（力 × 偏移距离）
- 摩擦力方向与作用力切向分量相反

**动作类型到方向的映射**：

| 动作类型 | 主方向 | 说明 |
|----------|--------|------|
| push / pull | +x / -x | 水平方向 |
| press / grasp / hold | +x（法向） | 垂直于接触面方向 |
| carry / lift | +y | 垂直向上 |
| place / release | -y | 垂直向下 |
| insert / rotate | 混合 | 力矩为主 |
| 其他默认 | +x | 水平 |

### Layer 2：动量-冲量-时间链路（momentum_analysis）

| 字段 | 类型 | 说明 |
|------|------|------|
| `impulse_Ns` | float | 冲量（N·s = kg·m/s），力 × 接触时间 |
| `contact_duration_s` | float | 接触持续时间（s） |
| `force_time_profile` | string | 力-时曲线类型：`instant_impact`（瞬时冲击）/ `steady_pressure`（稳态施压）/ `gradual_loading`（渐进加载） |
| `momentum_transfer_kgms` | float | 动量传递量（kg·m/s），= 冲量（牛顿第三定律） |
| `object_mass_kg` | float | 物体质量（kg），从物体属性读取，无则按类别估算 |
| `velocity_change_mps` | float | 物体速度变化（m/s）= 动量变化 / 质量 |
| `kinetic_energy_change_J` | float | 动能变化（J）= 0.5 × m × Δv² |

**物体质量估算**（物体库没有 mass_kg 字段时按类别估算）：

| 类别 | 质量范围 | 典型值 |
|------|----------|--------|
| tiny / 微小 | 0.01~0.1 kg | 0.05 kg |
| light / 轻物 | 0.1~1 kg | 0.2~0.5 kg |
| medium / 中等 | 1~5 kg | 2 kg |
| heavy / 重物 | 5~20 kg | 10~20 kg |
| very_heavy / 极重 | 20~100 kg | 25~70 kg |

**接触时间默认值**：

| 动作类别 | 默认接触时间 | 说明 |
|----------|-------------|------|
| 冲击类（impact / collision） | 0.01 s | 瞬时接触 |
| 普通动作（push / pull / press） | 0.1~0.5 s | 正常交互 |
| 持续动作（hold / carry / lift） | 1.0 s | 稳态保持 |

### 使用方式

力学分析自动触发，无需额外配置：

```python
from safety_engine import SafetyEngineV4

engine = SafetyEngineV4()

# 基础使用 — 自动计算，零配置
result = engine.check_command("push", "box",
                              params={"force": 50, "speed": 0.5})
physics = result["physics"]

# 查看作用-反作用力
print(physics["action_reaction"]["action_force_magnitude_N"])     # 50.0
print(physics["action_reaction"]["reaction_force_magnitude_N"])   # 50.0
print(physics["action_reaction"]["action_force_direction"])       # [1.0, 0.0, 0.0]

# 查看动量变化
print(physics["momentum_analysis"]["impulse_Ns"])                  # 冲量
print(physics["momentum_analysis"]["momentum_transfer_kgms"])      # 动量传递
print(physics["momentum_analysis"]["velocity_change_mps"])         # 速度变化
print(physics["momentum_analysis"]["kinetic_energy_change_J"])     # 动能变化

# 高级使用 — 传入更丰富的物理参数（通过 context）
result = engine.check_command("push", "box",
                              params={"force": 50, "speed": 0.5},
                              context={
                                  "contact_duration_s": 2.0,
                                  "force_direction": [1.0, 0.0, 0.0],
                                  "contact_point_mm": [50.0, 20.0, 0.0],
                              })
```

### 关键公式

**牛顿第三定律（作用与反作用）**：
```
F_action = -F_reaction  （大小相等，方向相反）
```

**冲量-动量定理**：
```
I = F_net × Δt = Δp = m × Δv
```

**动能变化**：
```
ΔE_k = 0.5 × m × Δv²
```

**摩擦力**：
```
f = μ × N  （μ = 摩擦系数，N = 法向力）
```

---

## 性能指标

> 测试环境：Python 3.10+ / x86_64 / JSON 参数模式 / 10 万次循环

| 指标 | 数值 |
|------|------|
| 平均延迟 | **~15 μs**（0.015 ms） |
| P95 延迟 | ~18 μs |
| P99 延迟 | ~22 μs |
| 单线程吞吐量 | **~100,000 次/秒** |
| 自然语言模式延迟 | 0.3–0.8 ms |
| 内存占用 | < 5 MB |
| 外部依赖 | 0（仅 Python 标准库） |
| 确定性 | 100%（相同输入永远相同输出） |

---

## 技术原理

### 三层力约束模型

从动量-能量双守恒推导，力的安全性由三层约束共同决定，最终取最严格值：

1. **输出端约束** — 机器人输出能力上限（电机 / 关节极限）
2. **接收端约束** — 物体力学响应上限（材料 / 结构极限，含动态接触面积、冲量、压强计算）
3. **双向约束** — 反作用力与本体稳定性（牛顿第三定律，机器人基座摩擦力上限）

### 动态接触面积

软质 / 易碎物体在受力后接触面积会增大，压强随面积动态调整：
- 刚体（金属、石头）：刚度大，接触面积基本不变
- 软物（面包、水果）：刚度小，接触面积随力线性增长，最大可达 base_area 的 1.5 倍
- 易碎物（鸡蛋、玻璃）：变形极小但压强大了直接碎，以压强阈值判定

### 冲量安全校验

对 carry / move / push / pull / lift 等有位移的动作，增加 `冲量 = 质量 × 速度` 的安全约束：
- 易碎物 / 液体：冲量上限低（防止碎裂 / 晃洒）
- 重物 / 刚体：冲量上限高
- 人体接触：冲量上限收紧（避免高速碰撞）

非移动类动作（grasp / hold / press 等）不触发冲量校验。

### 反作用力约束

机器人对物体施加的力，等于物体对机器人的反作用力（牛顿第三定律）。
当反作用力超过机器人基座能提供的最大静摩擦力时，机器人会失稳。

```
max_reaction = base_weight_kg × G × friction_coef
```

- `base_weight_kg`：机器人自重 / 基座重量
- `ground_friction`：地面摩擦系数（robot_data 可配置，默认 0.6 橡胶-地面）
  - 光滑大理石 ~0.2，草地 ~0.5，冰面 ~0.1，橡胶地面 ~0.6-1.0
- `fixed_base` 标记的工业机器人不受此约束

---

## 七级风险分级规则

引擎采用七级风险等级（L0~L6），对 PASS 和 FAIL 样本分别使用不同的划分依据：

- **PASS 样本**：按 `safety_margin`（安全裕度）分档，裕度越高越安全
- **FAIL 样本**：按 `over_ratio`（超标倍率）分档，倍率越大越危险

| 等级 | 中文标签 | 判定 | 划分依据 | 范围 | 典型场景 |
|------|----------|------|----------|------|----------|
| **L0** | 安全 | PASS | safety_margin | 0.50 ~ 1.00 | 日常抓取轻物、低速操作，远低于安全边界 |
| **L1** | 低风险 | PASS | safety_margin | 0.30 ~ 0.50 | 中等参数操作，有一定安全余量 |
| **L2** | 中低风险 | PASS | safety_margin | 0.15 ~ 0.30 | 接近安全区间上限的正常操作 |
| **L3** | 中风险/接近边界 | PASS | safety_margin | 0.05 ~ 0.15 | 边界操作，建议留有余量 |
| **L4** | 中高风险/临界 | PASS | safety_margin | 0.00 ~ 0.05 | 参数紧贴安全上限，稍有波动即 FAIL |
| **L5** | 高风险/越线 | FAIL | over_ratio | 1.0 ~ 1.5 | 参数轻微超标，降低参数即可安全 |
| **L6** | 危险/严重越线 | FAIL | over_ratio | > 1.5 | 参数严重超标，需要大幅降低或重新规划 |

> **说明**：
> - `safety_margin` = 1 - 最大超限比，取值范围 [0, 1]，仅 PASS 样本有意义
> - `over_ratio` = 最严维度的实际值 / 限值，取值范围 [0, +∞)，PASS 样本为 0.0
> - 原五级 `risk_level` 字段保持不变（LOW/MEDIUM/HIGH），向后兼容
> - 可通过 `SafetyEngineV4.RISK_LEVEL_7_RULES` 类常量程序化访问完整规则定义

---

## 超标倍率（over_ratio）说明

`over_ratio` 是 v4.2.3 新增的 FAIL 样本超标程度指标，表示最严维度的实际值与限值之比。

**计算公式**：
```
over_ratio = max(
    force / force_limit,
    speed / speed_limit,
    pressure / pressure_limit,
    impulse / impulse_limit,      # 仅冲量触发时
    reaction_force / reaction_force_limit,  # 仅反作用约束时
    force / iso_force_limit,      # 仅人体接触时
)
```

**取值含义**：

| over_ratio | 含义 |
|------------|------|
| 0.0 | PASS 样本，未越线 |
| 1.0 | 刚好在边界上 |
| 1.2 | 超出限值 20% |
| 2.0 | 超出限值 100%（即 2 倍） |
| 5.0 | 超出限值 400%（即 5 倍，严重越线） |

> **注意**：`over_ratio` 取的是**最严维度**的倍率。即使只有一个维度严重超标，
> 其他维度正常，over_ratio 也会反映那个最严重的超标程度。

---

## 支持的动作

**JSON 模式核心动作（10+）**：
`grasp`（抓取）、`carry`（搬运）、`push`（推）、`pull`（拉）、
`lift`（抬起）、`place`（放置）、`press`（按压）、`insert`（插入）、
`rotate`（旋转）、`hold`（保持）、`release`（释放）等。

**自然语言模式（中文 35+ 动词）**：
抓态类（抓/拿/取/放/推/拉/拍/打/扔/接/开/关/插/拔/拧/摸/按/捏）+
持态类（握/持/举/端/夹/扶/托/提/压）+
复合类（搬/踩/吸/顶/卡/背/扛/搬运）

> 更多动作可通过扩展动词库和安全规则快速添加。

---

## 支持的物体类型

`rigid`（刚性体）/ `semi_rigid`（半刚性）/ `flexible`（柔性体）/ `fragile`（易碎品）/ `fluid`（液体）/ `human`（人体接触）/ `heavy`（重物）

每种类型有独立的交互规则和安全限值。

---

## ISO 标准

- **ISO 10218-1/2** — 工业机器人安全标准（设计参考）
- **ISO/TS 15066** — 协作机器人技术规范（设计参考）

触发条件：物体类别为 `human`（人体部位）时，自动触发 ISO 力限值校验；
`human_contact` 属性仅用于标注接触态场景参考，不施加 ISO 力约束（避免日常物体过度收紧）。

> 注：本产品为软件中间件，ISO 标准为设计层面的对齐与参考，非第三方认证级合规声明。

---

## 与 VLA 模型的关系

```
  用户指令
     │
     ▼
┌──────────┐     动作+参数      ┌──────────────────┐
│ VLA 模型 │ ────────────────► │ Safety Engine V4 │
│（语义/规划）│                  │  （物理安全守门员） │
└──────────┘ ◄──────────────── └──────────────────┘
     │          PASS/修正建议          │
     ▼                                 │
  动作执行 ◄───────────────────────────┘
```

- VLA 模型负责**理解意图、规划动作**
- Safety Engine 负责**校验动作的物理安全性**
- 两层各司其职，互补不重叠

---

## 部署

### 边缘设备

纯 Python 标准库实现，无外部依赖，可直接部署到：
- 机器人控制器（ARM / x86）
- 边缘计算设备（Jetson、RK3588 等）
- 工控机 / PLC 上位机

### 集成方式

```python
# 方式1：直接嵌入
from safety_engine import SafetyEngineV4
engine = SafetyEngineV4()
result = engine.check_command(action, obj, params)

# 方式2：封装为 HTTP 服务
# 方式3：编译为 C 扩展（通过 Cython / Nuitka）
```

---

## 路线图

- [ ] 更多动词扩展
- [ ] 多物体交互场景支持
- [ ] 连续动作轨迹安全校验
- [ ] ROS / ROS2 集成包
- [ ] C++ / Rust 版本
- [ ] 可视化调试工具

---

## ⚠️ Disclaimer

This project is provided for **research and educational purposes only**. It is NOT a certified safety device, NOT a substitute for professional risk assessment, and NOT intended for use in safety-critical applications without independent validation.

The authors make NO representations or warranties of any kind, express or implied, about the completeness, accuracy, reliability, or suitability of the software. Any use is at your own risk.

Always comply with local safety regulations (ISO 10218, ISO/TS 15066, and applicable national standards) and conduct thorough risk assessments before deploying any robotic system.

---
## License

MIT License — 详见 [LICENSE](LICENSE) 文件。

自由使用，欢迎 Star ⭐
