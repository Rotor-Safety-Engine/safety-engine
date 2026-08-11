# Rotor Safety Engine — 面向具身智能的物理安全实时判定引擎

> **Real-time Robot Safety Middleware — with Dynamic Contact Area, Impulse Boundary & Reaction Force Stability**
>
> 面向协作机器人与人形机器人的 Physical AI 安全层。ISO 10218 / ISO/TS 15066 对齐，七级风险粒度 · 动宾不可能组合校验。
>
> 纯确定性物理 · 动力学驱动 · 单文件零依赖 · 亚毫秒级 · 边缘推理就绪

[![Python](https://img.shields.io/badge/Python-3.7%2B-blue)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Tests](https://img.shields.io/badge/tests-349%20passed-brightgreen)]()
[![Mypy](https://img.shields.io/badge/mypy-0%20errors-blue)]()
[![Performance](https://img.shields.io/badge/latency-~17%20μs-blueviolet)]()

**当前版本：v1.0.0（社区首发版）**

---

## 这是什么

Rotor Safety Engine 是一款面向协作机器人与人形机器人的 **实时安全中间件（real-time safety middleware）**，也是 VLA 推理管线中的 **VLA safety layer**，连接 AI 规划与运动执行。
不同于静态阈值检查器，它引入了 **动态接触面积（Dynamic Contact Area）** 用于软物压强建模、**冲量安全边界（Impulse Safety Boundary）** 用于重物运动控制、以及 **反作用力稳定性（Reaction Force Stability）** 约束用于移动操作机器人底盘稳定性判定。
设计对齐 **ISO 10218** 与 **ISO/TS 15066** 安全标准，提供 **Power and Force Limiting (PFL)** 力与功率限制能力，搭配 **七级风险粒度（7-level risk granularity）**，纯 **零依赖 Python（zero-dependency Python）** 实现，适合 **边缘推理（edge inference）** 实时控制场景。

**一句话定位：不理解你的任务，只保证你的动作在物理上是安全的。**

---

## 版本选择

| 功能 | 社区版（Community） | 企业版（Pro） |
|------|-------------------|--------------|
| 版本 | v1.0.0 | v1.1.0+ |
| 四层安全判定 | ✅ | ✅ |
| **动态接触面积（Dynamic Contact Area）** | ✅ | ✅ |
| **冲量安全边界（Impulse Safety Boundary）** | ✅ | ✅ |
| **反作用力稳定性（Reaction Force Stability）** | ✅ | ✅ |
| **七级风险粒度（7-Level Risk Granularity）** | ✅ | ✅ |
| 超标倍率（over_ratio） | ✅ | ✅ |
| 回退参数推荐（retreat_params） | ✅ | ✅ |
| 语义合理性分（semantic_plausibility） | ✅ | ✅ |
| **动-宾不可能组合（Verb-Object Impossibility）** | ✅ | ✅ |
| ISO 合规标注 | ✅ | ✅ |
| **作用-反作用完整对（3D向量）** | — | ✅ |
| **动量-冲量-时间链路分析** | — | ✅ |
| **旋转矩阵方向向量（rotation_matrix）** | — | ✅ |
| **Stribeck 静动摩擦曲线** | — | ✅ |
| **接触时间物理推导** | — | ✅ |
| **能量守恒校验（Work = F × d）** | — | ✅ |
| **纯函数力学分析接口（pure_analyze）** | — | ✅ |
| **世界模型力学校验能力** | — | ✅ |
| 开源协议 | MIT | 商业授权 |
| 适用场景 | 研发、原型、教育 | 生产、工业、世界模型 |

> **企业版咨询**：contact@rotor-dynamics.ai

---

## 核心特性（社区版）

- ⚡ **亚毫秒级延迟 · 实时安全看门狗** — Python 版平均 ~17μs，单线程吞吐量 ~6 万次/秒，**边缘推理**实时控制场景就绪
- 🎯 **Physical AI · 100% 可解释** — 纯物理不等式 + 确定性规则，**动力学驱动**，面向人形机器人与协作机器人安全，零神经网络，零黑盒
- 📦 **单文件 · 零依赖** — 一个 Python 文件，仅用标准库，直接嵌入任何管线
- 🏗️ **四层安全架构** — 语义解析 → 安全适配 → 动作分类 → 综合决策
- 🤖 **35+ 中文动词支持** — 抓取/握持/推拉/插拔/拧转/按压……自然语言模式开箱即用
- 📏 **ISO 标准参考** — 设计对齐 ISO 10218 / ISO/TS 15066，人体接触场景自动标注
- 🎛️ **两种输入模式** — 自然语言模式（接 VLA 输出）+ JSON 参数模式（接控制器）

### 🔬 独家技术亮点

- **🟢 动态接触面积（Dynamic Contact Area）** — 根据 force / stiffness 实时计算接触面积与压强，而非将面积视为常量；可区分"面包压手"与"铁块砸手"的本质差异
- **🟡 冲量安全边界（Impulse Safety Boundary）** — 引入动量（mass × velocity）判定，区分 grasp（轻冲量）与 carry（重冲量）阈值，防止高速移重物侧翻
- **🔴 反作用力稳定性（Reaction Force Stability）** — 将底盘稳定性纳入安全判定（base_weight × g × friction），机械臂力过大可能推倒自身时直接判 FAIL，适配移动操作机器人（Mobile Manipulator）
- **📊 七级风险粒度（7-Level Risk Granularity）** — L0～L6 细粒度分级 + over_ratio（超标倍率），破除传统 Low/Medium/High 三档的粗糙划分
- **🧠 动-宾不可能组合（Verb-Object Impossibility）** — 语义知识图谱硬编码常识壁垒（如 grasp + fluid → REJECT），纯物理引擎做不到的"常识推理"第一道防线
- 🔌 **外部数据配置** — 动词库/物体库/规则表支持 JSON 文件加载，业务定制无需改源码
- 🛡️ **健壮输入校验** — 类型/范围/NaN 全面校验，非法输入优雅降级，附带 input_warnings 诊断

---

## 四层安全架构

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
│  ISO 合规标注 · 七级风险  │
└──────────────────────────┘
```

> 企业版额外提供 **Layer 5 力学增强层**（作用-反作用 / 动量-冲量 / 能量守恒），详见版本对比。

---

## 快速开始

### 安装

```bash
# 方式 1：pip 安装（推荐）
pip install rotor-safety-engine

# 方式 2：从 GitHub 安装最新版
pip install git+https://github.com/rotor-dynamics/safety-engine.git

# 方式 3：可编辑模式安装（开发/贡献代码用）
git clone https://github.com/rotor-dynamics/safety-engine.git
cd safety-engine
pip install -e .

# 方式 4：单文件，直接拷走即可
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
print(result["risk_level_7"]) # L1

# 力太大 → FAIL
result = engine.check_command("抓", "玻璃面板",
                              params={"force": 80.0, "speed": 0.5},
                              robot="humanoid_basic")
print(result["verdict"])       # FAIL
print(result["correction"])    # 参数不安全: ...
print(result["over_ratio"])    # 超标倍率
print(result["recommended_params_v2"])  # 推荐安全参数
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
print(result["contact_area_mm2"])  # 动态接触面积
```

### 自定义数据配置（v1.0.0+）

动词库、物体库、动作规则表支持从外部 JSON 文件加载，业务定制无需改源码：

```python
from safety_engine import SafetyEngineV4

# 方式 1：从 JSON 文件加载
engine = SafetyEngineV4.from_config(
    verb_db_path="config/verbs.json",
    object_db_path="config/objects.json",
    action_rules_path="config/rules.json",
)

# 方式 2：直接传入字典
engine = SafetyEngineV4(
    verb_db={"my_verb": {...}},
    action_rules={"my_action": {...}},
)
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
```

---

## 性能指标

> 测试环境：Python 3.10+ / x86_64 / JSON 参数模式 / 10 万次循环

| 指标 | 数值 |
|------|------|
| 平均延迟 | **~17 μs**（0.017 ms） |
| P95 延迟 | ~22 μs |
| P99 延迟 | ~27 μs |
| 单线程吞吐量 | **~58,000 次/秒** |
| 自然语言模式延迟 | 0.3–0.8 ms |
| 内存占用 | < 5 MB |
| 外部依赖 | 0（仅 Python 标准库） |
| 确定性 | 100%（相同输入永远相同输出） |

---

## 七级风险分级规则

引擎采用七级风险等级（L0~L6），对 PASS 和 FAIL 样本分别使用不同的划分依据：

- **PASS 样本**：按 `safety_margin`（安全裕度）分档，裕度越高越安全
- **FAIL 样本**：按 `over_ratio`（超标倍率）分档，倍率越大越危险

| 等级 | 中文标签 | 判定 | 划分依据 | 范围 |
|------|----------|------|----------|------|
| **L0** | 安全 | PASS | safety_margin | 0.50 ~ 1.00 |
| **L1** | 低风险 | PASS | safety_margin | 0.30 ~ 0.50 |
| **L2** | 中低风险 | PASS | safety_margin | 0.15 ~ 0.30 |
| **L3** | 中风险/接近边界 | PASS | safety_margin | 0.05 ~ 0.15 |
| **L4** | 中高风险/临界 | PASS | safety_margin | 0.00 ~ 0.05 |
| **L5** | 高风险/越线 | FAIL | over_ratio | 1.0 ~ 1.5 |
| **L6** | 危险/严重越线 | FAIL | over_ratio | > 1.5 |

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
- 软物（面包、水果）：刚度小，接触面积随力线性增长
- 易碎物（鸡蛋、玻璃）：变形极小但压强大了直接碎，以压强阈值判定

### 冲量安全校验

对 carry / move / push / pull / lift 等有位移的动作，增加 `冲量 = 质量 × 速度` 的安全约束：
- 易碎物 / 液体：冲量上限低
- 重物 / 刚体：冲量上限高
- 人体接触：冲量上限收紧

### 反作用力约束

机器人对物体施加的力，等于物体对机器人的反作用力。
当反作用力超过机器人基座能提供的最大静摩擦力时，机器人会失稳。

```
max_reaction = base_weight_kg × G × friction_coef
```

---

## 支持的动作 & 物体类型

**JSON 模式核心动作（10+）**：
`grasp` / `carry` / `push` / `pull` / `lift` / `place` / `press` /
`insert` / `rotate` / `hold` / `release` 等

**自然语言模式（中文 35+ 动词）**：
抓态类 + 持态类 + 复合类

**物体类型（7种）**：
`rigid`（刚性）/ `semi_rigid`（半刚性）/ `flexible`（柔性）/
`fragile`（易碎）/ `fluid`（液体）/ `human`（人体）/ `heavy`（重物）

---

## ISO 标准参考

- **ISO 10218-1/2** — 工业机器人安全标准（设计参考）
- **ISO/TS 15066** — 协作机器人技术规范（设计参考）

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

## 与 Google Gemini Robotics ER 2 的关系

[Google Gemini Robotics ER 2](https://deepmind.google/) 是当前具身智能领域最受关注的 VLA 模型之一，代表了语义推理和任务规划的最高水平。我们和 ER 2 不在同一层竞争——而是**互补关系**：ER 2 做任务级决策，我们做动作级物理安全校验。

| 维度 | Google Gemini Robotics ER 2 | Rotor Safety Engine |
|------|----------------------------|---------------------|
| 定位 | 具身推理「大脑」：任务规划、进度追踪、错误恢复 | 物理安全「反射弧」：动作级实时拦截 |
| 判断方式 | 神经网络概率推理 | 确定性物理不等式 |
| 延迟 | 亚秒级（MAE ~0.96s） | ~17μs（P99 < 100μs） |
| 部署 | 云端 API（Gemini Live） | 边缘/本地，零网络依赖 |
| 安全保证 | 概率性判断（可能漏判） | 零漏判（物理约束是硬性的） |
| 适合做什么 | 理解用户意图、规划复杂任务、协调多机器人 | 确保每一个物理动作安全可靠 |

> **核心观点**：无论机器人的"大脑"多聪明，都需要一个 100% 可靠的安全反射弧。
> VLA 模型输出动作 → Safety Engine 做最后一道安全校验 → 执行。
> ER 2 是公司的 CEO（做决策），我们是公司的安全官（一票否决权）。

**对 ER 2 用户的价值**：如果你的机器人运行在 ER 2 或类似 VLA 模型之上，接入 Safety Engine 可以在不改变上层模型的前提下，获得确定性的物理安全底层——解决概率推理在安全关键场景的固有局限。

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
# 方式3：编译为 C 扩展（Cython / Nuitka）
```

---

## API 参考

### SafetyEngineV4

主引擎类，提供两种输入模式。

#### `__init__(self, verb_db=None, object_db=None, action_rules=None, rules=None)`

初始化引擎。所有参数均可选，不传则使用内置默认值。

| 参数 | 类型 | 说明 |
|------|------|------|
| `verb_db` | `Optional[Dict]` | 自定义动词库字典 |
| `object_db` | `Optional[Dict]` | 自定义物体属性库字典 |
| `action_rules` | `Optional[Dict]` | 自定义动作规则表字典 |
| `rules` | `Optional[SafetyRules]` | 自定义全局安全规则 |

#### `from_config(verb_db_path=None, object_db_path=None, action_rules_path=None, rules=None)`

类方法，从 JSON 文件加载配置数据并创建引擎实例。

```python
engine = SafetyEngineV4.from_config(
    verb_db_path="config/verbs.json",
    action_rules_path="config/rules.json",
)
```

#### `check_command(self, action, obj, params=None, robot=None, object_params=None, context=None) -> Dict`

自然语言模式输入。

| 参数 | 类型 | 说明 |
|------|------|------|
| `action` | `str` | 动作名称（中文动词，如"抓取"/"推"） |
| `obj` | `str` | 目标物体名称 |
| `params` | `Optional[Dict]` | 动作参数 `{"force": N, "speed": m/s, ...}` |
| `robot` | `Optional[str / Dict]` | 机器人配置：字符串机型名或自定义能力字典 |
| `object_params` | `Optional[Dict]` | 自定义物体属性（覆盖内置物体库） |
| `context` | `Optional[Dict]` | 上下文 `{"near_human": bool, "fragile": bool, "semantic_score": float}` |

返回：V4Result 字典，含 verdict / risk_level / risk_level_7 / over_ratio / input_warnings 等 20+ 字段。

#### `check_action(self, scene_data, action_data, robot_data) -> Dict`

JSON 参数模式输入（生产推荐，性能最优）。

| 参数 | 类型 | 说明 |
|------|------|------|
| `scene_data` | `Dict` | 场景数据，含 `objects` 列表 |
| `action_data` | `Dict` | 动作数据，含 `type` / `force_n` / `velocity_ms` / `target_object` 等 |
| `robot_data` | `Dict` | 机器人能力数据 |

### 输入校验（v1.0.0+）

引擎在入口处自动进行输入有效性检查：

| 情况 | 处理方式 |
|------|----------|
| 类型错误（如 params 不是 dict） | `REJECT` + 说明 |
| 空值 / None 关键参数 | `REJECT` + 说明 |
| 数值为 NaN / 无穷大 | `REJECT` + 说明 |
| force > 10000N / speed > 100m/s 明显越界 | `REJECT` + 说明 |
| force / speed 为负数 | 自动取绝对值 + `input_warnings` 记录 |
| 缺失可选参数 | 使用默认值 + `input_warnings` 记录 |

`input_warnings` 为字符串列表，始终输出（空列表也输出），方便排查输入问题。

---

## 配置指南

### 外部 JSON 配置文件格式

#### 动词库（verbs.json）

```json
{
  "my_action": {
    "phase_type": "grasp",
    "risk_level": 3,
    "grasp_params": {
      "max_force": 50,
      "max_speed": 200,
      "max_acceleration": 500
    },
    "hold_params": {
      "min_force": 5,
      "max_force": 50,
      "max_displacement": 3
    }
  }
}
```

#### 物体库（objects.json）

```json
{
  "my_object": {
    "category": "rigid",
    "fragile": 0.2,
    "mass_kg": 1.5,
    "contact_area_mm2": 400,
    "contact_stiffness": 3.0,
    "max_deform": 0.1
  }
}
```

#### 规则表（rules.json）

```json
{
  "rigid": {
    "force_limit": 80,
    "speed_limit": 2.0,
    "pressure_limit_kpa": 500,
    "impulse_max": 10.0
  }
}
```

> 完整字段定义请参考源码中 `VERB_DATABASE` / `OBJECT_PROPERTIES` / `ACTION_RULES` 的结构。

---

## 路线图

- [x] v4.0 基础四层架构
- [x] v4.1 评估改进与性能优化
- [x] v4.2 物理三层结构升级（动态接触面积 + 冲量 + 反作用力）
- [x] v4.2.x 质量修复与七级风险分级
- [x] v1.0.0 首发：质量增强：typing / 外部配置 / 输入校验 / 公共逻辑抽取
- [ ] 更多动词扩展
- [ ] 多物体交互场景支持
- [ ] 连续动作轨迹安全校验
- [ ] ROS / ROS2 集成包
- [ ] C++ / Rust 版本
- [ ] **企业版 Layer 5 力学增强层**（商业授权）

---

## 开发指南

### 运行测试

```bash
pip install pytest pytest-cov
pytest tests/ -v --cov=safety_engine
```

### 类型检查

代码通过严格的 mypy 类型检查：

```bash
pip install mypy
mypy src/safety_engine.py --ignore-missing-imports
```

### CI/CD

每次 push 和 PR 都会触发 GitHub Actions，在 Python 3.8–3.12 全版本运行完整测试。详见 `.github/workflows/pytest.yml`。

---

## 关键词索引

> 便于搜索引擎索引 · 覆盖风口热词 + 技术深度词 + 独家标识符

**🔥 风口热词**
`Embodied AI` · `具身智能` · `Physical AI` · `物理人工智能` · `Humanoid Robot Safety` · `人形机器人安全` · `Collaborative Robot` · `Cobot` · `协作机器人安全` · `VLA Safety Layer` · `Real-time Safety Middleware` · `实时安全中间件` · `Safety Watchdog` · `安全看门狗` · `Edge Inference` · `边缘推理` · `Robot Middleware` · `机器人中间件` · `Python Safety Library` · `Python安全库` · `Deterministic Safety` · `确定性安全` · `Zero-dependency Python`

**🏛️ 标准合规**
`ISO 10218` · `ISO/TS 15066` · `TS 15066` · `工业机器人安全标准` · `人机协作安全规范` · `Human-Robot Collaboration` · `HRC` · `Power and Force Limiting` · `PFL` · `力与功率限制` · `Speed and Separation Monitoring` · `SSM` · `速度与分离监控` · `Safety-Rated Monitored Stop` · `安全等级监控停止`

**🔬 核心技术**
`Force-Velocity-Amplitude Constraint Checking` · `力-速度-幅度三维约束` · `Dynamics-based AI` · `基于动力学的AI判定` · `Semantic Parsing for Robotics` · `机器人语义解析` · `FAV Classifier` · `力-幅值-速度三维状态分类器` · `Quasi-Static Force Limits` · `准静态力限值` · `Reaction Force Constraint` · `反作用力约束` · `Force-Velocity Envelope` · `力-速包络线安全算法`

**🏆 独家技术**
`Dynamic Contact Area` · `动态接触面积` · `Impulse Safety Boundary` · `冲量安全边界` · `Reaction Force Stability` · `反作用力稳定性` · `7-Level Risk Granularity` · `七级风险粒度` · `Verb-Object Impossibility` · `动-宾不可能组合` · `Over-Ratio Metric` · `超标倍率`

---

## ⚠️ 安全免责声明（重要）

本软件（Rotor Safety Engine Community Edition）是一个**研究性质的开源中间件**，旨在为机器人安全领域提供参考实现与技术交流。它**不等同于经过认证的功能安全产品**，也未通过 ISO 13482 / ISO 10218 / IEC 61508 等任何安全标准的官方认证。

- **仅限研发与原型验证使用**，不得直接用于生产环境、载人系统或任何可能危及人身安全的场景
- 任何基于本软件进行的部署、改造或商业产品化，使用者需自行承担全部安全责任，并应委托具备资质的第三方机构完成必要的安全认证与风险评估
- 本软件按 "AS IS" 提供，不提供任何形式的担保，包括但不限于适销性、特定用途适用性和非侵权性的默示担保
- 在任何情况下，作者及贡献者均不对因使用本软件而产生的任何索赔、损害或其他责任负责

如需在生产环境使用，请通过 `contact@rotor-dynamics.ai` 联系企业版咨询。

---

## License

MIT License — 详见 [LICENSE](LICENSE) 文件。

自由使用，欢迎 Star ⭐
