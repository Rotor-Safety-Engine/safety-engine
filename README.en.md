# Rotor Safety Engine

> **Real-time Robot Safety Middleware — with Dynamic Contact Area, Impulse Boundary & Reaction Force Stability**
>
> Physical AI safety layer for collaborative robots & humanoids. ISO 10218 / ISO/TS 15066 aligned. 7-Level Risk Granularity · Verb-Object Impossibility guard.
>
> Deterministic Physics · Zero-dependency Python · Sub-millisecond · Edge Inference Ready

**Current Version: v1.0.0 (Community First Release)**

[![Python](https://img.shields.io/badge/Python-3.7%2B-blue)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Tests](https://img.shields.io/badge/tests-349%20passed-brightgreen)]()
[![Mypy](https://img.shields.io/badge/mypy-0%20errors-blue)]()
[![Performance](https://img.shields.io/badge/latency-~17%20μs-blueviolet)]()

---

## What is this

Rotor Safety Engine is a **real-time safety middleware** for collaborative robots and humanoid platforms — a **VLA safety layer** that sits between AI planning and motion execution.
Unlike static threshold checkers, it introduces **Dynamic Contact Area** for soft-object pressure modeling, **Impulse Safety Boundaries** for heavy-load motion control, and **Reaction Force Stability** constraints for mobile manipulator bases.
Aligned with **ISO 10218** and **ISO/TS 15066**, it delivers **Power and Force Limiting (PFL)** with **7-level risk granularity** in pure **zero-dependency Python** — ready for **edge inference** in real-time control loops.

**One-liner: We don't understand your task — we just make sure your action is physically safe.**

---

## Editions

| Feature | Community Edition | Pro Edition |
|---------|------------------|-------------|
| Version | v1.0.0 | v1.1.0+ |
| 4-layer safety verdict | ✅ | ✅ |
| **Dynamic Contact Area** | ✅ | ✅ |
| **Impulse Safety Boundary** | ✅ | ✅ |
| **Reaction Force Stability** | ✅ | ✅ |
| **7-Level Risk Granularity** | ✅ | ✅ |
| Over-ratio metric | ✅ | ✅ |
| Retreat parameter recommendation | ✅ | ✅ |
| Semantic plausibility score | ✅ | ✅ |
| **Verb-Object Impossibility** guard | ✅ | ✅ |
| ISO compliance labeling | ✅ | ✅ |
| **Action-Reaction force pairs (3D vectors)** | — | ✅ |
| **Momentum-Impulse-Time chain analysis** | — | ✅ |
| **Rotation matrix direction vectors** | — | ✅ |
| **Stribeck static & dynamic friction curve** | — | ✅ |
| **Physics-based contact time derivation** | — | ✅ |
| **Energy conservation check (Work = F × d)** | — | ✅ |
| **Pure-function physics analysis API** | — | ✅ |
| **World model physics verification** | — | ✅ |
| License | MIT | Commercial |
| Use cases | R&D, prototyping, education | Production, industrial, world models |

> **Pro Edition inquiries**: contact@rotor-dynamics.ai

---

## Highlights

- ⚡ **Sub-millisecond latency · Real-time safety watchdog** — ~17 μs average in Python, ~58K calls/sec single-thread, built for **edge inference** real-time control loops
- 🎯 **Physical AI · 100% interpretable** — Pure physics inequalities + deterministic rules, **dynamics-based AI** for **humanoid robot safety** and cobot applications, zero neural networks, zero black boxes
- 📦 **Single-file · Zero dependencies** — One Python file, standard library only, drop into any pipeline
- 🏗️ **4-layer safety architecture** — Semantic parsing → Safety adaptation → Action classification → Decision
- 🤖 **35+ Chinese verbs** — grasp / hold / push / pull / insert / rotate / press... NL mode works out of the box
- 📏 **ISO standard reference** — Designed with ISO 10218 / ISO/TS 15066 in mind, human contact auto-labeled
- 🎛️ **Two input modes** — Natural language mode (VLA output) + JSON parameter mode (controller)

### 🔬 Exclusive Technical Features

- **🟢 Dynamic Contact Area** — Real-time contact area and pressure calculation based on force / stiffness, not treated as a constant; distinguishes "bread vs iron block" impact difference
- **🟡 Impulse Safety Boundary** — Momentum (mass × velocity) judgment with separate thresholds for grasp (light impulse) and carry (heavy impulse), prevents high-speed heavy-load tipping
- **🔴 Reaction Force Stability** — Chassis stability integrated into safety verdict (base_weight × g × friction); FAIL if arm force could topple the robot itself, designed for Mobile Manipulators
- **📊 7-Level Risk Granularity** — L0～L6 fine-grained grading + over_ratio metric, beyond the crude traditional Low/Medium/High trichotomy
- **🧠 Verb-Object Impossibility** — Semantic knowledge graph hard-coded common-sense barriers (e.g., grasp + fluid → REJECT), a "common-sense reasoning" first line of defense pure physics engines can't match
- 🔌 **External data config** — Verb DB / object DB / rules loaded from JSON files, no source changes for business customization
- 🛡️ **Robust input validation** — Type / range / NaN checks with graceful degradation and `input_warnings` diagnostic

---

## Architecture

```
Input (action + object + params)
    │
    ▼
┌──────────────────────────┐
│ Layer 1  Semantic Parser  │
│  Verb library · Object DB │
│  Robot capability table   │
│  Safety zone calculation  │
└───────────┬──────────────┘
            │
            ▼
┌──────────────────────────┐
│ Layer 2  Safety Adapter   │
│  Robot capability limits  │
│  Action-target compatibility │
│  Param safety threshold   │
└───────────┬──────────────┘
            │
            ▼
┌──────────────────────────┐
│ Layer 3  Action Classifier│
│  FAV 3D classifier        │
│  Phase detection          │
│  (idle / grasping / hold) │
└───────────┬──────────────┘
            │
            ▼
┌──────────────────────────┐
│ Layer 4  Decision Engine  │
│  PASS / FAIL / REJECT     │
│  Full output + correction │
│  ISO labeling · 7-level risk│
└──────────────────────────┘
```

> Pro Edition adds **Layer 5 — Mechanics Enhancement** (action-reaction / momentum-impulse / energy conservation). See edition comparison.

---

## Quick Start

### Installation

```bash
# Option 1: Install via pip (recommended)
pip install rotor-safety-engine

# Option 2: Install latest from GitHub
pip install git+https://github.com/rotor-dynamics/safety-engine.git

# Option 3: Editable install (for development / contributing)
git clone https://github.com/rotor-dynamics/safety-engine.git
cd safety-engine
pip install -e .

# Option 4: Single file — just copy it
cp src/safety_engine.py your_project/
```

### Basic usage — Natural language mode

```python
from safety_engine import SafetyEngineV4

engine = SafetyEngineV4()

# Normal operation → PASS
result = engine.check_command("grasp", "egg",
                              params={"force": 2.0, "speed": 0.03},
                              robot="humanoid_basic")
print(result["verdict"])      # PASS
print(result["risk_level"])   # LOW
print(result["risk_level_7"]) # L1

# Too much force → FAIL
result = engine.check_command("grasp", "glass panel",
                              params={"force": 80.0, "speed": 0.5},
                              robot="humanoid_basic")
print(result["verdict"])       # FAIL
print(result["correction"])    # Unsafe parameters: ...
print(result["over_ratio"])    # Over-limit ratio
print(result["recommended_params_v2"])  # Safe parameter recommendations
```

### Basic usage — JSON parameter mode (production recommended)

```python
scene = {
    "objects": [{
        "object_id": "metal_part",
        "name": "aluminum workpiece",
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
print(result["verdict"])             # PASS
print(result["pressure_kPa"])        # Contact pressure
print(result["contact_area_mm2"])    # Dynamic contact area
```

### Custom data configuration (v1.0.0+)

Verb database, object database, and action rules can be loaded from external JSON files — no source code changes needed for business customization:

```python
from safety_engine import SafetyEngineV4

# Option 1: Load from JSON files
engine = SafetyEngineV4.from_config(
    verb_db_path="config/verbs.json",
    object_db_path="config/objects.json",
    action_rules_path="config/rules.json",
)

# Option 2: Pass dicts directly
engine = SafetyEngineV4(
    verb_db={"my_verb": {...}},
    action_rules={"my_action": {...}},
)
```

### Run demo

```bash
python examples/demo.py
```

### Run tests

```bash
# Option 1: Direct run (zero dependencies, recommended for quick validation)
python tests/test_engine.py

# Option 2: pytest (requires pytest)
pip install pytest
pytest tests/test_engine.py -v
```

---

## Performance

> Test environment: Python 3.10+ / x86_64 / JSON parameter mode / 100K iterations

| Metric | Value |
|--------|-------|
| Average latency | **~16 μs** (0.016 ms) |
| P95 latency | ~20 μs |
| P99 latency | ~25 μs |
| Single-thread throughput | **~60,000 calls/sec** |
| NL mode latency | 0.3–0.8 ms |
| Memory footprint | < 5 MB |
| External dependencies | 0 (Python stdlib only) |
| Determinism | 100% (same input → same output) |

---

## 7-Level Risk Grading

The engine uses a 7-level risk scale (L0–L6), with different criteria for PASS vs FAIL samples:

- **PASS samples**: Graded by `safety_margin` — higher margin = safer
- **FAIL samples**: Graded by `over_ratio` — higher ratio = more dangerous

| Level | Label | Verdict | Criterion | Range |
|-------|-------|---------|-----------|-------|
| **L0** | Safe | PASS | safety_margin | 0.50 ~ 1.00 |
| **L1** | Low Risk | PASS | safety_margin | 0.30 ~ 0.50 |
| **L2** | Medium-Low | PASS | safety_margin | 0.15 ~ 0.30 |
| **L3** | Medium / Near Boundary | PASS | safety_margin | 0.05 ~ 0.15 |
| **L4** | Medium-High / Critical | PASS | safety_margin | 0.00 ~ 0.05 |
| **L5** | High Risk / Over Limit | FAIL | over_ratio | 1.0 ~ 1.5 |
| **L6** | Dangerous / Severe | FAIL | over_ratio | > 1.5 |

---

## How It Works

### 3-Layer Force Constraint Model

Derived from momentum-energy dual conservation. Force safety is determined by three layers of constraints, taking the strictest:

1. **Output constraint** — Robot output capability limit (motor / joint limit)
2. **Receive constraint** — Object mechanical response limit (material / structure, including dynamic contact area, impulse, pressure)
3. **Bidirectional constraint** — Reaction force & body stability (Newton's 3rd law, robot base friction limit)

### Dynamic Contact Area

Soft / fragile objects' contact area grows under force, pressure adjusts dynamically:
- Rigid (metal, stone): High stiffness, area nearly constant
- Soft (bread, fruit): Low stiffness, area grows linearly with force
- Fragile (egg, glass): Minimal deformation but shatters at high pressure — judged by pressure threshold

### Impulse Safety Check

For motion actions (carry / move / push / pull / lift), adds `impulse = mass × velocity` safety constraint:
- Fragile / liquid: Low impulse limit (prevents breaking / spilling)
- Heavy / rigid: High impulse limit
- Human contact: Tightened impulse limit (avoids high-speed collision)

### Reaction Force Constraint

The force the robot applies to an object equals the reaction force from the object back to the robot.
When reaction force exceeds the maximum static friction the robot base can provide, the robot becomes unstable.

```
max_reaction = base_weight_kg × G × friction_coef
```

---

## Supported Actions & Object Types

**JSON mode core actions (10+)**:
`grasp` / `carry` / `push` / `pull` / `lift` / `place` / `press` /
`insert` / `rotate` / `hold` / `release` and more.

**Natural language mode (35+ Chinese verbs)**:
Grasp-type + hold-type + compound-type.

**Object types (7)**:
`rigid` / `semi_rigid` / `flexible` /
`fragile` / `fluid` / `human` / `heavy`

---

## ISO Standards Reference

- **ISO 10218-1/2** — Industrial robot safety standard (design reference)
- **ISO/TS 15066** — Collaborative robot technical specification (design reference)

> Note: This product is a software middleware. ISO standards are design-level alignment and reference, not third-party certified compliance.

---

## Relationship with VLA Models (VLA Safety Layer)

```
  User command
     │
     ▼
┌──────────┐   action + params    ┌──────────────────┐
│ VLA Model│ ──────────────────► │ Safety Engine V4 │
│(semantic/│                     │  (physics gate)   │
│ planning)│ ◄────────────────── └──────────────────┘
     │       PASS / correction         │
     ▼                                 │
  Execution ◄──────────────────────────┘
```

- VLA models handle **intent understanding & action planning**
- Safety Engine handles **physical safety validation**
- Two layers work in parallel, complementary not overlapping

---

## Relationship with Google Gemini Robotics ER 2 (VLA Safety Comparison)

VLA safety is the core challenge in embodied AI deployment.(https://deepmind.google/) is one of the most capable VLA models in embodied AI today, representing state-of-the-art in semantic reasoning and task planning. We do not compete with ER 2 at the same layer — we are **complementary**: ER 2 makes task-level decisions, we provide action-level physical safety checks.

| Dimension | Google Gemini Robotics ER 2 | Rotor Safety Engine |
|-----------|----------------------------|---------------------|
| Role | Embodied reasoning "brain": task planning, progress tracking, error recovery | Physical safety "reflex arc": real-time action-level interception |
| Method | Neural network probabilistic reasoning | Deterministic physics inequalities |
| Latency | Sub-second (MAE ~0.96s) | ~17μs (P99 < 100μs) |
| Deployment | Cloud API (Gemini Live) | Edge / on-premise, zero network dependency |
| Safety guarantee | Probabilistic (may miss edge cases) | Zero misses (physical constraints are hard bounds) |
| Best for | Understanding user intent, planning complex tasks, multi-robot coordination | Ensuring every physical action is safe and reliable |

> **Core insight**: No matter how smart a robot's "brain" is, it needs a 100% reliable safety reflex arc.
> VLA model outputs action → Safety Engine does final safety validation → Execution.
> ER 2 is the company's CEO (making decisions); we're the safety officer (with veto power).

**Value for ER 2 users**: If your robot runs on ER 2 or any similar VLA model, integrating Safety Engine gives you a deterministic physical safety layer without changing the upstream model — addressing the inherent limitations of probabilistic reasoning in safety-critical scenarios.

---

## Deployment

### Edge Devices

Pure Python standard library, no external dependencies. Deploy directly to:
- Robot controllers (ARM / x86)
- Edge computing (Jetson, RK3588, etc.)
- Industrial PCs / PLC upper computers

### Integration

```python
# Option 1: Embed directly
from safety_engine import SafetyEngineV4
engine = SafetyEngineV4()
result = engine.check_command(action, obj, params)

# Option 2: Wrap as HTTP service
# Option 3: Compile as C extension (Cython / Nuitka)
```

---

## Roadmap

---

## API Reference

### SafetyEngineV4

Main engine class with two input modes.

#### `__init__(self, verb_db=None, object_db=None, action_rules=None, rules=None)`

Initialize the engine. All parameters are optional — built-in defaults are used when not provided.

| Parameter | Type | Description |
|-----------|------|-------------|
| `verb_db` | `Optional[Dict]` | Custom verb database dict |
| `object_db` | `Optional[Dict]` | Custom object property database dict |
| `action_rules` | `Optional[Dict]` | Custom action rule table dict |
| `rules` | `Optional[SafetyRules]` | Custom global safety rules |

#### `from_config(verb_db_path=None, object_db_path=None, action_rules_path=None, rules=None)`

Class method — load configuration from JSON files and create an engine instance.

```python
engine = SafetyEngineV4.from_config(
    verb_db_path="config/verbs.json",
    action_rules_path="config/rules.json",
)
```

#### `check_command(self, action, obj, params=None, robot=None, object_params=None, context=None) -> Dict`

Natural language mode input.

| Parameter | Type | Description |
|-----------|------|-------------|
| `action` | `str` | Action name (Chinese verb, e.g. "抓取"/"推") |
| `obj` | `str` | Target object name |
| `params` | `Optional[Dict]` | Action params `{"force": N, "speed": m/s, ...}` |
| `robot` | `Optional[str / Dict]` | Robot config: string model name or custom capability dict |
| `object_params` | `Optional[Dict]` | Custom object properties (overrides built-in DB) |
| `context` | `Optional[Dict]` | Context `{"near_human": bool, "fragile": bool, "semantic_score": float}` |

Returns: V4Result dict with 20+ fields including verdict / risk_level / risk_level_7 / over_ratio / input_warnings.

#### `check_action(self, scene_data, action_data, robot_data) -> Dict`

JSON parameter mode input (production recommended, best performance).

| Parameter | Type | Description |
|-----------|------|-------------|
| `scene_data` | `Dict` | Scene data, contains `objects` list |
| `action_data` | `Dict` | Action data: `type` / `force_n` / `velocity_ms` / `target_object` |
| `robot_data` | `Dict` | Robot capability data |

### Input Validation (v1.0.0+)

The engine automatically performs input validation at the entry point:

| Condition | Handling |
|-----------|----------|
| Type error (e.g. params is not a dict) | `REJECT` + explanation |
| Null / None key parameters | `REJECT` + explanation |
| NaN / Infinity values | `REJECT` + explanation |
| Force > 10000N / speed > 100m/s out of range | `REJECT` + explanation |
| Negative force / speed | Auto absolute value + `input_warnings` entry |
| Missing optional parameters | Default values + `input_warnings` entry |

`input_warnings` is a list of strings, always output (empty list when clean), useful for input diagnostics.

---

## Configuration Guide

### External JSON Configuration File Formats

#### Verb Database (verbs.json)

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

#### Object Database (objects.json)

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

#### Rule Table (rules.json)

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

> For complete field definitions, refer to the `VERB_DATABASE` / `OBJECT_PROPERTIES` / `ACTION_RULES` structures in the source code.

## Roadmap

- [x] v4.0 — Core 4-layer architecture
- [x] v4.1 — Evaluation improvements & performance optimization
- [x] v4.2 — Physics 3-layer structure upgrade (dynamic contact area + impulse + reaction force)
- [x] v4.2.x — Quality fixes & 7-level risk grading
- [x] v1.0.0 — First public release: quality enhancements: typing / external config / input validation / common logic extraction
- [ ] More verb extensions
- [ ] Multi-object interaction support
- [ ] Continuous motion trajectory safety validation
- [ ] ROS / ROS2 integration package
- [ ] C++ / Rust version
- [ ] **Pro Edition Layer 5 Mechanics Enhancement** (commercial license)

---

## Development

### Running Tests

```bash
pip install pytest pytest-cov
pytest tests/ -v --cov=safety_engine
```

### Type Checking

The codebase passes strict mypy type checking:

```bash
pip install mypy
mypy src/safety_engine.py --ignore-missing-imports
```

### CI/CD

GitHub Actions runs the full test suite on every push and pull request across Python 3.8–3.12. See `.github/workflows/pytest.yml` for details.

---

## Keyword Index

> Search-engine friendly keyword coverage — trending terms + deep tech terms + exclusive identifiers

**🔥 Trending Topics**
`Embodied AI` · `Physical AI` · `Humanoid Robot Safety` · `Collaborative Robot` · `Cobot Safety` · `VLA Safety Layer` · `Real-time Middleware` · `Safety Watchdog` · `Edge Inference` · `Robot Middleware` · `Zero-dependency Python` · `Python Safety Library` · `Deterministic Safety`

**🏛️ Standards & Compliance**
`ISO 10218` · `ISO/TS 15066` · `TS 15066` · `Industrial Robot Safety` · `Human-Robot Collaboration` · `HRC` · `Power and Force Limiting` · `PFL` · `Speed and Separation Monitoring` · `SSM` · `Safety-Rated Monitored Stop`

**🔬 Core Technology**
`Force-Velocity-Amplitude Constraint Checking` · `Force-Velocity Envelope` · `Dynamics-based AI` · `Semantic Parsing for Robotics` · `FAV Classifier` · `Quasi-Static Force Limits` · `Reaction Force Constraint`

**🏆 Exclusive Features**
`Dynamic Contact Area` · `Impulse Safety Boundary` · `Reaction Force Stability` · `7-Level Risk Granularity` · `Verb-Object Impossibility` · `Over-Ratio Metric`

---

## ⚠️ Safety Disclaimer (Important)

Rotor Safety Engine Community Edition is a **research-grade open-source middleware** intended for technical reference and educational purposes. It is **not a certified functional safety product** and has not been officially certified against ISO 13482, ISO 10218, IEC 61508, or any other safety standard.

- **For R&D and prototyping only.** Do not use in production environments, human-in-the-loop systems, or any scenario where human safety may be at risk.
- Anyone deploying, modifying, or commercializing this software assumes full safety responsibility and must engage qualified third-party institutions for required safety certification and risk assessment.
- This software is provided "AS IS" without warranty of any kind, including implied warranties of merchantability, fitness for a particular purpose, and non-infringement.
- In no event shall the authors or contributors be liable for any claim, damages, or other liability arising from the use of this software.

For production deployments, contact us at `contact@rotor-dynamics.ai` for enterprise licensing.

---

## License

MIT License — see [LICENSE](LICENSE) file.

Free to use. Stars welcome ⭐
