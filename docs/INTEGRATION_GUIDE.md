# 集成指南

## 集成方式概览

Safety Engine 有三种常见集成方式，根据你的架构选择：

```
方式1：嵌入式调用        方式2：服务化调用         方式3：前置代理
┌─────────────┐        ┌─────────────┐        ┌───────────────────┐
│ VLA + Safety │        │   VLA模型    │        │  Safety Proxy      │
│  同进程调用   │        │             │        │  （独立服务）       │
└─────────────┘        │  HTTP/gRPC  │───────►│  拦截不安全动作     │
                        │   服务      │        └───────────────────┘
                        └─────────────┘                  │
                                                         ▼
                                                    机器人控制器
```

---

## 方式1：嵌入式调用（最简单）

直接 import，在 VLA 推理进程内调用。

```python
from safety_engine import SafetyEngineV4

# 全局初始化一次
safety_engine = SafetyEngineV4()

def execute_action(scene, action, robot):
    # 动作执行前先做安全校验
    result = safety_engine.check_action(scene, action, robot)
    
    if result["verdict"] == "PASS":
        # 安全，执行动作
        robot_controller.execute(action)
        return True
    elif result["verdict"] == "FAIL":
        # 不安全，用推荐参数重试或拒绝
        print(f"动作不安全: {result['correction']}")
        if result.get("recommended_params"):
            print(f"建议参数: {result['recommended_params']}")
        return False
    else:  # REJECT
        # 不支持的动作，拒绝执行
        return False
```

**适用场景**：嵌入式系统、边缘设备、低延迟要求

---

## 方式2：HTTP 服务（分布式部署）

用 FastAPI / Flask 封装成独立服务。

```python
# 示例：FastAPI 封装
from fastapi import FastAPI
from safety_engine import SafetyEngineV4
from pydantic import BaseModel
from typing import List, Optional

app = FastAPI()
engine = SafetyEngineV4()

class ObjectData(BaseModel):
    object_id: str
    name: str
    mass_kg: float
    stability: str
    contact_area_mm2: float

class SceneData(BaseModel):
    objects: List[ObjectData]

class ActionData(BaseModel):
    type: str
    force_n: float
    velocity_ms: float
    acceleration_ms2: float
    target_object: str

class RobotData(BaseModel):
    max_force_n: float
    max_velocity_ms: float
    max_acceleration_ms2: float

@app.post("/check")
def check(scene: SceneData, action: ActionData, robot: RobotData):
    result = engine.check_action(
        scene.model_dump(),
        action.model_dump(),
        robot.model_dump()
    )
    return result
```

启动：
```bash
pip install fastapi uvicorn pydantic
uvicorn server:app --host 0.0.0.0 --port 8000
```

调用：
```bash
curl -X POST http://localhost:8000/check \
  -H "Content-Type: application/json" \
  -d '{...}'
```

**适用场景**：多机器人共享、微服务架构

---

## 方式3：Cython 编译（极致性能）

如果需要更高性能，可以用 Cython 编译为 C 扩展。

性能提升参考：3~5 倍。

```python
# setup_cython.py
from setuptools import setup
from Cython.Build import cythonize

setup(
    ext_modules=cythonize("safety_engine.py", language_level="3"),
)
```

```bash
pip install cython
python setup_cython.py build_ext --inplace
```

**适用场景**：高吞吐量要求、实时控制系统

---

## 最佳实践

### 1. 引擎实例复用

SafetyEngineV4 初始化有一定开销（加载动词库、对象库）。
**全局初始化一次，重复调用。**

✅ 正确：
```python
engine = SafetyEngineV4()  # 全局初始化
for action in actions:
    engine.check_action(...)
```

❌ 错误：
```python
for action in actions:
    engine = SafetyEngineV4()  # 每次都新建，极慢
    engine.check_action(...)
```

### 2. 优先使用 JSON 模式

自然语言模式有语义解析开销（~0.3-0.8ms）。
生产环境请使用 `check_action()` JSON 模式（~16μs）。

### 3. 风险分级用于告警策略

用七级风险分级做差异化处理：

| 等级 | 处理策略 |
|------|---------|
| L0-L2 | 直接执行 |
| L3-L4 | 记录日志，监控告警 |
| L5 | 降低参数后重试 |
| L6 | 立即拒绝，通知上层重新规划 |

### 4. 自定义机器人参数

根据你的机器人实际能力配置：

```python
robot = {
    "max_force_n": 100,        # 根据末端执行器额定力
    "max_velocity_ms": 1.5,    # 根据关节最大速度
    "max_acceleration_ms2": 5.0,  # 根据加减速曲线
    "base_weight_kg": 50,      # 机器人自重（反作用约束用）
    "ground_friction": 0.7,    # 地面摩擦系数
}
```
