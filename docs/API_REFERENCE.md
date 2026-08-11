# API 参考

## SafetyEngineV4

主引擎类。创建实例后即可使用。

```python
from safety_engine import SafetyEngineV4

engine = SafetyEngineV4()
```

### 方法

#### `check_action(scene_data, action_data, robot_data) -> dict`

JSON 参数模式，生产环境推荐。零解析开销，延迟最低。

**参数：**

| 参数 | 类型 | 说明 |
|------|------|------|
| scene_data | dict | 场景数据 |
| action_data | dict | 动作数据 |
| robot_data | dict | 机器人能力数据 |

**scene_data 结构：**

| 字段 | 类型 | 说明 |
|------|------|------|
| objects | list | 物体列表 |

每个 object 包含：
- `object_id` (string) — 物体标识
- `name` (string) — 物体名称
- `mass_kg` (float) — 质量（kg）
- `stability` (string) — 材质类型：rigid/semi_rigid/flexible/fragile/fluid/human/heavy
- `contact_area_mm2` (float) — 接触面积（mm²）

**action_data 结构：**

| 字段 | 类型 | 说明 |
|------|------|------|
| type | string | 动作类型 |
| force_n | float | 作用力（N） |
| velocity_ms | float | 速度（m/s） |
| acceleration_ms2 | float | 加速度（m/s²） |
| target_object | string | 目标物体ID |

**robot_data 结构：**

| 字段 | 类型 | 说明 |
|------|------|------|
| max_force_n | float | 最大作用力（N） |
| max_velocity_ms | float | 最大速度（m/s） |
| max_acceleration_ms2 | float | 最大加速度（m/s²） |

---

#### `check_command(action, obj, params=None, robot=None, context=None) -> dict`

自然语言模式，快速原型和调试用。

**参数：**

| 参数 | 类型 | 说明 |
|------|------|------|
| action | string | 中文动词（如"抓取""推""抬起"） |
| obj | string | 物体名称（如"鸡蛋""纸箱"） |
| params | dict | 动作参数（force/speed/angle等） |
| robot | string/dict | 机器人型号或能力字典 |
| context | dict | 扩展上下文参数 |

---

### 返回值字段

| 字段 | 类型 | 说明 |
|------|------|------|
| verdict | string | 判定：PASS / FAIL / REJECT |
| state | string | 动作状态：idle / grasp / hold |
| risk_level | string | 风险等级：LOW / MEDIUM / HIGH |
| risk_level_7 | string | 七级风险：L0 ~ L6 |
| risk_subtypes | list | 风险子类型标签 |
| latency_ms | float | 判定耗时（ms） |
| pressure_kPa | float | 接触压强（kPa） |
| contact_area_mm2 | float | 动态接触面积（mm²） |
| over_ratio | float | 超标倍率（FAIL样本） |
| correction | string | 判定原因 / 修正建议 |
| recommended_params | dict | 推荐安全参数 |
| retreat_params | dict/null | 回退参数（边界PASS样本） |
| semantic_plausibility_score | float/null | 语义合理性分数 |
| iso_compliance | string | ISO合规标注 |
| capability_match | dict | 能力适配结果 |
| action_target_match | dict | 动作-目标适配结果 |
| param_check | dict | 参数安全校验结果 |
| safety_zone | dict | 安全区间 |
| disturbance | dict | 干扰等级评估 |
