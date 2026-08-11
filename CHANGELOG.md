# 更新日志

所有重要的版本更新都会记录在这个文件中。

格式基于 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，
版本号遵循 [语义化版本](https://semver.org/lang/zh-CN/)。

> **关于版本号的说明**：本项目于 2026年8月首次在 GitHub 公开开源。
> CHANGELOG 中记录的 v4.0.0~v4.2.3 为项目内部研发期间的迭代版本，
> 对应的早期日期为内部版本发布时间，不代表彼时已开源。
> 首个公开开源版本为 **v4.3.0（2026-08-11）**。

## [4.3.0] - 2026-08-11

### 🏁 首个公开开源版本

### 新增
- 完整 Typing 类型注解覆盖，mypy 零错误
- 公共逻辑抽取：`_build_result` 私有方法消除重复代码
- 输入参数校验：类型/范围/NaN/越界全面检查，新增 `input_warnings` 字段
- 外部 JSON 配置：`load_verb_database` / `load_object_database` / `load_action_rules` + `from_config` 类方法
- `SemanticParser` / `SafetyAdapter` 实例属性化，支持多实例独立配置
- `__version__` / `__author__` 模块级属性
- GitHub Actions CI（Python 3.8~3.12 矩阵测试）
- 中英文 Issue / PR 模板

### 工程化
- PyPI 包发布（`pip install rotor-safety-engine`）
- 单文件零依赖，可拷贝直接使用或 editable mode 安装
- 349 项自动化测试全通过

### 文档
- 中英文 README 全面重写
- API_REFERENCE / FAQ / INTEGRATION_GUIDE 三套文档

## [4.2.3] - 2026-08-05

### 新增
- `over_ratio` 超标倍率字段，表示FAIL样本最严维度的实际值与限值之比
- 边界PASS样本 `retreat_params` 安全回退参数推荐

### 优化
- FAIL样本参数推荐升级为 v2 版本，智能推荐更精准的安全参数
- 语义合理性分数 `semantic_plausibility_score` 透传到最终输出
- margin分级规则细化，七级风险边界更合理

### 修复
- 若干边界场景判定精度修复

## [4.2.0] - 2026-07-28

### 新增
- 动态接触面积计算：软质物体受力后接触面积增大，压强更贴近真实物理
- 冲量安全校验：移动类动作增加质量×速度约束，高速移重物自动收紧
- 反作用力约束：牛顿第三定律校验，机器人本体稳定性自动评估
- 七级风险分级（L0~L6），更精细的风险程度划分
- 风险子类型标签（risk_subtypes）

### 性能
- JSON模式平均延迟从 ~20μs 优化到 ~15μs

## [4.1.0] - 2026-07-20

### 新增
- JSON 参数模式（check_action），生产环境零解析开销
- FAV 三维动作分类器（Force-Action-Velocity）
- 干扰等级评估模块

### 优化
- 11项评估改进与性能优化
- 测试覆盖扩展到 240+ 场景

## [4.0.0] - 2026-07-15

### 新增
- 首个公开发布版本
- 四层安全架构：语义解析 → 安全适配 → 动作分类 → 综合决策
- 35+ 中文动词支持
- 7种物体类型
- ISO 10218 / ISO/TS 15066 设计对齐
- 自然语言 + JSON 双输入模式
