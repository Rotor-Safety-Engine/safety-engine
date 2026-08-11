# 更新日志

所有重要的版本更新都会记录在这个文件中。

格式基于 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，
版本号遵循 [语义化版本](https://semver.org/lang/zh-CN/)。

> **关于版本历史**：本项目于 2026 年 8 月首次在 GitHub 公开开源，
> 开源版本号从 v1.0.0 起算。此前为内部研发阶段，不计入开源版本序列。

## [1.0.1] - 2026-08-11

### 修复
- 修复 Python 3.8 兼容性：移除重复的 `@staticmethod` 装饰器（3.8 不支持双重装饰）
- CI 性能测试阈值从 0.1ms 放宽至 0.15ms，避免 CI runner 性能波动导致误杀
- CI 关闭 fail-fast，单个 Python 版本失败不再取消其他版本测试

## [1.0.0] - 2026-08-11

### 🏁 首次公开开源

### 核心特性
- 纯物理动力学安全判定引擎，四层拦截架构
- 349 项自动化测试全通过，向后兼容
- 单文件零依赖，可直接拷贝使用，也可 pip 安装
- 支持 JSON 模式与 API 模式两种调用方式
- 平均延迟 ~17μs，约 5.8 万次/秒

### 工程化
- 完整 Typing 类型注解覆盖，mypy 零错误
- 外部 JSON 配置加载，支持自定义动词库 / 物体库 / 动作规则
- 输入参数全面校验（类型 / 范围 / NaN / 越界）
- GitHub Actions CI（Python 3.8 ~ 3.12 矩阵测试）
- PyPI 包发布（`pip install rotor-safety-engine`）
- 中英文双 README + Issue / PR 模板
