# 贡献指南

感谢你对 Rotor Safety Engine 的关注！欢迎以各种形式参与贡献。

## 你可以贡献什么

- 🐛 **报告 Bug** — 发现任何异常行为、错误结果或性能问题
- 💡 **功能建议** — 新的动作类型、物体类别、判定规则
- 📖 **文档改进** — 错别字、表述不清、示例补充
- 🔧 **代码贡献** — Bug修复、性能优化、新功能

## 快速开始

### 报告 Issue

1. 先搜索已有 Issue，避免重复
2. 选择合适的 Issue 模板
3. 提供可复现的代码、输入参数和预期输出
4. 说明你的环境（Python版本、操作系统）

### 提交代码

1. Fork 本仓库
2. 创建你的特性分支 (`git checkout -b feature/amazing-feature`)
3. 确保所有测试通过 (`python tests/test_engine.py`)
4. 提交你的更改 (`git commit -m 'Add some amazing feature'`)
5. 推送到分支 (`git push origin feature/amazing-feature`)
6. 发起 Pull Request

## 代码规范

- 遵循 PEP 8 编码风格
- 保持单文件零依赖的设计原则（社区版）
- 新增功能必须附带测试用例
- 性能敏感代码请附带基准测试数据

## 测试

```bash
# 运行全部测试
python tests/test_engine.py

# 使用 pytest
pip install pytest
pytest tests/test_engine.py -v
```

新增功能时，请确保：
- 不破坏已有测试
- 为新功能添加对应的测试用例
- 性能测试无明显退化

## 社区版 vs 企业版

本仓库是社区版（MIT协议），仅包含基础安全判定功能。
力学增强层（Layer 5）属于企业版，不接受开源贡献。

如果你对企业版功能感兴趣，请联系 contact@rotor-dynamics.ai。

## 行为准则

- 尊重不同意见和技术选择
- 聚焦技术讨论，避免人身攻击
- 保持专业和友善的沟通氛围
