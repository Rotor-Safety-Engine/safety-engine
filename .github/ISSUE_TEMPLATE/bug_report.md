---
name: Bug 报告
about: 报告一个错误或异常行为
title: "[Bug] "
labels: bug
assignees: ''

---

## 描述

清楚地描述这个bug是什么。

## 复现步骤

1. 传入参数：'...'
2. 执行函数：'....'
3. 得到结果：'....'
4. 预期应该是：'....'

## 复现代码

```python
from safety_engine import SafetyEngineV4

engine = SafetyEngineV4()
result = engine.check_command(...)
print(result)
```

## 环境信息

- Python版本：[例如 3.10.6]
- 操作系统：[例如 Ubuntu 22.04 / Windows 11]
- 引擎版本：[例如 v4.2.3]

## 其他信息

任何补充说明、截图或相关文件。
