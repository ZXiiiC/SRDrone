# Workflow Docs Index

用于快速审阅当前 3.2 / 3.3 实现文档。

## Core Documents

- `workflow/IMPLEMENTATION_3_2_3_3.md`
  - 当前实现总览（中文）
  - 重点：text-first 输出策略、阶段职责、未实现项

- `evaluator/ARCHITECTURE.md`
  - 3.2 Evaluator 骨架说明
  - 重点：`raw_text` 主输出 + 可选结构化解析

- `reflector/ARCHITECTURE.md`
  - 3.3 Reflector 骨架说明
  - 重点：text-first 反思输出、`A/L` 约束、`Enew` 说明

- `workflow/README.md`
  - Workflow 框架总说明
  - 重点：输出约定与集成边界

## Review Checklist

1. 是否确认“论文符号仅概念表达，LLM 主输出为文本”这一设计。
2. 阶段命名与顺序是否与论文定稿一致。
3. `A/L` 白名单与后续 validator 设计是否符合你的实验实现。
4. 是否确认 Stage-A 以 `controller/config/BTlog.txt` 作为默认输入来源。
5. 是否确认 BTlog 的 session 拆分规则（plan XML + filtered log）符合 controller 实际输出。
