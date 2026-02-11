# SRDrone 3.2 / 3.3 骨架实现说明（当前版本）

本文档说明我已实现的**论文对齐骨架**，用于你快速审查结构是否符合论文意图。

## 1. 实现目标与边界

- 目标：先搭建论文 `3.2 Continuous State Evaluation` 与 `3.3 Hierarchical BT Modification` 的**工程骨架与数据流**。
- 边界：当前仅骨架/占位逻辑，不包含最终算法细节与阈值。
- 约束：**未修改任何底层控制代码**（如 `controller/`、`common_msgs/`、`object_det/`）。

## 额外说明：输出格式策略（重要）

- 当前实现已切换为**text-first**：每个阶段的主输出是 `raw_text`（自由文本）。
- 论文中的符号（如 `Π`、`π_i`、`T_sem`、`F_alpha`）只用于解释语义，不作为强制 JSON/固定 schema。
- 可选结构化字段仅用于工程调试和后处理（例如可视化、日志检索），不要求 LLM 严格遵守。
- 即：阶段可以“提及” `Π` 等概念，但真实 LLM 输出依然是一段自然语言文本。

## 2. 总体接线方式

- workflow 入口通过 `workflow/cli.py` 组装三部分：
  1) `StateProvider`（执行快照输入）
  2) `Evaluator`（3.2）
  3) `Reflector`（3.3）
- 运行闭环：`snapshot -> evaluate -> reflect -> experience_store(JSONL)`。
- 当前接线类：
  - Evaluator: `evaluator.pipeline.ContinuousStateEvaluatorPipeline`
  - Reflector: `reflector.pipeline.HierarchicalBTReflectorPipeline`

### 2.1 Stage A 输入来源（更新）

- `Action-Centric State Filtering` 的工程输入默认来自 `controller/config/BTlog.txt`。
- 即：controller 已完成过滤，workflow 侧不重复做原始高频流过滤。
- `BTlog` 会按 session 拆分：每次任务由两段组成：
  1) 一段 `<root ...> ... </root>` 行为树（本次 plan）
  2) 紧随其后的过滤日志（直到下一个 `<root ...>`）
- workflow 解析后按 cycle 选择 session：`cycle=0` 取最新一次，`cycle=1` 取倒数第二次。
- CLI 默认参数：
  - `--state-source controller_log`
  - `--controller-log-path controller/config/BTlog.txt`
  - `--controller-log-max-lines 120`
- 如需调试可切换 `--state-source stub`。

## 3. Evaluator（对应论文 3.2）

### 3.1 阶段划分（已对齐）

1. **Action-Centric State Filtering**
   - 主输出：来自 controller 过滤日志的自然语言文本（`raw_text`）。
   - 可选：解析为 `Π = (π_1, ..., π_n)` 仅用于工程处理。
2. **Spatio-Temporal Semantic Derivation (CMSR)**
   - 主输出：`T_sem` 的自然语言叙事（`raw_text`）。
   - 可选：保留 `T_ego/R` 解析结果作为调试信息。
3. **Semantic-Based Task Evaluation**
   - 主输出：多维评价解释文本（`raw_text`）。
   - 可选：三维评价解析字段占位：
     - Navigational Intent Verification
     - Proximity Awareness
     - Collision-Centric Safety

### 3.2 代码位置

- 接口：`evaluator/core/interfaces.py`
- 数据结构：`evaluator/core/models.py`
- 阶段实现：`evaluator/core/stages.py`
- 管线封装：`evaluator/pipeline.py`

### 3.3 输出结构（写入 workflow）

- `EvaluationResult.task_narrative`：承载 `T_sem` 文本叙事。
- `EvaluationResult.stage_outputs`：保留三阶段中间结果（便于后续替换为真实算法）。
- 三阶段均附带 `raw_text`，作为主语义输出。

## 4. Reflector（对应论文 3.3）

### 4.1 阶段划分（已对齐）

1. **Hierarchical Plan Analysis**
   - 主输出：分层问题定位文本（`raw_text`）。
   - 可选：解析为 `F_alpha / F_lambda / F_pi`。
2. **Dual-Constraint Processing**
   - 主输出：约束检查解释文本（`raw_text`）。
   - 可选：`A/L` 白名单与 validator 结构化结果。
3. **Node-level Precise Modification**
   - 主输出：节点级修正建议文本（`raw_text`）。
   - 可选：解析成 `e = <tau, omega>`，其中 `omega = [OPERATION] + [RATIONALE]`。
4. **Experience Base Update**
   - 主输出：经验更新说明文本（`raw_text`）。
   - 可选：形成 `Enew` 并输出 `Ebase = Ebase U Enew` 的结构化载荷。

### 4.2 代码位置

- 接口：`reflector/core/interfaces.py`
- 数据结构：`reflector/core/models.py`
- 阶段实现：`reflector/core/stages.py`
- 管线封装：`reflector/pipeline.py`

### 4.3 当前策略

- 所有修改均为 `proposal_only`，**不会直接改 BT 文件**。
- `A/L` 约束与 validator 已有框架字段，后续可直接替换为你论文/实验中的完整规则。
- Reflector 四阶段也采用 text-first 输出：分析文本、约束验证文本、修改建议文本、经验更新文本。

## 5. Workflow 侧配套改动

- 扩展了 workflow 输出字段以承载论文语义：
  - `EvaluationResult` 新增叙事与阶段输出字段。
  - `ReflectionPlan` 新增分层发现与经验更新字段。
- 状态提供 stub 扩展为 action/modality/pose/environment/failure_tag 结构，便于驱动 3.2 骨架。

## 6. 运行与验证

```bash
python3 -m workflow.cli --mission-id srdrone_paper --max-cycles 1
```

- 已验证：链路可执行，且会写入 `workflow/data/experience_records.jsonl`。

## 7. 当前未实现（待你确认后补）

- 3.2 中 `DeriveMotion` / `InferSpatialRelation` / `Align` 的真实算法。
- 三维语义评估的正式判定逻辑与阈值。
- 3.3 中 `GenerateOp/GenerateRation` 的真实生成器与 deterministic validator 实装。
- Experience Base 的 kNN 检索（论文提到 `all-mpnet-base-v2`）尚未接入。

## 8. 建议你优先审查

1. 阶段命名和顺序是否与论文最终定稿完全一致。
2. `Π / T_sem / F_alpha/F_lambda/F_pi / e=<tau,omega>` 这些符号是否需要改成你论文中的精确记号。
3. `A/L` 的白名单定义是否覆盖你真实平台动作与 BT 控制节点。
