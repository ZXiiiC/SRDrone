# Evaluator Skeleton (Paper §3.2)

This module provides a paper-aligned **skeleton** for Continuous State Evaluation.

## Output Strategy (Text-First)

- Stage outputs are **text-first**: each stage emits `raw_text` as the primary result.
- Paper symbols (`Π`, `π_i`, `T_sem`) are used for conceptual alignment only.
- Optional structured fields are parser artifacts for engineering/debug usage, not strict LLM output contracts.

## Stage Pipeline

1. **Action-Centric State Filtering**
   - Input: execution snapshot from workflow state provider (default source: `controller/config/BTlog.txt`).
   - Output: free-form filtering narrative (`raw_text`) containing:
     - current run plan XML block
     - current run filtered log block
     - optional parsed metadata (session index/count)
2. **CMSR Extraction**
   - Input: stage-1 text context (+ optional parsed states).
   - Output: free-form semantic trajectory narrative (`raw_text`), optionally parsed into CMSR artifacts.
3. **Semantic-Based Task Evaluation**
   - Input: stage-2 semantic narrative (`raw_text`).
   - Output: success flag, anomaly attribution, and three dimensions:
     - Navigational Intent Verification
     - Proximity Awareness
     - Collision-Centric Safety

## Key Files

- `evaluator/core/interfaces.py`: stage contracts.
- `evaluator/core/stages.py`: placeholder stage implementations.
- `evaluator/pipeline.py`: end-to-end evaluator pipeline integrated with workflow.

## Current Status

- Interfaces and stage boundaries are complete.
- Implementations are placeholders for algorithm insertion.
- Stage-A source defaults to controller filtered logs; no raw high-frequency sensor filtering is re-implemented here.
