# Reflector Skeleton (Paper §3.3)

This module provides a paper-aligned **skeleton** for Hierarchical BT Modification.

## Output Strategy (Text-First)

- Reflective outputs are **text-first**: analysis, constraint checks, correction suggestions, and experience updates are generated as `raw_text`.
- Paper symbols (`F_alpha/F_lambda/F_pi`, `e=<tau,omega>`, `Enew`) are explanatory abstractions, not strict output schema constraints.
- Structured objects in code are optional parsed envelopes for tooling and logging.

## Stage Pipeline

1. **Hierarchical BT Analysis**
   - Analyze three layers with paper notation:
     - Action Layer (`α`) -> `F_alpha`
     - Logic Layer (`λ`) -> `F_lambda`
     - Mission Layer (`Π`) -> `F_pi`
2. **Dual-Constraint Processing**
   - Evaluate strategy-space constraints with whitelist validation:
     - Action Space (`A`): hardware-supported atomic actions
     - Logic Space (`L`): BT.CPP-supported control nodes
3. **Node-Level Correction Specification**
   - Generate structured reflective experiences `e = <tau, omega>`:
     - `tau ∈ {alpha, lambda, pi}`
     - `omega = [OPERATION] + [RATIONALE]`
   - Current stage remains proposal-only (non-destructive), with text-first correction suggestions.
4. **Experience Base Update Planning**
   - Form `Enew` and prepare `Ebase = Ebase U Enew` update payload.

## Key Files

- `reflector/core/interfaces.py`: stage contracts.
- `reflector/core/stages.py`: placeholder stage implementations.
- `reflector/pipeline.py`: end-to-end reflector pipeline integrated with workflow.

## Current Status

- Pipeline order and interfaces are fixed.
- Node correction remains non-destructive (no BT file mutation).
- Ready for later algorithm insertion under paper constraints.
