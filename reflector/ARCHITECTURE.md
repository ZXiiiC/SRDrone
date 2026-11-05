# Reflector architecture

`HierarchicalBTAnalysisStage` separates flaws by action alpha, logic lambda and mission pi. `DualConstraintStage` checks the source plan and provides the audited action/logic strategy space. `NodeLevelCorrectionStage` emits structured reflective experience, following Eq. 7. `ExperienceBaseUpdateStage` preserves the experience proposal and its source-plan identity.

`BTPlanner.refine` applies the proposed experience to regenerate a complete XML. Candidate validation checks the actual generated tree rather than merely checking a list of suggested edits. The candidate is returned in `ReflectionPlan.candidate_bt_xml`, with its validation report, for the separate execution loop.

This is reconstructed code and adapted response schemas. The exact original private modification implementation is unavailable. See [RECOVERY.md](../docs/RECOVERY.md).
