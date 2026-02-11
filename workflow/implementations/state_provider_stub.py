from __future__ import annotations

from datetime import datetime

from workflow.core.interfaces import StateProvider
from workflow.core.models import ExecutionSnapshot, MissionContext


class ActionCentricStateProviderStub(StateProvider):
    """Temporary state provider.

    TODO (paper §3.2): replace with action-centric state capture over ROS topics.
    """

    def capture(self, mission: MissionContext, cycle_index: int) -> ExecutionSnapshot:
        return ExecutionSnapshot(
            timestamp=datetime.utcnow(),
            cycle_index=cycle_index,
            state_payload={
                "mission_id": mission.mission_id,
                "bt_id": mission.bt_id,
                "status": "placeholder_execution_state",
                "action": "MoveForward",
                "modality": "motion_control",
                "x": 1.0,
                "y": 0.2,
                "z": 1.5,
                "yaw": 15.0,
                "environment": {
                    "objects": [
                        {
                            "type": "frame",
                            "position": {"x": 2.0, "y": 0.5, "z": 1.5},
                        }
                    ]
                },
                "failure_tag": None,
                "notes": "Replace with real flight/task state capture adapter.",
            },
        )
