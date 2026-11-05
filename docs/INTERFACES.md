# Audited controller interfaces

`srdrone.bt.ACTIONS` is a conservative subset of the registrations and `providedPorts()` declarations in `controller/src/ros1_node.cpp`. It is the authoritative generation whitelist. Coordinates are NWU in meters; yaw ports/logs use degrees, positive counterclockwise. Relative forward/left movement depends on current yaw; fixed NWU axes do not rotate with the drone.

| Action | Input ports | Optional output ports |
| --- | --- | --- |
| `TakeOff` | `takeoff_target` Position3D | — |
| `FlyToCoordinates` | `coordinates` Position3D | — |
| `MoveForward`, `MoveBack`, `MoveLeft`, `MoveRight`, `MoveUp`, `MoveDown` | `<NodeName>_distance` positive scalar | — |
| `SetYawAngle` | `yaw` scalar degrees | — |
| `TurnLeft`, `TurnRight` | `turn_attitude` positive scalar degrees | — |
| `Land` | — | — |
| `ForwardDetectFrame` | — | `FramePosition` Position3D, `ForwardDetectFrameData` objects |
| `DownDetectH` | — | `DownDetectHData` object, `DownDetectHDxy` Dxy, `SendVelocity` vector |

Position literals are `north;west;up`. A blackboard reference is a whole entry, e.g. `{frame_position}`. `"{p.x};{p.y};1"` is not supported by the controller's Position3D conversion. Generated expressions are never evaluated as code.

Supported logic: `Sequence`, `Fallback`, `Inverter`, `Repeat num_cycles="1..5"`, `RetryUntilSuccessful num_attempts="1..5"`. Decorators have exactly one child. For example, retrying a Sequence that **turns and then detects** changes the viewpoint; retrying a stationary detector alone does not broaden coverage.

The parser preserves completion/failure order, malformed legacy opening brackets (`[ ... )`), scientific notation, missing failure poses, and successful detection observations. It does not turn an object position into a drone position or infer a missing TakeOff pose from the commanded target. All raw log text is retained for the model stages. Historical files often lack a root status and may log Land success before touchdown; evidence remains qualified.

Examples in the notebook use interfaces from several experimental revisions. `ForwardDetectBalloon`, `DownScanHtag`, `DetectTarget`, `SaveData` and `UseData` are not registered by the current public C++ main. No alias silently changes those behaviors. Other registered legacy nodes are also omitted until their port/data semantics are audited. To extend the whitelist, implement/register the actual controller node, document its ports and failure semantics, and add matching dataflow tests.

The validator establishes an interface/structure check. It cannot prove camera observability, actuator feasibility, obstacle clearance, continuous collision avoidance, touchdown or overall task success.
