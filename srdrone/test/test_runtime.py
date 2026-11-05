from __future__ import annotations

import hashlib
import json
import tempfile
import threading
import unittest
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from unittest.mock import patch
from urllib.error import HTTPError

from evaluator.core.stages import ActionCentricStateCaptureStage
from evaluator.pipeline import ContinuousStateEvaluatorPipeline
from reflector.pipeline import HierarchicalBTReflectorPipeline
from srdrone.bt import plan_digest, validate_bt
from srdrone.demo import EXAMPLES, FixtureExecutor, run_demo
from srdrone.execution import FileExchangeExecutor, write_json
from srdrone.experience import ExperienceBank
from srdrone.llm import ChatModel, ModelError, RecordedModel, json_object
from srdrone.logs import parse_action_states
from srdrone.planner import BTPlanner
from srdrone.ros_adapter import collect_result
from srdrone.runner import RefinementRunner
from workflow.core.models import EvaluationResult, ExecutionSnapshot, MissionContext
from workflow.parsers.controller_btlog import parse_btlog_sessions, select_session_for_cycle


def tree(body):
    return '<root BTCPP_format="4"><BehaviorTree ID="MainTree">' + body + "</BehaviorTree></root>"


class ValidationTests(unittest.TestCase):
    def test_generated_fixtures_and_notebook_plan_validate(self):
        for name in ("initial.xml", "corrected.xml"):
            self.assertTrue(validate_bt((EXAMPLES / name).read_text()).valid)
        original = Path(__file__).parents[2] / "examples/notebook_detection_failure.xml"
        if original.exists():
            self.assertTrue(validate_bt(original.read_text()).valid)

    def test_invalid_action_ports_numbers_and_decorators(self):
        bodies = [
            "<GrabObject/>",
            '<TakeOff takefoff_target="0;0;1"/>',
            '<FlyToCoordinates coordinates="nan;0;1"/>',
            '<MoveForward MoveForward_distance="-1"/>',
            '<Repeat num_cycles="-1"><Land/></Repeat>',
            '<RetryUntilSuccessful num_attempts="6"><Land/></RetryUntilSuccessful>',
            '<Repeat num_cycles="2"><Land/><Land/></Repeat>',
            "<Sequence/>",
            '<Land bogus="1"/>',
            "<Land><Land/></Land>",
            '<FlyToCoordinates coordinates="{frame.x};{frame.y};1"/>',
        ]
        for body in bodies:
            with self.subTest(body=body):
                self.assertFalse(validate_bt(tree(body)).valid)

    def test_branch_dataflow_requires_guaranteed_typed_definition(self):
        detect = '<ForwardDetectFrame FramePosition="{p}"/>'
        fly = '<FlyToCoordinates coordinates="{p}"/>'
        good = tree("<Sequence><Fallback>" + detect + detect + "</Fallback>" + fly + "</Sequence>")
        bad = tree("<Sequence><Fallback>" + detect + "<Land/></Fallback>" + fly + "</Sequence>")
        self.assertTrue(validate_bt(good).valid)
        self.assertFalse(validate_bt(bad).valid)
        self.assertFalse(validate_bt(tree(fly)).valid)
        wrong_type = tree('<Sequence><DownDetectH DownDetectHDxy="{p}"/>' + fly + "</Sequence>")
        self.assertFalse(validate_bt(wrong_type).valid)

    def test_xml_entities_and_oversized_input_rejected(self):
        self.assertFalse(validate_bt('<!DOCTYPE x [<!ENTITY e "bad">]>' + tree("<Land/>")).valid)
        self.assertFalse(validate_bt("x" * 200001).valid)
        nested = "<Inverter>" * 40 + "<Land/>" + "</Inverter>" * 40
        self.assertFalse(validate_bt(tree(nested)).valid)

    def test_digest_ignores_formatting_but_not_motion(self):
        self.assertEqual(plan_digest(tree("<Land/>")), plan_digest(tree("  <Land />\n")))
        self.assertNotEqual(
            plan_digest((EXAMPLES / "initial.xml").read_text()),
            plan_digest((EXAMPLES / "corrected.xml").read_text()),
        )

    def test_xml_repair_gets_actual_validation_error(self):
        model = RecordedModel(
            [
                {"stage": "plan", "response": tree("<Invented/>")},
                {"stage": "xml_repair", "response": tree("<Land/>")},
            ]
        )
        xml = BTPlanner(model).generate("land", {})
        self.assertTrue(validate_bt(xml).valid)
        self.assertIn("unsupported node", model.calls[-1]["payload"]["validation_error"])


class LogTests(unittest.TestCase):
    def test_sensor_and_drone_poses_do_not_mix(self):
        text = (
            "Takeoff finished. drone pose[nwu,yaw]:(0,0,1.5,179)\n"
            "ForwardDetectFrame frame position [nwu]:(2e0,0,1.5)\n"
            "ForwardDetectFrame finished. drone pose [nwu,yaw]:(0,0,1.5,-179)\n"
            "ForwardDetectFrame FAILURE. no object\n"
            "MoveForward finished. drone pose [nwu,yaw] [ 1, 0, 1.5, -179)"
        )
        states = parse_action_states(text)
        self.assertEqual(len(states), 4)
        self.assertEqual(states[0].action, "TakeOff")
        self.assertEqual(states[1].environment_observation["position_nwu"], [2, 0, 1.5])
        self.assertEqual(states[1].state_vector["x"], 0)
        self.assertEqual(states[2].state_vector, {})
        self.assertTrue(states[2].failure_tag)
        self.assertEqual(states[3].state_vector["yaw"], -179)

    def test_sessions_support_one_line_and_do_not_consume_partial_plan(self):
        xml = tree("<Land/>")
        text = (
            xml
            + "\nLand finished. drone pose [nwu,yaw]:(0,0,0,0)\n"
            + xml
            + "\nfailed\n<root incomplete"
        )
        sessions = parse_btlog_sessions(text)
        self.assertEqual(len(sessions), 2)
        self.assertEqual(sessions[-1].filtered_log_text, "failed")
        self.assertIsNone(select_session_for_cycle(sessions, 2))

    def test_empty_log_is_not_evaluated(self):
        with self.assertRaises(ValueError):
            ActionCentricStateCaptureStage().run(
                MissionContext("m", "MainTree"),
                ExecutionSnapshot(
                    datetime.now(timezone.utc),
                    0,
                    {"plan_xml": tree("<Land/>"), "filtered_log_text": ""},
                ),
            )


class ModelTests(unittest.TestCase):
    def test_json_is_strict(self):
        self.assertEqual(json_object('```json\n{"ok": true}\n```'), {"ok": True})
        for text in ("Success", "[]", '{"success": True}', "```json\n{}"):
            with self.assertRaises(ModelError):
                json_object(text)

    def test_unknown_result_does_not_trigger_unsupported_reflection(self):
        model = RecordedModel([])
        reflection = HierarchicalBTReflectorPipeline(model=model).reflect(
            MissionContext("m", "MainTree"),
            ExecutionSnapshot(datetime.now(timezone.utc), 0, {}),
            EvaluationResult(0, False, "insufficient evidence", diagnostics={"result": "unknown"}),
        )
        self.assertFalse(reflection.needs_modification)
        self.assertEqual(model.calls, [])

    def test_auth_error_is_not_retried_or_leaked(self):
        model = ChatModel("test", "private-test-key", "https://provider.example/v1")
        with patch(
            "srdrone.llm.urlopen", side_effect=HTTPError("x", 401, "secret body", None, None)
        ) as call:
            with self.assertRaises(ModelError) as context:
                model.complete("test", "system", {})
        self.assertEqual(call.call_count, 1)
        self.assertNotIn("private-test-key", str(context.exception))
        self.assertNotIn("secret body", str(context.exception))

    def test_transport_roundtrip_and_trace_exclude_credentials(self):
        received = []

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                received.append(json.loads(self.rfile.read(int(self.headers["Content-Length"]))))
                body = json.dumps(
                    {
                        "choices": [
                            {
                                "finish_reason": "stop",
                                "message": {"content": '{"result":"unknown"}'},
                            }
                        ],
                        "usage": {"total_tokens": 12},
                    }
                ).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *args):
                pass

        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            with tempfile.TemporaryDirectory() as temporary:
                trace = Path(temporary) / "trace.jsonl"
                model = ChatModel(
                    "test",
                    "local-private-key",
                    f"http://127.0.0.1:{server.server_port}/v1",
                    trace_path=str(trace),
                )
                response = model.complete("outcome", "system", {"task": "land"})
                self.assertEqual(json_object(response)["result"], "unknown")
                self.assertEqual(received[0]["messages"][0]["role"], "system")
                self.assertNotIn("local-private-key", trace.read_text())
                self.assertEqual(json.loads(trace.read_text())["usage"]["total_tokens"], 12)
        finally:
            server.shutdown()
            server.server_close()
            thread.join()

    def test_contradictory_success_is_rejected(self):
        responses = json.loads((EXAMPLES / "responses.json").read_text())[:4]
        responses[-1]["response"]["result"] = "success"
        model = RecordedModel(responses)
        mission = MissionContext("m", "MainTree", metadata={"task": "cross frame", "scene": {}})
        snapshot = FixtureExecutor().execute(mission, (EXAMPLES / "initial.xml").read_text(), 0)
        with self.assertRaises(ModelError):
            ContinuousStateEvaluatorPipeline(model=model).evaluate(mission, snapshot)
        # Raw measured logs reach outcome determination, alongside the ideal one.
        self.assertIn("Land finished.", model.calls[-1]["payload"]["raw_log"])
        self.assertIn("idealized_reconstruction", model.calls[-1]["payload"]["cmsr"]["metadata"])


class ExperienceAndLoopTests(unittest.TestCase):
    def test_demo_refines_then_executes_and_validates_experience(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "demo"
            results = run_demo(str(output))
            self.assertEqual(len(results), 2)
            self.assertFalse(results[0].evaluation.success)
            self.assertTrue(results[0].reflection.candidate_bt_xml)
            self.assertTrue(results[1].evaluation.success)
            bank = ExperienceBank(str(Path(temporary) / "demo-experience.sqlite3"))
            self.assertEqual(len(bank.records()), 1)
            self.assertEqual(bank.records()[0]["status"], "execution_validated")
            summary = json.loads((output / "summary.json").read_text())
            self.assertEqual(summary["refinements_executed"], 1)

    def test_pending_experiences_excluded_and_retrieval_persists(self):
        with tempfile.TemporaryDirectory() as temporary:
            bank = ExperienceBank(str(Path(temporary) / "experience.sqlite3"))
            entry = {
                "layer": "action",
                "target": "frame",
                "operation": "Move beyond frame",
                "rationale": "Cross frame instead of stopping at center",
            }
            bank.add("cross frame", "navigation", "a", "b", "run1", [entry, entry])
            self.assertEqual(len(bank.records(include_proposed=True)), 1)
            self.assertEqual(bank.retrieve("cross frame", backend="lexical"), [])
            bank.validate_candidate("b", "wrong-run")
            self.assertEqual(bank.records(), [])
            bank.validate_candidate("b", "run1")
            reopened = ExperienceBank(str(bank.path))
            self.assertEqual(
                reopened.retrieve("cross frame", backend="lexical")[0]["experience"], entry
            )

    def test_mismatched_execution_is_rejected(self):
        class WrongExecutor:
            def execute(self, mission, plan, cycle):
                return ExecutionSnapshot(
                    datetime.now(timezone.utc), cycle, {"plan_xml": tree("<Land/>")}
                )

        with tempfile.TemporaryDirectory() as temporary:
            model = RecordedModel([])
            planner = BTPlanner(model)
            runner = RefinementRunner(
                WrongExecutor(),
                ContinuousStateEvaluatorPipeline(model=model),
                HierarchicalBTReflectorPipeline(model=model),
                planner,
                ExperienceBank(str(Path(temporary) / "bank.sqlite3")),
            )
            with self.assertRaisesRegex(ValueError, "does not match"):
                runner.run(
                    MissionContext("m", "MainTree", metadata={"task": "cross frame"}),
                    str(Path(temporary) / "run"),
                    initial_plan=(EXAMPLES / "initial.xml").read_text(),
                    top_k=0,
                )
            self.assertTrue((Path(temporary) / "run/error.json").exists())

    def test_file_exchange_rejects_stale_result(self):
        with tempfile.TemporaryDirectory() as temporary:
            exchange = FileExchangeExecutor(
                str(Path(temporary) / "exchange"), timeout=0.1, poll_interval=0.01
            )
            mission = MissionContext("m", "MainTree", metadata={"task": "land"})

            def write_wrong_result(_):
                write_json(
                    exchange.directory / "cycle-000/result.json",
                    {
                        "mission_id": "m",
                        "cycle_index": 0,
                        "plan_digest": "wrong",
                        "root_status": "SUCCESS",
                        "filtered_log_text": "Land finished.",
                    },
                )

            with patch("srdrone.execution.time.sleep", side_effect=write_wrong_result):
                with self.assertRaisesRegex(ValueError, "does not match"):
                    exchange.execute(mission, tree("<Land/>"), 0)

    def test_ros_receipt_matches_actual_embedded_plan(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            xml = tree("<Land/>")
            (directory / "controller-BTlog.txt").write_text(
                xml + "\nLand finished. drone pose[nwu,yaw]:(0,0,0,0)\nBT_ROOT_STATUS: SUCCESS\n"
            )
            request = {"mission_id": "m", "cycle_index": 0, "plan_digest": plan_digest(xml)}
            self.assertEqual(collect_result(directory, request)["root_status"], "SUCCESS")
            request["plan_digest"] = "wrong"
            with self.assertRaisesRegex(ValueError, "does not match"):
                collect_result(directory, request)

    def test_exhausted_budget_does_not_store_or_validate_a_correction(self):
        with tempfile.TemporaryDirectory() as temporary:
            model = RecordedModel(json.loads((EXAMPLES / "responses.json").read_text())[:4])
            bank = ExperienceBank(str(Path(temporary) / "bank.sqlite3"))
            runner = RefinementRunner(
                FixtureExecutor(),
                ContinuousStateEvaluatorPipeline(model=model),
                HierarchicalBTReflectorPipeline(model=model),
                BTPlanner(model),
                bank,
            )
            results = runner.run(
                MissionContext("m", "MainTree", metadata={"task": "cross frame"}),
                str(Path(temporary) / "run"),
                initial_plan=(EXAMPLES / "initial.xml").read_text(),
                max_refinements=0,
                top_k=0,
            )
            self.assertEqual(len(results), 1)
            self.assertFalse(results[0].evaluation.success)
            self.assertEqual(bank.records(include_proposed=True), [])
            self.assertEqual(model.responses, [])

    def test_file_exchange_accepts_matching_fresh_result(self):
        with tempfile.TemporaryDirectory() as temporary:
            exchange = FileExchangeExecutor(
                str(Path(temporary) / "exchange"), timeout=0.1, poll_interval=0.01
            )
            xml = tree("<Land/>")

            def finish(_):
                request = json.loads((exchange.directory / "cycle-000/request.json").read_text())
                write_json(
                    exchange.directory / "cycle-000/result.json",
                    {
                        "mission_id": request["mission_id"],
                        "cycle_index": 0,
                        "plan_digest": request["plan_digest"],
                        "root_status": "SUCCESS",
                        "filtered_log_text": "Land finished. drone pose[nwu,yaw]:(0,0,0,0)",
                    },
                )

            with patch("srdrone.execution.time.sleep", side_effect=finish):
                snapshot = exchange.execute(MissionContext("m", "MainTree"), xml, 0)
            self.assertEqual(snapshot.state_payload["root_status"], "SUCCESS")

    def test_recovered_prompts_match_manifest(self):
        root = Path(__file__).parents[2]
        manifest = json.loads((root / "srdrone/prompts/manifest.json").read_text())
        for item in manifest["excerpts"]:
            path = root / item["file"]
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), item["sha256"])
            self.assertGreater(len(path.read_text().strip()), 20)


if __name__ == "__main__":
    unittest.main()
