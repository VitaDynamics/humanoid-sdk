"""Explicit two-Node SDK fixture, invoked only inside the local Mock runner."""
import argparse
from contextlib import ExitStack
import json
import os
from pathlib import Path

try:
    from .run import port, session_config, validate_client_environment
except ImportError:
    from run import port, session_config, validate_client_environment


def run(output, *, legacy_fixed_listener=False):
    output = Path(output).resolve()
    result_path = output / "multinode-fixture.json"
    if result_path.exists():
        raise ValueError("refusing to overwrite multi-Node fixture evidence")
    group = validate_client_environment(output)
    from aorta.errors import InternalError
    from locomotion_aorta import PullController, PullStateSource
    result = {"status": "INCOMPLETE", "pid": os.getpid(), "group": group,
              "legacy_fixed_listener": legacy_fixed_listener,
              "scope": "same-process SDK Nodes and local Mock; no EXTERNAL or BodyCmd"}
    try:
        if legacy_fixed_listener:
            # Negative control reproduces the old generated profile only in
            # this new capture. Keep the assigned profile as separate evidence.
            profile = output / "client-session.json"
            original = profile.read_text()
            (output / "client-session.generated.json").write_text(original)
            peers = [int(value.rsplit(":", 1)[1])
                     for value in json.loads(original)["connect"]["endpoints"]]
            profile.write_text(session_config(port(), peers))
        with ExitStack() as cleanup:
            state = PullStateSource(group=group, node_name="mock-multinode-state")
            cleanup.callback(state.close)
            result["state_node_created"] = True
            try:
                command = PullController(group=group, node_name="mock-multinode-command")
            except InternalError as exc:
                if not legacy_fixed_listener or "Address already in use" not in str(exc):
                    raise
                result["expected_listener_failure"] = {"code": int(exc.code), "error": str(exc)}
                result["status"] = "PASS_EXPECTED_OLD_FIXED_LISTENER_FAILURE"
            else:
                cleanup.callback(command.close)
                if legacy_fixed_listener:
                    raise AssertionError("old fixed-listener profile unexpectedly allowed both Nodes")
                if state.transport.node is command.transport.node:
                    raise AssertionError("fixture must create two independent Nodes")
                groups = [state.transport.node.group(), command.transport.node.group()]
                if groups != [group, group]:
                    raise AssertionError("effective Node groups differ from the assigned Mock group")
                first = state.wait_state(5)
                second = command.wait_state(5)
                info = command.get_robot_info(timeout=5)
                if first.state_id <= 0 or second.state_id <= 0 or info.info.body.slot_count != 31:
                    raise AssertionError("both Nodes must communicate with the synthetic body31 Mock")
                result.update(command_node_created=True, effective_groups=groups,
                              state_node_state_id=int(first.state_id),
                              command_node_state_id=int(second.state_id),
                              robot_info_slots=info.info.body.slot_count,
                              status="PASS_SAME_PROCESS_TWO_NODES_WITH_MOCK")
    except BaseException as exc:
        result["error"] = repr(exc)
        raise
    finally:
        result_path.write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--legacy-fixed-listener", action="store_true",
                        help="explicit negative control; expect second Node INTERNAL(-5)")
    args = parser.parse_args()
    run(args.output, legacy_fixed_listener=args.legacy_fixed_listener)
