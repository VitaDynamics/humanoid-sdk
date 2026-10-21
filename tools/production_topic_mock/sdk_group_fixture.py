"""Explicit real-SDK group precedence fixture, invoked only by the Mock runner."""
import argparse
import json
from pathlib import Path

try:
    from .run import validate_client_environment
except ImportError:
    from run import validate_client_environment


def run(output):
    output = Path(output).resolve()
    result_path = output / "sdk-group-fixture.json"
    if result_path.exists():
        raise ValueError("refusing to overwrite SDK group evidence")
    group = validate_client_environment(output)
    from locomotion_aorta import PullController
    result = {"expected_group": group, "cases": [], "status": "INCOMPLETE",
              "scope": "real loopback SDK/RobotInfo/BodyState only; no EXTERNAL or BodyCmd"}
    try:
        for label, kwargs in (("sdk_default", {}),
                              ("sdk_explicit_other", {"group": "sdk-argument-unused"})):
            requested = kwargs.get("group", "default")
            if requested == group:
                raise ValueError("group fixture requires an isolated, non-default group")
            try:
                controller = PullController(**kwargs)
            except ValueError as exc:
                message = str(exc)
                if (f"requested={requested!r}" not in message or
                        f"actual={group!r}" not in message or
                        f"AORTA_GROUP={group!r}" not in message):
                    raise
                result["cases"].append({"case": label, "constructor_arguments": kwargs,
                    "expected_rejection": True, "error": message})
            else:
                controller.close()
                raise AssertionError(label + ": SDK accepted a mismatched group")
        controller = PullController(group=group)
        try:
            effective = controller.transport.node.group()
            if effective != group:
                raise ValueError(f"effective group {effective!r} != {group!r}")
            reply = controller.get_robot_info(timeout=5)
            state = controller.wait_state(5)
            if reply.info is None or reply.info.body is None or reply.info.body.slot_count != 31:
                raise ValueError("synthetic RobotInfo missing")
            if state.state_id <= 0:
                raise ValueError("no real synthetic BodyState received")
            result["cases"].append({"case": "sdk_explicit_matching",
                "constructor_arguments": {"group": group}, "effective_group": effective,
                "robot_info_slots": reply.info.body.slot_count, "state_id": int(state.state_id)})
        finally:
            controller.close()
        result["status"] = "PASS_ISOLATED_SDK_GROUP_PRECEDENCE_ONLY"
    except Exception as exc:
        result["error"] = repr(exc)
        raise
    finally:
        result_path.write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    run(parser.parse_args().output)
