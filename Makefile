PYTHON ?= $(if $(wildcard .venv/bin/python),.venv/bin/python,python3)

.PHONY: test check check-hil build
test:
	PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=python:tests $(PYTHON) -m unittest discover -s tests -v

check check-hil: export PYTHONPATH := python:tests

check: test
	$(PYTHON) -m unittest tools.production_topic_mock.test_group_contract -v
	$(PYTHON) -m pip check
	$(PYTHON) examples/body_state_subscriber/body_state_subscriber.py --help
	$(PYTHON) examples/end_effector_state_subscriber/end_effector_state_subscriber.py --help
	$(PYTHON) examples/end_effector/end_effector_external.py --help
	$(PYTHON) examples/external_control/external_control.py --help
	$(PYTHON) examples/teleop_policy/teleop_policy.py --help
	$(PYTHON) examples/brainco_hand/brainco_hand_external.py --help
	$(PYTHON) examples/brainco_hand/brainco_capture.py --help
	$(PYTHON) examples/gripper/gripper_external.py --help
	$(PYTHON) examples/velocity_mode/velocity_mode.py --help
	$(PYTHON) examples/wbc_tracking_mode/wbc_tracking_mode.py --help
	test -L CLAUDE.md
	test "$$(readlink CLAUDE.md)" = AGENTS.md
	cmp AGENTS.md CLAUDE.md
	git diff --check

# Real public EE binding/capture gate; no robot connection or lease.
check-hil:
	$(PYTHON) -c 'import mcap; from locomotion_aorta.brainco_capture import load_bindings; load_bindings()'
	PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=python:tests $(PYTHON) -m unittest test_brainco_capture -v

build:
	$(PYTHON) -m build --wheel --no-isolation --outdir dist/raw
