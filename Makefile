PYTHON ?= $(if $(wildcard .venv/bin/python),.venv/bin/python,python3)

.PHONY: test check check-hil build
test:
	PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=.:tests $(PYTHON) -m unittest discover -s tests -v

check: test
	$(PYTHON) -m pip check
	$(PYTHON) examples/lowstate_subscriber.py --help
	$(PYTHON) examples/external_control.py --help
	$(PYTHON) examples/brainco_hand_control.py --help
	$(PYTHON) examples/brainco_capture.py --help
	test -L CLAUDE.md
	test "$$(readlink CLAUDE.md)" = AGENTS.md
	cmp AGENTS.md CLAUDE.md
	git diff --check

# Optional HIL checks must not silently skip when the candidate is requested.
check-hil:
	$(PYTHON) -c 'import mcap; from locomotion_aorta.brainco_capture import load_bindings; load_bindings()'
	PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=.:tests $(PYTHON) -m unittest test_brainco_capture -v

build:
	$(PYTHON) -m build --wheel
