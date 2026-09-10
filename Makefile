PYTHON ?= $(if $(wildcard .venv/bin/python),.venv/bin/python,python3)

.PHONY: test check build
test:
	PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=.:tests $(PYTHON) -m unittest discover -s tests -v

check: test
	$(PYTHON) -m pip check
	$(PYTHON) examples/lowstate_subscriber.py --help
	$(PYTHON) examples/external_control.py --help
	test -L CLAUDE.md
	test "$$(readlink CLAUDE.md)" = AGENTS.md
	cmp AGENTS.md CLAUDE.md
	git diff --check

build:
	$(PYTHON) -m build --wheel
