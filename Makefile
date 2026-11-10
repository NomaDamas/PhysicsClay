# PhysicsClay developer commands. Everything runs from the repo's .venv (created by `make setup`).
PY      ?= .venv/bin/python
PORT    ?= 8777
ARGS    ?=
LINT    := pps scripts tests

.PHONY: help setup dev run test lint fmt selftest clean

help:
	@echo "make setup     .venv + deps + robot models (scripts/setup.sh; PC_EXTRAS=vision,data,dev,yolo to choose)"
	@echo "make run       start the app on http://127.0.0.1:$(PORT)   (ARGS='--mock --https --webcam ...')"
	@echo "make test      unit tests (pytest, no server)"
	@echo "make selftest  end-to-end self test against an isolated server (PC_SELFTEST_OFFLINE=1 without keys)"
	@echo "make lint      ruff check"
	@echo "make fmt       ruff --fix, and ruff format on tests/"

setup:
	./scripts/setup.sh

dev: setup

run:
	$(PY) -m pps.app --port $(PORT) $(ARGS)

test:
	$(PY) -m pytest

selftest:
	$(PY) scripts/selftest.py

lint:
	$(PY) -m ruff check $(LINT)

# The older modules keep their dense style for now; new code (tests/, new modules) follows `ruff format`.
fmt:
	$(PY) -m ruff check --fix $(LINT)
	$(PY) -m ruff format tests

clean:
	find . -path ./.venv -prune -o -path ./vendor -prune -o -name __pycache__ -type d -exec rm -rf {} +
	rm -rf .pytest_cache .ruff_cache
