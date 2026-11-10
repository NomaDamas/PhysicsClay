# PhysicsClay developer commands. Everything runs from the repo's .venv (created by `make setup`).
PY         ?= .venv/bin/python
PORT       ?= 8780
HTTPS_PORT ?= 8781
HTTPS      ?= 1
ARGS       ?=
LINT       := pps scripts tests
# https by default: phones only share their camera and mic over https (a self-signed certificate in certs/)
HTTPS_ARGS := $(if $(filter 1 yes true on,$(HTTPS)),--https --https-port $(HTTPS_PORT),)

.PHONY: help setup dev run test lint fmt selftest clean

help:
	@echo "make setup     .venv + deps + robot models (scripts/setup.sh; PC_EXTRAS=vision,data,dev,yolo to choose)"
	@echo "make run       the app on http://127.0.0.1:$(PORT) and https://<lan-ip>:$(HTTPS_PORT) (phones)"
	@echo "               HTTPS=0 for http only; ARGS='--mock --webcam --kinematic ...'"
	@echo "make test      unit tests (pytest, no server)"
	@echo "make selftest  end-to-end self test against an isolated server (PC_SELFTEST_OFFLINE=1 without keys)"
	@echo "make lint      ruff check"
	@echo "make fmt       ruff --fix, and ruff format on tests/"

setup:
	./scripts/setup.sh

dev: setup

run:
	$(PY) -m pps.app --port $(PORT) $(HTTPS_ARGS) $(ARGS)

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
