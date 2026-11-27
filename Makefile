PYTHON ?= python3
.PHONY: all bootstrap build check
all: build
bootstrap:
	$(PYTHON) tools/nexfoot.py bootstrap
build:
	$(PYTHON) tools/nexfoot.py build
check:
	$(PYTHON) tools/nexfoot.py check
