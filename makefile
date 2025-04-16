OS := $(shell uname 2>/dev/null || echo Unknown)

ifeq ($(OS), Darwin)  # macOS
    PYTHON := python3
else ifeq ($(OS), Linux)  # Linux
    PYTHON := python3
else  # Assume Windows (uname not available or unknown)
    PYTHON := python
endif

PIP      := $(PYTHON) -m pip
PYLINT   := $(PYTHON) -m pylint
COVERAGE := $(PYTHON) -m coverage
PYDOC    := $(PYTHON) -m pydoc
AUTOPEP8 := $(PYTHON) -m autopep8

SIM ?= kin

.PHONY: init remove_dep clean unit_tests report test

init:
	$(PIP) install -r requirements.txt

remove_dep:
	$(PYTHON) -c "import os, shutil; [shutil.rmtree(p, ignore_errors=True) for p, d, f in os.walk('.') if '__pycache__' in p]"
	$(PYTHON) -c "import shutil; shutil.rmtree('.mypy_cache', ignore_errors=True)"
	$(PYTHON) -c "import shutil; shutil.rmtree('.pytest_cache', ignore_errors=True)"
	-$(PYTHON) -c "import os; [os.remove(f) for f in os.listdir('.') if f.startswith('.coverage')]"

clean:
	$(PYTHON) -c "import shutil; shutil.rmtree('./4_unit_tests/python_test_results', ignore_errors=True)"
	$(PYTHON) -c "import os; os.makedirs('./4_unit_tests/python_test_results', exist_ok=True)"

unit_tests:
	-$(PYTHON) -m pytest ./4_unit_tests/python_tests --cov=src --cov-branch --cov=./4_unit_tests/python_tests --cov-report=html:./4_unit_tests/python_test_results/coverage_html \
	--html=./4_unit_tests/python_test_results/test_results.html --self-contained-html --css=./4_unit_tests/dark_mode.css $(ARGS)

report:
	$(PYTHON) -c "import webbrowser, os; webbrowser.open_new_tab(os.path.abspath('4_unit_tests/python_test_results/test_results.html'))"
	$(PYTHON) -c "import webbrowser, os; webbrowser.open_new_tab(os.path.abspath('4_unit_tests/python_test_results/coverage_html/index.html'))"

test: clean unit_tests remove_dep report

sim:
	python3 kernel.py $(SIM)