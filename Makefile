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
CONTAINER_CMD := "make init"

.PHONY: init remove_dep clean unit_tests report test

init:
	$(PIP) install -r requirements.txt

remove_dep:
	$(PYTHON) -c "import os, shutil; [shutil.rmtree(p, ignore_errors=True) for p, d, f in os.walk('.') if '__pycache__' in p]"
	$(PYTHON) -c "import shutil; shutil.rmtree('.mypy_cache', ignore_errors=True)"
	$(PYTHON) -c "import shutil; shutil.rmtree('.pytest_cache', ignore_errors=True)"
	$(PYTHON) -c "import os; [os.remove(f) for f in os.listdir('.') if f.startswith('.coverage')]"

clean:
	$(PYTHON) -c "import shutil; shutil.rmtree('./unit_tests/python_test_results', ignore_errors=True)"
	$(PYTHON) -c "import os; os.makedirs('./unit_tests/python_test_results', exist_ok=True)"

report:
	$(PYTHON) -c "import webbrowser, os; webbrowser.open_new_tab(os.path.abspath('unit_tests/python_test_results/test_results.html'))"
	$(PYTHON) -c "import webbrowser, os; webbrowser.open_new_tab(os.path.abspath('unit_tests/python_test_results/coverage_html/index.html'))"

test:
	$(MAKE) clean
	docker build -t simulation-toolkit .

	docker run --name sim_env simulation-toolkit \
		/bin/bash -c " \
		$(PYTHON) -m pytest ./unit_tests/python_tests \
			--cov=src \
			--cov-branch \
			--cov=./unit_tests/python_tests \
			--cov-report=html:./unit_tests/python_test_results/coverage_html \
			--html=./unit_tests/python_test_results/test_results.html \
			--self-contained-html \
			--css=./unit_tests/dark_mode.css $(ARGS)"
	
	rm -r ./unit_tests/python_test_results
	docker cp sim_env:/home/vmod/unit_tests/python_test_results ./unit_tests/
	docker rm sim_env

	$(MAKE) remove_dep
	$(MAKE) report

sim:
	docker build -t simulation-toolkit .

	docker run --name sim_env simulation-toolkit \
		/bin/bash -c " \
		$(PYTHON) kernel.py $(SIM) $(MODEL_PATH) $(COMPARISON_PATH)"
	
	rm -r ./src/simulations/$(SIM)/$(SIM)_outputs
	docker cp sim_env:/home/vmod/src/simulations/$(SIM)/$(SIM)_outputs ./src/simulations/$(SIM)/
	docker rm sim_env

build:
	docker build -t simulation-toolkit .
	docker run --name sim_env simulation-toolkit
	# docker cp sim_env:/home/vmod/unit_tests/python_test_results ./unit_tests/
	docker rm sim_env
