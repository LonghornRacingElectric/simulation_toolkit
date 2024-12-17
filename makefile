ifeq ($(shell uname), Darwin) # MacOS
	@echo "Device type: MacOS"
    PYTHON   := python3
    PIP      := pip3
    PYLINT   := pylint
    COVERAGE := coverage
    PYDOC    := pydoc3
    AUTOPEP8 := autopep8
else ifeq ($(shell uname -p), unknown) # Windows
    PYTHON   := python
    PIP      := pip3
    PYLINT   := pylint
    COVERAGE := coverage
    PYDOC    := python -m pydoc
    AUTOPEP8 := autopep8
else	
    @echo Device type not detected as MacOS or Windows
endif

init:
	pip install -r requirements.txt

clean:
	rm -rf 4_unit_tests/python_test_results
	mkdir 4_unit_tests/python_test_results
	find . -type d -name "__pycache__" -exec rm -rf {} +

test: 4_unit_tests/python_tests/
	-$(PYTHON) -m pytest --cov=src --cov-branch --cov=4_unit_tests/python_tests --cov-report=html:4_unit_tests/python_test_results/coverage_html \
	--html=4_unit_tests/python_test_results/test_results.html --self-contained-html --css=4_unit_tests/dark_mode.css
	find . -type d -name "__pycache__" -exec rm -rf {} +
	rm -rf .mypy_cache
	rm -rf .pytest_cache
	rm .coverage*
	setsid firefox 4_unit_tests/python_test_results/test_results.html
	setsid firefox 4_unit_tests/python_test_results/coverage_html/index.html

report:
	setsid firefox 4_unit_tests/python_test_results/test_results.html
	setsid firefox 4_unit_tests/python_test_results/coverage_html/index.html

test: clean test