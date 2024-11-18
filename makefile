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

clean:
	rm -rf unit_tests/python_test_results
	mkdir unit_tests/python_test_results

test_results: unit_tests/python_tests/
	$(PYTHON) -m pytest --cov=src --cov-branch --cov=unit_tests/python_tests --cov-report=html:unit_tests/python_test_results/coverage_html \
    --html=unit_tests/python_test_results/test_results.html --self-contained-html --css=unit_tests/dark_mode.css
	firefox unit_tests/python_test_results/test_results.html
	firefox unit_tests/python_test_results/coverage_html/index.html

test: clean test_results
