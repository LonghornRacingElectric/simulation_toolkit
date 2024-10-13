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
	rm -rf unit_test_results
	rm -rf model_outputs

sim_setup:
	mkdir model_outputs

unit_test_results:
	mkdir unit_test_results

unit_test_results/test_node.tmp: vehicle_model/suspension_model/suspension_elements/primary_elements/node.py
	python3 unit_tests/test_node.py

run_tests: unit_test_results unit_test_results/test_node.tmp

run_sim: sim_setup