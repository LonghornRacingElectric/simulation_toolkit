import numpy as np
import matplotlib.pyplot as plt
from fmpy import simulate_fmu, read_model_description
from fmpy import extract

import os
import tempfile

class FMUSimulator:
    def __init__(self, fmu_path):
        self.fmu_path = fmu_path
        self.model_description = read_model_description(fmu_path)
        self.temp_dir = tempfile.mkdtemp()
        self.unpacked_fmu = extract(fmu_path, self.temp_dir)

    def get_variables(self):
        parameters = [v.name for v in self.model_description.modelVariables if v.causality == 'parameter' and "CG_height" in v.name]
        print(parameters)
        """List input, output, and parameter variables"""
        inputs = []
        outputs = []
        parameters = []

        for variable in self.model_description.modelVariables:
            if variable.variability == 'parameter':
                parameters.append(variable.name)
            elif variable.causality == 'input':
                inputs.append(variable.name)
            elif variable.causality == 'output':
                outputs.append(variable.name)

        return {
            'inputs': inputs,
            'outputs': outputs,
            'parameters': parameters
        }

    def simulate(self,
                 fmi_type,
                 start_time=0.0,
                 stop_time=1.0,
                 input_data=None,
                 start_values=None,
                 output_vars=None,
                 step_size=None,
                 debug_logging=False,
                 relative_tolerance = None
                 ):
        """
        Simulate the FMU with optional input signals and parameter settings.
        """

        # Build simulation options
        sim_args = {
            'filename': self.fmu_path,
            'start_time': start_time,
            'stop_time': stop_time,
            'apply_default_start_values': False  # To ensure parameters override defaults
        }

        if input_data is not None:
            sim_args['input'] = input_data

        if start_values is not None:
            sim_args['start_values'] = start_values  # pass parameters as start_values

        if output_vars is not None:
            sim_args['output'] = output_vars

        if step_size is not None:
            sim_args['step_size'] = step_size

        if debug_logging:
            sim_args['debug_logging'] = True

        
        result = simulate_fmu(
            filename=self.fmu_path,
            start_time=start_time,
            stop_time=stop_time,
            input=input_data,
            output=output_vars,
            step_size=step_size,
            fmi_type=fmi_type,
            debug_logging=debug_logging,
            start_values=start_values or {},    
            apply_default_start_values=False,
            relative_tolerance=relative_tolerance
        )
        return result
