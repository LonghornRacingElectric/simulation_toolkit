from collections import defaultdict

from sim.model_parameters.parameters.parameter import Parameter


class ModelParameters:
    def __init__(self):
        self._model_type: str = "Abstract Model"
        self._model_name: str = ""
        self._parameters_locked = False
        self._parameters_set_count = defaultdict(int)

    def __contains__(self, parameter_name: str):
        return hasattr(self, parameter_name)

    def __setattr__(self, key: str, value):
        if key[0] == "_":
            super().__setattr__(key, value)
            return
        if not isinstance(value, Parameter):
            raise Exception(f"{self._model_type} \"{self._model_name}\" was passed {key} = {value}, "
                            + "which isn't a valid parameter type.")
        if self._parameters_locked and self._parameters_set_count[key] == 0:
            raise Exception(f"{self._model_type} \"{self._model_name}\" tried to set parameter \"{key}\", "
                            + "but it doesn't exist.")
        super().__setattr__(key, value)
        self._parameters_set_count[key] += 1

    def __getattribute__(self, key: str):
        if key[0] == "_" or key == "driver_program":
            return super().__getattribute__(key)
        if self._parameters_set_count[key] < 2:
            if self._parameters_set_count[key] == 0:
                raise Exception(f"{self._model_type} \"{self._model_name}\" was asked for parameter \"{key}\", "
                                + "but it doesn't exist.")
            else:
                raise Exception(f"{self._model_type} \"{self._model_name}\" was asked for parameter \"{key}\", "
                                + "but it hasn't been set.")
        return super().__getattribute__(key).get()

    def __setitem__(self, key: str, value):
        self.__setattr__(key, value)

    def _lock(self):
        self._parameters_locked = True
