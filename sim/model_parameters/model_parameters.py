
class ModelParameters:
    def __init__(self):
        self._parameters: dict[str, type] = {}
        self.model_type: str = "Abstract Model"
        self.model_name: str = ""

    def _validate_parameters(self):
        for pname, ptype in self._parameters.items():
            if not hasattr(self, pname):  # TODO replace prints with logging system
                print(f"{self.model_type} \"{self.model_name}\" is missing parameter {pname} ({ptype.__name__}).")
            elif type(getattr(self, pname)) != ptype:
                print(f"{self.model_type} \"{self.model_name}\" parameter {pname} should be a {ptype.__name__}.")
