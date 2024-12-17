import yaml


class Processor:
    """
    ## Processor

    Processor for vehicle definition files (yaml)

    Parameters
    ----------
    file_path : str
        File path to vehicle yaml file
    """
    def __init__(self, file_path: str) -> None:
        self.file_path = file_path

        with open(self.file_path) as f:
            try:
                self.raw_params: dict[str, dict[str, dict]] = yaml.safe_load(f)
            except yaml.YAMLError as error:
                print("Failed to import yaml file. Reason:\n")
                print(error)
        
        self.desired_params = ["Environment", "Mass Properties", "Geometric Properties", "Suspension"]
        self.desired_fields = ['Value']

        self.filtered_params: dict[str, dict] = {}

        for key, value in self.raw_params.items():
            if key in self.desired_params:
                self.filtered_params[key] = {}
                for param_key, param_vals in value.items():
                    self.filtered_params[key][param_key] = {}
                    for field_key, field_val in param_vals.items():
                        if field_key in self.desired_fields:
                            self.filtered_params[key][param_key] = field_val

    @property
    def params(self) -> dict:
        """
        ## Parameters

        Desired vehicle parameters

        Returns
        -------
        dict
            Dictionary of all desired parameters
        """
        return self.filtered_params