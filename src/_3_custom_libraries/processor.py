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

        self.filtered_params: dict[str, dict] = self.raw_params

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