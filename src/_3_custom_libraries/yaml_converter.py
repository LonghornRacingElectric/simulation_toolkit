import os
import json
import yaml
import collections
from copy import deepcopy
class YamlToJsonConverter:
    """
    Converts a YAML file into one or more JSON files.

    Example:
        converter = YamlToJsonConverter("config/settings.yaml")
        converter.convert(output_dir="output/json")
    """

    def __init__(self, yaml_path: str):
        """
        Initialize the converter with a YAML file path.
        """
        if not os.path.isfile(yaml_path):
            raise FileNotFoundError(f"YAML file not found: {yaml_path}")
        self.yaml_path = yaml_path
        self.data = None  # will hold parsed YAML content

    def load_yaml(self) -> None:
        """Load and parse the YAML file."""
        with open(self.yaml_path, "r", encoding="utf-8") as f:
            self.data = yaml.safe_load(f)

    def parse_raw(self) -> None:
        """ Parse YAML into structure JSON will be written in. """
        def recursively_default_dict():
            return collections.defaultdict(recursively_default_dict)

        json_data = recursively_default_dict()
        # General Info
        json_data["General_Info"]["General_Info"]["Name"] = self.data["Name"]
        json_data["General_Info"]["General_Info"]["Version"] = self.data["Version"]
        json_data["General_Info"]["General_Info"]["Coordinates"] = self.data["Coordinates"]
        # Environment
        json_data["Environment"]["Environment"] = self.data["Environment"]
        # Mass Properties
        json_data["Mass_Properties"]["Mass_Properties"] = self.data["Mass Properties"]
        # Brake Properties
        json_data["Brake_Properties"]["Brake_Properties"] = self.data["Brake Properties"]
        # Corners
        for corner_L, corner_R in list(zip(["FL QuarterCar", "RL QuarterCar"],["FR QuarterCar", "RR QuarterCar"])):
            # Tire data is same across about y-axis
            json_data[corner_L]["Tire"] = self.data[corner_L]["tire"]
            # Hardpoint data
            json_data[corner_L]["Hardpoints"] = {
                key:value for key, value in self.data[corner_L].items() if key != "tire"
            }
        
            json_data[corner_R] = deepcopy(json_data[corner_L])

            def negate_second_value(d):
                """
                Recursively traverse a nested dict (and lists within it),
                and if a key 'Value' has a list of 3 floats, negate the second element.
                """
                if isinstance(d, dict):
                    for key, val in d.items():
                        if key == "Value" and isinstance(val, list) and len(val) == 3 \
                                and all(isinstance(x, (float, int)) for x in val):
                            d[key][1] = -val[1]
                        else:
                            negate_second_value(val)

                elif isinstance(d, list):
                    for item in d:
                        negate_second_value(item)
            
            negate_second_value(json_data[corner_R])

        return json_data
    def convert(self, output_dir: str) -> None:
        """
        Convert YAML file to one or more JSON files.
        """
        os.makedirs(output_dir, exist_ok=True)
        self.load_yaml()
        chunks = self.parse_raw()

        for folder, _ in chunks.items():
            for file, data in chunks[folder].items():
                folder_path = os.path.join(output_dir, folder) 
                file_path = f"{folder_path}/{file}.json"
                os.makedirs(folder_path, exist_ok=True)
                json_str = json.dumps(chunks[folder][file], indent=4)
                with open(file_path, "w") as f:
                    f.write(json_str)