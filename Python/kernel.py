from simulations.visual.visual import VisualModel
from simulations.kin.kin import Kinematics
from simulations.qss.qss import QSS

import shutil
import yaml
import sys
import os

# Initialization
avail_sims = ["kin", "visual", "qss"]
input_dir = "./_1_model_inputs/"

try:
    sim_selected = sys.argv[1].lower()
    model_path = input_dir + sys.argv[2]
except:
    raise Exception("Please specify SIM arg: make sim SIM={} MODEL_PATH={}")

# Validation
if sim_selected not in avail_sims:
    raise Exception(f"Selected simulation is not available. Use the following command: make sim SIM=()\nWhere () is replaced with one of: {', '.join(avail_sims)}")

# Print model information
with open(model_path) as f:
    try:
        model_properties: dict[str, dict[str, dict]] = yaml.safe_load(f)
    except yaml.YAMLError as error:
        print("Failed to import yaml file. Reason:\n")
        print(error)

print(f"\nSelected Model: {model_properties["Name"]["Value"]}")

# Select simulation
if sim_selected == "kin":
    print("Running simulation: kinematics")
    kin = Kinematics(model_path=model_path)
elif sim_selected == "visual":
    print("Running simulation: visual model")
    visual = VisualModel(model_path=model_path)
elif sim_selected == "qss":
    print("Running simulation: quasi-steady-state metrics")
    # Generate animation
    shutil.rmtree("./simulations/qss/qss_outputs/ymd_animation")
    os.mkdir("./simulations/qss/qss_outputs/ymd_animation")
    visual = QSS(model_path=model_path)