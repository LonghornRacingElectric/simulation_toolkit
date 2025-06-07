from simulations.ymd_cv.ymd_cv import YMDConstantVelocity
from simulations.ymd_cr.ymd_cr import YMDConstantRadius
from simulations.visual.visual import VisualModel
from simulations.kin.kin import Kinematics

import yaml
import sys


# Initialization
avail_sims = ["kin", "visual", "ymd_cr", "ymd_cv"]

try:
    sim_selected = sys.argv[1].lower()
    model_path = sys.argv[2]
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
elif sim_selected == "ymd_cr":
    print("Running simulation: yaw-moment model, constant radius")
    visual = YMDConstantRadius(model_path=model_path)
elif sim_selected == "ymd_cv":
    print("Running simulation: yaw-moment model, constant velocity")
    visual = YMDConstantVelocity(model_path=model_path)