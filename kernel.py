from src.simulations.visual.visual import VisualModel
from src.simulations.kin.kin import Kinematics
from src.simulations.qss.qss import QSS
from src.simulations.comp_eval.comp_eval import CompEval

import shutil
import time
import yaml
import sys
import os

start_time = time.time()

# Initialization
avail_sims = ["kin", "visual", "qss", "comp_eval"]
input_dir = "./src/_1_model_inputs/"

try:
    sim_selected = sys.argv[1].lower()
    model_path = input_dir + sys.argv[2]

    if sim_selected == "kin":
        comparison_paths = [input_dir + path for path in sys.argv[3:]]

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
    kin = Kinematics(model_path=model_path, comparison_paths=comparison_paths)
elif sim_selected == "visual":
    print("Running simulation: visual model")
    visual = VisualModel(model_path=model_path)
elif sim_selected == "qss":
    print("Running simulation: quasi-steady-state metrics")
    # Generate animation
    shutil.rmtree("./src/simulations/qss/qss_outputs/ymd_animation")
    os.mkdir("./src/simulations/qss/qss_outputs/ymd_animation")
    visual = QSS(model_path=model_path)
elif sim_selected == "comp_eval":
    print("Running simulation: comp evaluation")
    comp_eval = CompEval(model_path=model_path)


end_time = time.time()

print(f"Workflow duration: {end_time - start_time} sec")