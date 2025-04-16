from simulations.kin.kin import Kinematics

import sys

avail_sims = ["kin"]

sim_selected = sys.argv[1].lower()

if sim_selected not in avail_sims:
    raise Exception(f"Selected simulation is not available. Use the following command: make sim SIM=()\nWhere () is replaced with one of: {', '.join(avail_sims)}")

if sim_selected == "kin":
    print("Running simulation: kinematics")
    kin = Kinematics()