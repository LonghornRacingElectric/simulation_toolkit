from vehicle_model.suspension_model.suspension_data import SuspensionData
from vehicle_model.suspension_model.suspension import Suspension

import pyvista as pv
import time

sus_data = SuspensionData(path="./1_model_inputs/Nightwatch.yml")
sus = Suspension(sus_data=sus_data)
