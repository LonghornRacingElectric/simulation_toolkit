# from LHR_tire_toolkit.MF52 import MF52
# import numpy as np

# test = MF52(tire_name='test', file_path='tire.tir')

# print(test.tire_eval(FZ=1000, alpha=0 * np.pi / 180, kappa=0, gamma=0 * np.pi / 180))

from vehicle_model.vehicle_model import VehicleModel

new_vehicle = VehicleModel(defn_file_path="model_inputs/sample_vehicle.yml", tir_file_path="model_inputs/hoosier_16_7p5_R25B_12psi.tir")