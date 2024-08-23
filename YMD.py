from vehicle_model.vehicle_model import VehicleModel
from simulations.yaw_moment import YMD

new_vehicle = VehicleModel(defn_file_path="model_inputs/sample_vehicle.yml", tir_file_path="model_inputs/hoosier_16_7p5_R25B_12psi.tir")
ymd_generator = YMD(vehicle=new_vehicle)

ymd_generator.generate_constant_velocity_YMD(velocity=25)