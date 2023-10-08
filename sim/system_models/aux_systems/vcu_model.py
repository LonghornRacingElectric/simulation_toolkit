from libs.vcu_core.vcu_core_process import VcuCoreProcess
from sim.model_parameters.cars.car import Car
from sim.model_parameters.vcu.vcu import VehicleControlUnit
from sim.system_models.vectors.sensor_data_vector import SensorDataVector
from sim.system_models.vectors.vcu_output_vector import VcuOutputVector


class VehicleControlUnitModel:
    """
    TODO driver controls, sensor data in --> vehicle controls out
    """

    def __init__(self, car_parameters: Car, vcu_parameters: VehicleControlUnit):
        self.car = car_parameters
        self.parameters = vcu_parameters
        self.core = VcuCoreProcess()

    def eval(self, sensor_data: SensorDataVector) -> VcuOutputVector:
        out = self.core.execute(sensor_data)
        return out

