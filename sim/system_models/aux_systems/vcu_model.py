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

    def eval(self, sensor_data: SensorDataVector) -> VcuOutputVector:
        out = VcuOutputVector()

        out.enable_inverter = True
        out.torque_request = 230

        return out


def output_to_core(core_input: SensorDataVector):
    pass


def input_from_core(core_output: VcuOutputVector):
    pass
