from sim.model_parameters.cars.car import Car
from sim.model_parameters.vcu.vcu import VehicleControlUnit
from sim.system_models.vectors.controls_vector import ControlsVector
from sim.system_models.vectors.driver_controls_vector import DriverControlsVector
from sim.system_models.vectors.sensor_data_vector import SensorDataVector


class VehicleControlUnitModel:
    """
    TODO driver controls, sensor data in --> vehicle controls out
    TODO for best results, bind to actual C++ code!
    """

    def __init__(self, car_parameters: Car, vcu_parameters: VehicleControlUnit):
        self.car = car_parameters
        self.parameters = vcu_parameters

        # TODO load C++ backend

    def eval(self, driver_controls: DriverControlsVector, sensor_data: SensorDataVector) -> ControlsVector:
        out = ControlsVector()

        # TODO replace temp shit example with call to C++ backend
        out.torque_request = driver_controls.accel_pedal_pct * 230

        if self.parameters.traction_control_enabled:
            out.brake_pedal_pressure = driver_controls.brake_pedal_pct * 200
        else:
            out.brake_pedal_pressure = driver_controls.brake_pedal_pct * 100

        return out
