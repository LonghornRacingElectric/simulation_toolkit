from sim.model_parameters.cars.car import Car
from sim.system_models.vectors.controls_vector import ControlsVector
from sim.system_models.vectors.driver_controls_vector import DriverControlsVector
from sim.system_models.vectors.vcu_output_vector import VcuOutputVector


class ControlsMux:

    def __init__(self):
        pass

    def eval(self, driver_controls: DriverControlsVector, vcu_controls: VcuOutputVector) -> ControlsVector:
        out = ControlsVector()

        # VCU controls
        out.torque_request = vcu_controls.torque_request if vcu_controls.enable_inverter else 0
        out.enable_drs = vcu_controls.enable_drs
        out.cooling_percent = 0  # TODO not supported yet

        # mechanical controls
        out.brake_pedal_pressure = driver_controls.brake_pedal_pct
        out.steering_angle = driver_controls.steering_angle

        return out
