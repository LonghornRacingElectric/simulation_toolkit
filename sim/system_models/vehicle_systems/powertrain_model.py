from sim.system_models.vehicle_systems.vehicle_system_model import VehicleSystemModel

from sim.model_parameters.cars.car import Car
from sim.system_models.vectors.controls_vector import ControlsVector
from sim.system_models.vectors.observables_vector import ObservablesVector
from sim.system_models.vectors.state_vector import StateVector
from sim.system_models.vectors.state_dot_vector import StateDotVector


# TODO this entire thing is temporary for testing, do not actually use

class PowertrainModel(VehicleSystemModel):
    def __init__(self):
        super().__init__()

        self.controls_in = [
            "torque_request",
            "cooling_percent",
        ]

        self.state_in = [
            "hv_soc",
            "lv_soc",
            "v_long",
            "motor_rpm",
            "hv_battery_temperature",
            "inverter_temperature",
            "motor_temperature",
            "coolant_temperature",
        ]

        self.state_out = [
            "applied_torque_fl",
            "applied_torque_fr",
            "applied_torque_rl",
            "applied_torque_rr",
        ]

        self.observables_out = [
            "available_torque"
        ]

    def eval(self, car: Car, controls_in: ControlsVector, state_in: StateVector,
             state_out: StateDotVector, observables_out: ObservablesVector):

        hv_battery_open_circuit_voltage = car.hv_battery_open_circuit_voltage(state_in.hv_soc)
        hv_battery_internal_resistance = \
            car.hv_battery_internal_resistance(state_in.hv_soc, state_in.hv_battery_temperature)

        available_torque = car.motor_peak_torque(state_in.motor_rpm)
        max_current = min(available_torque / car.motor_torque_current_factor, car.motor_peak_current)
        hv_battery_terminal_voltage = hv_battery_open_circuit_voltage - hv_battery_internal_resistance *

        motor_torque = min(available_torque, controls_in.torque_request, ____)

        # TODO also consider max power requirement

        # motor_current = torque_command / car.motor_torque_current_factor
        # inverter_current = motor_current / car.motor_efficiency(torque_command, state_in.motor_rpm)
        # battery_current = inverter_current / car.inverter_efficiency

        motor_power_out = motor_torque * state_in.motor_rpm
        inverter_power_out = motor_power_out / car.motor_efficiency(motor_torque, state_in.motor_rpm)
        battery_power_out = inverter_power_out / car.inverter_efficiency

        # TODO figure out voltage relationship

