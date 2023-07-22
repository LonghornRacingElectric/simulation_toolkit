import math

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
            "hv_battery_charge",
            "lv_battery_charge",
            "motor_rpm",
            "hv_battery_temperature",
            "inverter_temperature",
            "motor_temperature",
            "coolant_temperature",
            "v_long",
        ]

        self.state_out = [
            "hv_battery_current",
            "lv_battery_current",
            "motor_torque",
            "hv_battery_net_heat",
            "inverter_net_heat",
            "motor_net_heat",
            "coolant_net_heat",
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

        lv_system_power_out = car.lv_system_constant_power_draw + car.cooling_power_draw(controls_in.cooling_percent)
        # TODO cooling calculations
        hv_battery_cooling = 0
        inverter_cooling = 0
        motor_cooling = 0

        available_torque = car.motor_peak_torque(state_in.motor_rpm)
        possible_current = min(available_torque / car.motor_torque_current_factor, car.motor_peak_current)
        possible_power = car.hv_battery_nominal_voltage * possible_current
        max_power = min(possible_power, car.power_limit)
        max_torque_from_power_limit = max_power / state_in.motor_rpm

        motor_torque = min(controls_in.torque_request, available_torque, max_torque_from_power_limit)
        motor_current = max(motor_torque / car.motor_torque_current_factor, car.motor_peak_current)
        # TODO investigate relationship between voltage and motor torque

        motor_power_out = motor_torque * state_in.motor_rpm
        inverter_power_out = motor_power_out / car.motor_efficiency(motor_torque, state_in.motor_rpm)
        hv_battery_power_out = inverter_power_out / car.inverter_efficiency

        hv_battery_power_out += car.has_dcdc * (lv_system_power_out / car.dcdc_efficiency)
        lv_battery_power_out = (not car.has_dcdc) * lv_system_power_out

        hv_battery_open_circuit_voltage = car.hv_battery_open_circuit_voltage(state_in.hv_battery_charge)
        hv_battery_internal_resistance = \
            car.hv_battery_internal_resistance(state_in.hv_soc, state_in.hv_battery_temperature)
        hv_battery_current = self._mini_solve_hv_battery_current(hv_battery_power_out, hv_battery_open_circuit_voltage,
                                                                 hv_battery_internal_resistance)
        # TODO what if hv_battery_current is less than motor_current??

        hv_battery_voltage_drop = hv_battery_internal_resistance * hv_battery_current
        hv_battery_terminal_voltage = hv_battery_open_circuit_voltage - hv_battery_voltage_drop

        hv_battery_heat_loss = hv_battery_voltage_drop * hv_battery_current
        inverter_heat_loss = hv_battery_power_out - inverter_power_out
        motor_heat_loss = inverter_power_out - motor_power_out

        state_out.hv_battery_net_heat = hv_battery_heat_loss - hv_battery_cooling
        state_out.inverter_net_heat = inverter_heat_loss - inverter_cooling
        state_out.motor_net_heat = motor_heat_loss - motor_cooling


    def _mini_solve_hv_battery_current(self, hv_battery_power: float, hv_battery_open_circuit_voltage: float,
                                       hv_battery_internal_resistance: float) -> float:
        """
        solves for current given power output, open circuit voltage, and internal resistance

        i = p / v
        v = v_oc - i*r
        i = p / (v_oc - i*r)
        (-r)i^2 + (v_oc)i - (p) = 0

        quadratic equation

        i = (-v_oc +/- sqrt(v_oc**2 - 4*r*p)))/(-2r)
        i = (v_oc +/- sqrt(v_oc**2 - 4*r*p)))/(2r)
        i = v_oc/(2r) +/- sqrt(v_oc**2 - 4*r*p)/(2r)
        i = a +/- b (and we only want the smaller one, the larger one is extraneous)
        """
        # TODO rethink this with regen, as power will be negative and some sign changes will be necessary

        a = hv_battery_open_circuit_voltage / 2 / hv_battery_internal_resistance
        discriminant = (hv_battery_open_circuit_voltage * hv_battery_open_circuit_voltage
                        - 4 * hv_battery_internal_resistance * hv_battery_power)
        if discriminant < 0:
            raise Exception("too much current! not sure what to do now. maybe try not flooring it bruh")
        b = math.sqrt(discriminant) / 2 / hv_battery_internal_resistance
        i = a - b
        return i
