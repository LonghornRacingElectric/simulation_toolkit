import math

from sim.system_models.vehicle_systems.vehicle_system_model import VehicleSystemModel

from sim.model_parameters.cars.car import Car
from sim.system_models.vectors.controls_vector import ControlsVector
from sim.system_models.vectors.observables_vector import ObservablesVector
from sim.system_models.vectors.state_vector import StateVector
from sim.system_models.vectors.state_dot_vector import StateDotVector
from sim.util.math.conversions import rpm_to_rads, rads_to_rpm


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
            # "v_long",  # this will be for cooling from external airflow later
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
            "hv_battery_open_circuit_voltage",
            "hv_battery_terminal_voltage",
            "lv_battery_open_circuit_voltage",
            "lv_battery_terminal_voltage",
        ]

    # TODO multi-motor configurations
    # TODO cooling from airflow (function of vehicle velocity)
    # TODO diff?
    # TODO cooling from infrared radiation??

    def eval(self, car: Car, controls_in: ControlsVector, state_in: StateVector,
             state_out: StateDotVector, observables_out: ObservablesVector):

        lv_system_power_out = car.lv_system_constant_power_draw + car.cooling_power_draw(controls_in.cooling_percent)
        hv_battery_cooling, hv_battery_cooling_from_coolant = self._calculate_cooling(0, 0, 0)  # TODO implement
        inverter_cooling, inverter_cooling_from_coolant = self._calculate_cooling(0, 0, 0)  # TODO implement
        motor_cooling, motor_cooling_from_coolant = self._calculate_cooling(0, 0, 0)  # TODO implement
        coolant_cooling = 0  # TODO calculate coolant cooling as a function of fan output
        coolant_heating = hv_battery_cooling_from_coolant + inverter_cooling_from_coolant + motor_cooling_from_coolant

        hv_battery_open_circuit_voltage = car.hv_battery_open_circuit_voltage(state_in.hv_battery_charge)
        hv_battery_internal_resistance = (
            car.hv_battery_internal_resistance(state_in.hv_battery_charge))  # , state_in.hv_battery_temperature))

        motor_torque = 0
        motor_back_emf = state_in.motor_rpm / car.motor_induced_voltage
        motor_efficiency = car.motor_efficiency(motor_torque)

        if controls_in.torque_request > 0:
            available_voltage = hv_battery_open_circuit_voltage - motor_back_emf
            available_current = available_voltage / (car.motor_winding_resistance + hv_battery_internal_resistance)
            rated_torque = car.motor_peak_torque(state_in.motor_rpm)

            possible_current = min(rated_torque / car.motor_kt, car.motor_peak_current, available_current)
            possible_power = car.hv_battery_nominal_voltage * possible_current
            available_power = min(possible_power, car.power_limit * car.inverter_efficiency * motor_efficiency)
            available_torque = available_power / rpm_to_rads(state_in.motor_rpm) if state_in.motor_rpm else 1e9

            motor_torque = min(controls_in.torque_request, rated_torque, available_torque)
        elif controls_in.torque_request < 0 and car.regen_enabled:
            raise NotImplementedError("regen not implemented yet, sorry :(")  # TODO regen!!

        motor_power_out = motor_torque * rpm_to_rads(state_in.motor_rpm)
        inverter_power_out = motor_power_out / motor_efficiency  # , state_in.motor_rpm)
        hv_battery_power_out = inverter_power_out / car.inverter_efficiency

        hv_battery_power_out += car.has_dcdc * (lv_system_power_out / car.dcdc_efficiency)
        lv_battery_power_out = (not car.has_dcdc) * lv_system_power_out

        hv_battery_current = self._calculate_battery_current(hv_battery_power_out, hv_battery_open_circuit_voltage,
                                                             hv_battery_internal_resistance)
        hv_battery_voltage_drop = hv_battery_internal_resistance * hv_battery_current
        hv_battery_terminal_voltage = hv_battery_open_circuit_voltage - hv_battery_voltage_drop

        lv_battery_open_circuit_voltage = car.lv_battery_open_circuit_voltage(state_in.lv_battery_charge)
        lv_battery_current = self._calculate_battery_current(lv_battery_power_out, lv_battery_open_circuit_voltage,
                                                             car.lv_battery_internal_resistance)
        lv_battery_voltage_drop = car.lv_battery_internal_resistance * lv_battery_current
        lv_battery_terminal_voltage = lv_battery_open_circuit_voltage - lv_battery_voltage_drop

        hv_battery_heat_loss = hv_battery_voltage_drop * hv_battery_current
        inverter_heat_loss = hv_battery_power_out - inverter_power_out
        motor_heat_loss = inverter_power_out - motor_power_out

        state_out.hv_battery_current = hv_battery_current
        state_out.lv_battery_current = lv_battery_current
        state_out.motor_torque = motor_torque
        state_out.hv_battery_net_heat = hv_battery_heat_loss - hv_battery_cooling
        state_out.inverter_net_heat = inverter_heat_loss - inverter_cooling
        state_out.motor_net_heat = motor_heat_loss - motor_cooling
        state_out.coolant_net_heat = coolant_heating - coolant_cooling
        state_out.applied_torque_fl = 0
        state_out.applied_torque_fr = 0
        state_out.applied_torque_bl = motor_torque * car.drivetrain_efficiency / 2  # TODO add diff lmao
        state_out.applied_torque_br = motor_torque * car.drivetrain_efficiency / 2

        observables_out.hv_battery_open_circuit_voltage = hv_battery_open_circuit_voltage
        observables_out.hv_battery_terminal_voltage = hv_battery_terminal_voltage
        observables_out.lv_battery_open_circuit_voltage = lv_battery_open_circuit_voltage
        observables_out.lv_battery_terminal_voltage = lv_battery_terminal_voltage
        # TODO add more observables from the existing variables

        if controls_in.torque_request > 0:
            pass

    def integrate(self, car: Car, state: StateVector, state_dot: StateDotVector, time_step: float):
        state.hv_battery_charge -= state_dot.hv_battery_current * time_step
        state.lv_battery_charge -= state_dot.lv_battery_current * time_step

        # TODO state.motor_rpm also needs rolling resistance from tire
        state.motor_rpm += rads_to_rpm(state_dot.motor_torque / car.drivetrain_moment_of_inertia) * time_step
        
        state.hv_battery_temperature += (state_dot.hv_battery_net_heat
                                         * car.hv_battery_thermal_resistance * time_step)
        state.inverter_temperature += (state_dot.inverter_net_heat
                                       * car.inverter_thermal_resistance * time_step)
        state.motor_temperature += (state_dot.motor_net_heat
                                    * car.hv_battery_thermal_resistance * time_step)
        state.coolant_temperature += (state_dot.coolant_net_heat
                                      * car.hv_battery_thermal_resistance * time_step)

    def _calculate_battery_current(self, battery_power: float, battery_open_circuit_voltage: float,
                                   battery_internal_resistance: float) -> float:
        """
        solves for current given power output, open circuit voltage, and internal resistance

        i = p / v
        v = v_oc - i*r
        i = p / (v_oc - i*r)
        (-r)i^2 + (v_oc)i - (p) = 0

        quadratic equation

        i = (-v_oc +/- sqrt(v_oc**2 - 4*r*p)))/(-2r)
        i = (v_oc +/- sqrt(v_oc**2 - 4*r*p)))/(2r)

        we only want the smaller i, the larger one is extraneous
        """
        # TODO rethink this with regen, as power will be negative and some sign changes will be necessary

        discriminant = (battery_open_circuit_voltage * battery_open_circuit_voltage
                        - 4 * battery_internal_resistance * battery_power)
        if discriminant < 0:
            raise Exception("too much current! not sure what to do now. maybe try not flooring it bruh")
        i = (battery_open_circuit_voltage - math.sqrt(discriminant)) / 2 / battery_internal_resistance
        return i

    def _calculate_cooling(self, object_temp: float, coolant_temp: float, coolant_area: float) -> (float, float):
        # TODO cooling calculations!!
        return 0, 0
