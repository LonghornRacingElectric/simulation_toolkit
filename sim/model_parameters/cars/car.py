from sim.model_parameters.parameters.constant_parameter import ConstantParameter
from sim.model_parameters.model_parameters import ModelParameters
from sim.model_parameters.parameters.curve_parameter import CurveParameter
from sim.model_parameters.parameters.toggle_parameter import ToggleParameter


class Car(ModelParameters):
    def __init__(self):
        super().__init__()
        self._model_type = "Car"
        self._model_name = "Unnamed Car"

        # ======================
        # ===== Powertrain =====
        # ======================
        # TODO multi-motor configurations
        # TODO cooling from airflow (function of vehicle velocity)
        # TODO diff?
        # TODO cooling from infrared radiation??

        # ----- Battery -----
        self.hv_battery_capacity = ConstantParameter()  # [J]
        self.hv_battery_open_circuit_voltage = CurveParameter()  # [V] = f(SoC [J])
        self.hv_battery_internal_resistance = CurveParameter()  # [R] = f(SoC [J], temp [C])
        self.hv_battery_thermal_resistance = ConstantParameter()  # [C/W]
        self.lv_battery_capacity = ConstantParameter()  # [J]
        self.has_dcdc = ToggleParameter()  # [true/false]
        self.dcdc_efficiency = ConstantParameter()  # [%]

        # ----- Cooling -----
        self.coolant_amount = ConstantParameter()  # [kg]
        self.coolant_flow_rate = ConstantParameter()  # [kg/s]
        self.coolant_thermal_resistance = ConstantParameter()  # [C/W]
        self.cooling_constant_power = ConstantParameter()  # [W]
        self.cooling_variable_power = ConstantParameter()  # [W]

        # ----- Drivetrain -----
        self.power_limit = ConstantParameter()  # [W]
        self.motor_peak_torque = CurveParameter()  # [Nm] = f(RPM [RPM])
        self.motor_peak_current = ConstantParameter()  # [A]
        self.motor_torque_current_factor = ConstantParameter()  # [Nm/A]
        self.motor_efficiency = CurveParameter()  # [%] = f(torque [Nm], RPM [RPM])
        self.motor_thermal_resistance = ConstantParameter()  # [C/W]
        self.drivetrain_efficiency = ConstantParameter()  # [%]
        self.inverter_efficiency = ConstantParameter()  # [%]
        self.inverter_thermal_resistance = ConstantParameter()  # [C/W]

        self._lock()
