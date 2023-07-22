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

        # ----- Battery -----
        self.hv_battery_capacity = ConstantParameter()  # [As]
        self.hv_battery_nominal_voltage = ConstantParameter()  # [V]
        self.hv_battery_open_circuit_voltage = CurveParameter()  # [V] = f(SoC [As])
        self.hv_battery_internal_resistance = CurveParameter()  # [Ohms] = f(SoC [As], temp [C])
        self.hv_battery_thermal_resistance = ConstantParameter()  # [C/W]
        self.lv_battery_capacity = ConstantParameter()  # [As]
        self.lv_battery_open_circuit_voltage = CurveParameter()  # [V] = f(SoC [As])
        self.lv_battery_internal_resistance = ConstantParameter()  # [Ohms]
        self.lv_system_constant_power_draw = ConstantParameter()  # [W]
        self.has_dcdc = ToggleParameter()  # [true/false]
        self.dcdc_efficiency = ConstantParameter()  # [%]

        # ----- Cooling -----
        self.coolant_amount = ConstantParameter()  # [kg]
        self.coolant_flow_rate = ConstantParameter()  # [kg/s]
        self.coolant_thermal_resistance = ConstantParameter()  # [C/W]
        self.cooling_power_draw = CurveParameter()  # [W] = f(cooling [%])

        # ----- Drivetrain -----
        self.regen_enabled = ToggleParameter()  # [true/false]
        self.power_limit = ConstantParameter()  # [W]
        self.motor_peak_torque = CurveParameter()  # [Nm] = f(RPM [RPM])
        self.motor_peak_current = ConstantParameter()  # [A]
        self.motor_winding_resistance = ConstantParameter()  # [Ohms]
        self.motor_kt = ConstantParameter()  # [Nm/A]
        self.motor_kv = ConstantParameter()  # [RPM/Vp]
        self.motor_efficiency = CurveParameter()  # [%] = f(torque [Nm], RPM [RPM])
        self.motor_thermal_resistance = ConstantParameter()  # [C/W]
        self.drivetrain_efficiency = ConstantParameter()  # [%]
        self.inverter_efficiency = ConstantParameter()  # [%]
        self.inverter_thermal_resistance = ConstantParameter()  # [C/W]

        self._lock()
