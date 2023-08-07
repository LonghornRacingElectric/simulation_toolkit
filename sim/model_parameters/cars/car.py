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
        # ======= General ======
        # ======================

        self.sprung_inertia = ConstantParameter() # kgm^2 (inertia tensor)
        self.gravity = ConstantParameter() # m/s^2
        self.cg_bias = ConstantParameter() # m (percent from front. 0 -> frontmost, 1 -> rearmost)
        self.cg_left = ConstantParameter() # m (percent from left. 0 -> leftmost, 1 -> rightmost)
        self.cg_height = ConstantParameter() # m
        self.wheelbase = ConstantParameter() # m
        self.front_track = ConstantParameter() # m
        self.rear_track = ConstantParameter() # m

        self.front_unsprung_mass = ConstantParameter() # kg
        self.rear_unsprung_mass = ConstantParameter() # kg
        self.driver_mass = ConstantParameter() # kg
        self.mass_sprung = ConstantParameter() # kg

        # ======================
        # ===== Suspension =====
        # ======================

        self.decoupled = ConstantParameter() # boolean

        # Decoupled rate inputs
        self.front_heave_springrate = ConstantParameter() # N/mm^2
        self.front_roll_springrate = ConstantParameter() # N/mm^2
        self.rear_heave_springrate = ConstantParameter() # N/mm^2
        self.rear_roll_springrate = ConstantParameter() # N/mm^2

        self.front_heave_MR = CurveParameter() # unitless
        self.front_roll_MR = CurveParameter() # unitless

        self.rear_heave_MR = CurveParameter() # unitless
        self.rear_roll_MR = CurveParameter() # unitless

        # Coupled rate inputs
        self.front_springrate = ConstantParameter() # N/mm^2
        self.rear_springrate = ConstantParameter() # N/mm^2

        self.front_ARB_rate = ConstantParameter() # Nm/rad
        self.rear_ARB_rate = ConstantParameter() # Nm/rad

        self.front_ARB_MR = ConstantParameter() # unitless
        self.rear_ARB_MR = ConstantParameter() # unitless

        self.front_MR = CurveParameter() # unitless
        self.rear_MR = CurveParameter() # unitless

        # Shared inputs

        self.front_anti = ConstantParameter() # percent
        self.rear_anti = ConstantParameter() # percent

        self.ackermann = ConstantParameter() # percent

        # Kinematics

        self.static_IA = ConstantParameter() # rad, list[float] (FL, FR, RL, RR)
        self.front_roll_camber_gain = CurveParameter() # deg/rad list[float] (FL, FR, RL, RR)
        self.front_bump_camber_gain = CurveParameter() # deg/m list[float] (FL, FR, RL, RR)
        self.rear_roll_camber_gain = CurveParameter() # deg/rad list[float] (FL, FR, RL, RR)
        self.rear_bump_camber_gain = CurveParameter() # deg/m list[float] (FL, FR, RL, RR)

        self.front_KPI = ConstantParameter() # rad
        self.rear_KPI = ConstantParameter() # rad
        self.front_caster = ConstantParameter() # rad
        self.rear_caster = ConstantParameter() # rad

        self.toe = ConstantParameter() # rad, list[float] (FL, FR, RL, RR)
        self.front_bump_steer = CurveParameter() # rad/m
        self.front_roll_steer = CurveParameter() # rad/rad
        self.rear_bump_steer = CurveParameter() # rad/m
        self.rear_roll_steer = CurveParameter() # rad/rad

        self.front_roll_center_height = CurveParameter() # m
        self.rear_roll_center_height = CurveParameter() # m

        # Tires

        self.front_tire_coeff_Fy = ConstantParameter() # coeffs
        self.front_tire_coeff_Fx = ConstantParameter() # coeffs
        self.rear_tire_coeff_Fy = ConstantParameter() # coeffs
        self.rear_tire_coeff_Fx = ConstantParameter() # coeffs

        self.front_tire_vertical_rate = CurveParameter() # N/mm^2
        self.rear_tire_vertical_rate = CurveParameter() # N/mm^2

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
        self.coolant_thermal_resistance = ConstantParameter()  # [C/W]
        self.cooling_power_draw = CurveParameter()  # [W] = f(cooling [%])
        self.coolant_area_cooling = ConstantParameter()  # [m^2]
        self.coolant_area_hv_battery = ConstantParameter()  # [m^2]
        self.coolant_area_inverter = ConstantParameter()  # [m^2]
        self.coolant_area_motor = ConstantParameter()  # [m^2]

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
        self.inverter_efficiency = ConstantParameter()  # [%]
        self.inverter_thermal_resistance = ConstantParameter()  # [C/W]
        self.drivetrain_efficiency = ConstantParameter()  # [%]
        self.drivetrain_moment_of_inertia = ConstantParameter()  # [kgm^2]

        self._lock()
