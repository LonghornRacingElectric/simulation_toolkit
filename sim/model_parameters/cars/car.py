from sim.model_parameters.parameters.constant_parameter import ConstantParameter
from sim.model_parameters.model_parameters import ModelParameters
from sim.model_parameters.parameters.curve_parameter import CurveParameter
from sim.model_parameters.parameters.toggle_parameter import ToggleParameter


# class Car(ModelParameters):
class Car:
    def __init__(self):
        super().__init__()
        self._model_type = "Car"
        self._model_name = "Unnamed Car"

        # ======================
        # ======= General ======
        # ======================

        self.sprung_inertia = ConstantParameter() # kgm^2 (inertia tensor)
        self.accel_gravity = ConstantParameter(9.81) # m/s^2
        self.cg_bias = ConstantParameter(0.52) # m (percent from front. 0 -> frontmost, 1 -> rearmost)
        self.cg_left = ConstantParameter(0.50) # m (percent from left. 0 -> leftmost, 1 -> rightmost)
        self.cg_height = ConstantParameter(0.28448) # m
        self.wheelbase = ConstantParameter(1.5494) # m
        self.front_track = ConstantParameter(1.27) # m
        self.rear_track = ConstantParameter(1.2192) # m

        self.front_unsprung_mass = ConstantParameter(16.328) # kg of entire axle
        self.rear_unsprung_mass = ConstantParameter(18.611) # kg of entire axle
        self.driver_mass = ConstantParameter(70) # kg
        self.mass_sprung = ConstantParameter(170) # kg
        self.total_mass = ConstantParameter(self.front_unsprung_mass.get() + self.rear_unsprung_mass.get() + 
                                            self.driver_mass.get() + self.mass_sprung.get()) # kg
        self.tire_positions = ConstantParameter([[self.wheelbase.get() * self.cg_bias.get(), -self.front_track.get() / 2, 0],
                                                 [self.wheelbase.get() * self.cg_bias.get(), self.front_track.get() / 2, 0],
                                                 [-self.wheelbase.get() * (1 - self.cg_bias.get()), -self.rear_track.get() / 2, 0],
                                                 [-self.wheelbase.get() * (1 - self.cg_bias.get()), self.rear_track.get() / 2, 0]]) # [FL, FR, RL, RR]

        # ======================
        # ===== Suspension =====
        # ======================

        self.decoupled = ConstantParameter(True) # boolean

        # Decoupled rate inputs
        self.front_heave_springrate = ConstantParameter(300 * 175.127) # N/m
        self.front_roll_springrate = ConstantParameter(450 * 175.127) # N/m
        self.rear_heave_springrate = ConstantParameter(400 * 175.127) # N/m
        self.rear_roll_springrate = ConstantParameter(400 * 175.127) # N/m

        self.front_heave_MR = ConstantParameter(1 / 1.382) # unitless, but make these curve parameters once we add functionality
        self.front_roll_MR = ConstantParameter(1 / 1.137) # unitless, but make these curve parameters once we add functionality

        self.rear_heave_MR = ConstantParameter(1 / 1.271) # unitless, but make these curve parameters once we add functionality
        self.rear_roll_MR = ConstantParameter(1 / 1.034) # unitless, but make these curve parameters once we add functionality

        # Coupled rate inputs
        self.front_springrate = ConstantParameter() # N/m
        self.rear_springrate = ConstantParameter() # N/m

        self.front_ARB_rate = ConstantParameter() # Nm/rad
        self.rear_ARB_rate = ConstantParameter() # Nm/rad

        self.front_ARB_MR = ConstantParameter() # unitless
        self.rear_ARB_MR = ConstantParameter() # unitless

        self.front_MR = CurveParameter() # unitless
        self.rear_MR = CurveParameter() # unitless

        # Shared inputs

        self.front_anti = ConstantParameter(0.20) # percent
        self.rear_anti = ConstantParameter(0.00) # percent

        self.ackermann = ConstantParameter() # percent

        # Kinematics

        self.static_IAs = ConstantParameter([1 * 0.01745, -1 * 0.01745, 1 * 0.01745, -1 * 0.01745]) # rad, list[float] (FL, FR, RL, RR)
        self.roll_IA_gain = ConstantParameter([0, 0, 0, 0]) # deg/rad list[float] (FL, FR, RL, RR), but make these curve parameters once we add functionality
        self.heave_IA_gain = ConstantParameter([0, 0, 0, 0]) # deg/m list[float] (FL, FR, RL, RR), but make these curve parameters once we add functionality

        self.front_KPI = ConstantParameter() # rad
        self.rear_KPI = ConstantParameter() # rad
        self.front_caster = ConstantParameter() # rad
        self.rear_caster = ConstantParameter() # rad

        self.toe_angles = ConstantParameter([0, 0, 0, 0]) # rad, list[float] (FL, FR, RL, RR)
        self.front_bump_steer = ConstantParameter(0) # rad/m, but make these curve parameters once we add functionality
        self.front_roll_steer = ConstantParameter(0) # rad/rad, but make these curve parameters once we add functionality
        self.rear_bump_steer = ConstantParameter(0) # rad/m, but make these curve parameters once we add functionality
        self.rear_roll_steer = ConstantParameter(0) # rad/rad, but make these curve parameters once we add functionality

        self.front_roll_center_height = CurveParameter() # m
        self.rear_roll_center_height = CurveParameter() # m

        self.rack_clevis_to_kingpin_y = ConstantParameter(16.115 * 0.0254) # m
        self.rack_clevis_to_kingpin_x = ConstantParameter(2.203 * 0.0254) # m
        self.tie_linkage_length = ConstantParameter(15.555 * 0.0254) # m
        self.steering_arm = ConstantParameter(2.207 * 0.0254) # m
        self.c_factor = ConstantParameter(3.46 * 0.0254) # m/rev


        # Tires
        self.front_tire_coeff_Fy = ConstantParameter([0.349, -0.00115, 8.760, 730.300, 1745.322, 0.0139, -0.000277, 1.02025435, 0, 0, 0, 0, 0, 0, 0, 0.00362, -0.0143, -0.0116]) # coeffs
        self.front_tire_coeff_Fx = ConstantParameter([0.46024966176377113, 4000.509873697152, 1097.1712081460967, 202.18848632159495, 100.8812198037175, -0.2557010431649166, 0.3066955241461764, 0.011822770671297778, -1.9521015799737094, 0, 0, 0, 0, 0]) # coeffs
        self.rear_tire_coeff_Fy = ConstantParameter([0.349, -0.00115, 8.760, 730.300, 1745.322, 0.0139, -0.000277, 1.02025435, 0, 0, 0, 0, 0, 0, 0, 0.00362, -0.0143, -0.0116]) # coeffs
        self.rear_tire_coeff_Fx = ConstantParameter([0.46024966176377113, 4000.509873697152, 1097.1712081460967, 202.18848632159495, 100.8812198037175, -0.2557010431649166, 0.3066955241461764, 0.011822770671297778, -1.9521015799737094, 0, 0, 0, 0, 0]) # coeffs

        self.front_tire_vertical_rate = ConstantParameter(725 * 175.127) # N/m, but make these curve parameters once we add functionality
        self.rear_tire_vertical_rate = ConstantParameter(725 * 175.127) # N/m, but make these curve parameters once we add functionality

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

        # self._lock()
