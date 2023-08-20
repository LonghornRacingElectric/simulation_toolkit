from sim.model_parameters.parameters import ConstantParameter, CurveParameter, ToggleParameter
from sim.model_parameters.model_parameters import ModelParameters


class Car(ModelParameters):
    def __init__(self):
        super().__init__()
        self._model_type = "Car"
        self._model_name = "Unnamed Car"

        # ======================
        # ======= General ======
        # ======================

        self.sprung_inertia = ConstantParameter()  # kgm^2 (inertia tensor)
        self.accel_gravity = ConstantParameter()  # m/s^2
        self.cg_bias = ConstantParameter()  # m (percent from front. 0 -> frontmost, 1 -> rearmost)
        self.cg_left = ConstantParameter()  # m (percent from left. 0 -> leftmost, 1 -> rightmost)
        self.cg_height = ConstantParameter()  # m
        self.wheelbase = ConstantParameter()  # m
        self.front_track = ConstantParameter()  # m
        self.rear_track = ConstantParameter()  # m

        self.front_unsprung_mass = ConstantParameter()  # kg of entire axle
        self.rear_unsprung_mass = ConstantParameter()  # kg of entire axle
        self.driver_mass = ConstantParameter()  # kg
        self.mass_sprung = ConstantParameter()  # kg
        self.total_mass = ConstantParameter()  # kg
        self.tire_positions = ConstantParameter()  # [FL, FR, RL, RR]

        # ======================
        # ===== Suspension =====
        # ======================

        self.decoupled = ToggleParameter()  # boolean

        # Decoupled rate inputs
        self.front_heave_springrate = ConstantParameter()  # N/m
        self.front_roll_springrate = ConstantParameter()  # N/m
        self.rear_heave_springrate = ConstantParameter()  # N/m
        self.rear_roll_springrate = ConstantParameter()  # N/m

        self.front_heave_MR = ConstantParameter()  # unitless, but make these curve parameters once we add functionality
        self.front_roll_MR = ConstantParameter()  # unitless, but make these curve parameters once we add functionality

        self.rear_heave_MR = ConstantParameter()  # unitless, but make these curve parameters once we add functionality
        self.rear_roll_MR = ConstantParameter()  # unitless, but make these curve parameters once we add functionality

        # Coupled rate inputs
        self.front_springrate = ConstantParameter()  # N/m
        self.rear_springrate = ConstantParameter()  # N/m

        self.front_ARB_rate = ConstantParameter()  # Nm/rad
        self.rear_ARB_rate = ConstantParameter()  # Nm/rad

        self.front_ARB_MR = ConstantParameter()  # unitless
        self.rear_ARB_MR = ConstantParameter()  # unitless

        self.front_MR = CurveParameter()  # unitless
        self.rear_MR = CurveParameter()  # unitless

        # Shared inputs

        self.front_anti = ConstantParameter()  # percent
        self.rear_anti = ConstantParameter()  # percent

        self.ackermann = ConstantParameter()  # percent

        # Kinematics

        self.static_IAs = ConstantParameter()  # rad, list[float] (FL, FR, RL, RR)
        self.roll_IA_gain = ConstantParameter()
        # deg/rad list[float] (FL, FR, RL, RR), but make these curve parameters once we add functionality
        self.heave_IA_gain = ConstantParameter()
        # deg/m list[float] (FL, FR, RL, RR), but make these curve parameters once we add functionality

        self.front_KPI = ConstantParameter()  # rad
        self.rear_KPI = ConstantParameter()  # rad
        self.front_caster = ConstantParameter()  # rad
        self.rear_caster = ConstantParameter()  # rad

        self.toe_angles = ConstantParameter()  # rad, list[float] (FL, FR, RL, RR)
        self.front_bump_steer = ConstantParameter()  # rad/m, but make these curve parameters once we add functionality
        self.front_roll_steer = ConstantParameter()  # rad/rad, but make these curve parameters later
        self.rear_bump_steer = ConstantParameter()  # rad/m, but make these curve parameters once we add functionality
        self.rear_roll_steer = ConstantParameter()  # rad/rad, but make these curve parameters once we add functionality

        self.front_roll_center_height = CurveParameter()  # m
        self.rear_roll_center_height = CurveParameter()  # m

        self.rack_clevis_to_kingpin_y = ConstantParameter()  # m
        self.rack_clevis_to_kingpin_x = ConstantParameter()  # m
        self.tie_linkage_length = ConstantParameter()  # m
        self.steering_arm = ConstantParameter()  # m
        self.c_factor = ConstantParameter()  # m/rev

        # Tires
        self.front_tire_coeff_Fy = ConstantParameter()  # coeffs
        self.front_tire_coeff_Fx = ConstantParameter()  # coeffs
        self.rear_tire_coeff_Fy = ConstantParameter()  # coeffs
        self.rear_tire_coeff_Fx = ConstantParameter()  # coeffs

        self.front_tire_vertical_rate = ConstantParameter(
            )  # N/m, but make these curve parameters once we add functionality
        self.rear_tire_vertical_rate = ConstantParameter(
            )  # N/m, but make these curve parameters once we add functionality

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
        self.motor_kt = ConstantParameter()  # [Nm/Arms]
        self.motor_induced_voltage = ConstantParameter()  # [RPM/Vp]
        self.motor_efficiency = CurveParameter()  # [%] = f(torque [Nm], RPM [RPM])
        self.motor_thermal_resistance = ConstantParameter()  # [C/W]
        self.inverter_efficiency = ConstantParameter()  # [%]
        self.inverter_thermal_resistance = ConstantParameter()  # [C/W]
        self.drivetrain_efficiency = ConstantParameter()  # [%]
        self.drivetrain_moment_of_inertia = ConstantParameter()  # [kgm^2]

        # ======================
        # ==== Aerodynamics ====
        # ======================

        self.air_density = ConstantParameter() # kg/m^3
        self.air_temperature = ConstantParameter() # degrees C
        self.ClA_tot = ConstantParameter()
        self.CdA_tot = ConstantParameter()
        self.CsA_tot = ConstantParameter()
        self.CdA0 = ConstantParameter() # Drag coeff from non-aero
        self.static_ride_height = ConstantParameter() # m
        self.CsA0 = ConstantParameter() # Sideforce coeff from non-aero

        # Distribution of downforce across components
        self.ClA_dist = ConstantParameter() # [front, undertray, rear]
        self.CdA_dist = ConstantParameter() # [front, undertray, rear]
        self.CsA_dist = ConstantParameter() # [front, undertray, rear]

        # Pitch, body_slip, and roll sensitivities,
        self.p_sens	= ConstantParameter()
        self.bs_sens = ConstantParameter()
        self.r_sens	= ConstantParameter()

        # CoP positions from vehicle origin (CAD)
        # Front, Undertray and Rear [x , y , z] (Inches)
        self.CoP = ConstantParameter()

        # Heave sensitivity regression fitted to undertray data
        self.h_sens_coefficients = ConstantParameter()

        self._lock()
