from sim.model_parameters.parameters import ConstantParameter, CurveParameter, ToggleParameter, SurfaceParameter
from sim.model_parameters.model_parameters import ModelParameters


class Car(ModelParameters):
    def __init__(self):
        super().__init__()
        self._model_type = "Car"
        self._model_name = "Unnamed Car"

        # Everything related to tires uses one parameter, stored in the order: [FL, FR, RL, RR]

        # ======================
        # ======= General ======
        # ======================

        self.sprung_inertia = ConstantParameter()  # kgm^2 (inertia tensor)
        self.accel_gravity = ConstantParameter()  # m/s^2
        self.cg_bias = ConstantParameter()  # (percent from front. 0 -> frontmost, 1 -> rearmost)
        self.cg_left = ConstantParameter()  # (percent from left. 0 -> leftmost, 1 -> rightmost)
        self.cg_height = ConstantParameter()  # m
        self.wheelbase = ConstantParameter()  # m
        self.front_track = ConstantParameter()  # m
        self.rear_track = ConstantParameter()  # m

        self.front_unsprung_mass = ConstantParameter()  # kg of entire axle
        self.rear_unsprung_mass = ConstantParameter()  # kg of entire axle
        self.driver_mass = ConstantParameter()  # kg
        self.mass_sprung = ConstantParameter()  # kg
        self.total_mass = ConstantParameter()  # kg
        self.tire_positions = ConstantParameter()  # m (relative to cg)

        # ======================
        # ===== Suspension =====
        # ======================

        self.decoupled = ToggleParameter()  # boolean (not currently used)

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

        # Kinematics

        self.static_IAs = ConstantParameter()  # rad
        self.FL_IA_gain = SurfaceParameter()  # rad
        self.FR_IA_gain = SurfaceParameter()  # rad
        self.RL_IA_gain = SurfaceParameter()  # rad
        self.RR_IA_gain = SurfaceParameter()  # rad
        self.FI_steering_response = CurveParameter()  # rad
        self.FO_steering_response = CurveParameter()  # rad

        self.toe_angles = ConstantParameter()  # rad
        # self.front_bump_steer = ConstantParameter()  # rad/m, but make these curve parameters once we add functionality
        # self.front_roll_steer = ConstantParameter()  # rad/rad, but make these curve parameters later
        # self.rear_bump_steer = ConstantParameter()  # rad/m, but make these curve parameters once we add functionality
        # self.rear_roll_steer = ConstantParameter()  # rad/rad, but make these curve parameters once we add functionality

        self.front_roll_center_height = CurveParameter()  # m
        self.rear_roll_center_height = CurveParameter()  # m

        # Tires
        self.tire_radii = ConstantParameter()  # [front, rear]
        self.front_tire_coeff_Fy = ConstantParameter()  # coeffs
        self.front_tire_coeff_Fx = ConstantParameter()  # coeffs
        self.rear_tire_coeff_Fy = ConstantParameter()  # coeffs
        self.rear_tire_coeff_Fx = ConstantParameter()  # coeffs

        self.front_tire_vertical_rate = ConstantParameter()  # N/m, but make these curve parameters once we add functionality
        self.rear_tire_vertical_rate = ConstantParameter()  # N/m, but make these curve parameters once we add functionality

        # ======================
        # ======= Pedals =======
        # ======================

        self.eff_rotor_radius = ConstantParameter()  # [front, rear]
        self.MC_SA = ConstantParameter()
        self.C_SA = ConstantParameter()
        self.mu = ConstantParameter()
        self.pedal_ratio = ConstantParameter()
        self.brake_bias = ConstantParameter()  # Frontward bias (1 is pure front and 0 is pure rear)
        self.max_DF = ConstantParameter()  # Maximum driver force

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
        self.max_torque = ConstantParameter()  # Nm
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
        self.front_unsprung_inertia = ConstantParameter()  # [kgm^2]
        self.rear_unsprung_inertia = ConstantParameter()  # [kgm^2]
        self.gear_ratio = ConstantParameter()  # diff to motor
        self.diff_efficiency = ConstantParameter()  # %

        # ======================
        # ==== Aerodynamics ====
        # ======================

        self.aero = ToggleParameter()

        self.air_density = ConstantParameter()  # kg/m^3
        self.air_temperature = ConstantParameter()  # degrees C
        self.ClA_tot = ConstantParameter()
        self.CdA_tot = ConstantParameter()
        self.CsA_tot = ConstantParameter()
        self.CdA0 = ConstantParameter()  # Drag coeff from non-aero
        self.static_ride_height = ConstantParameter()  # m
        self.CsA0 = ConstantParameter()  # Sideforce coeff from non-aero

        # Distribution of downforce across components
        self.ClA_dist = ConstantParameter()  # [front, undertray, rear]
        self.CdA_dist = ConstantParameter()  # [front, undertray, rear]
        self.CsA_dist = ConstantParameter()  # [front, undertray, rear]

        # Pitch, body_slip, and roll sensitivities,
        self.p_sens = ConstantParameter()
        self.bs_sens = ConstantParameter()
        self.r_sens = ConstantParameter()

        # CoP positions from vehicle origin (CAD)
        # Front, Undertray and Rear [x , y , z] (Inches)
        self.CoP = ConstantParameter()

        # Heave sensitivity regression fitted to undertray data
        self.h_sens_coefficients = ConstantParameter()

        self._lock()
