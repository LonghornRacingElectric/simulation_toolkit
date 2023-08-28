from sim.model_parameters.cars.car import Car
from sim.model_parameters.parameters import ConstantParameter, CurveParameter, ToggleParameter


class LadyLuck(Car):
    def __init__(self):
        super().__init__()
        self._model_name = "Lady Luck"

        # ======================
        # ======= General ======
        # ======================

        self.sprung_inertia = ConstantParameter([[119.8, 0, 0], [0, 33.4, 0], [0, 0, 108.2]])  # kgm^2 (inertia tensor) TODO temp 0
        self.accel_gravity = ConstantParameter(9.81)  # m/s^2
        self.cg_bias = ConstantParameter(0.52)  # m (percent from front. 0 -> frontmost, 1 -> rearmost)
        self.cg_left = ConstantParameter(0.50)  # m (percent from left. 0 -> leftmost, 1 -> rightmost)
        self.cg_height = ConstantParameter(0.28448)  # m
        self.wheelbase = ConstantParameter(1.5494)  # m
        self.front_track = ConstantParameter(1.27)  # m
        self.rear_track = ConstantParameter(1.2192)  # m

        self.front_unsprung_mass = ConstantParameter(16.328)  # kg of entire axle
        self.rear_unsprung_mass = ConstantParameter(18.611)  # kg of entire axle
        self.driver_mass = ConstantParameter(70)  # kg
        self.mass_sprung = ConstantParameter(170)  # kg
        self.total_mass = ConstantParameter(self.front_unsprung_mass + self.rear_unsprung_mass +
                                            self.driver_mass + self.mass_sprung)  # kg
        self.tire_positions = ConstantParameter(
            [[self.wheelbase * self.cg_bias, self.front_track / 2, -self.cg_height],
             [self.wheelbase * self.cg_bias, -self.front_track / 2, -self.cg_height],
             [-self.wheelbase * (1 - self.cg_bias), self.rear_track / 2, -self.cg_height],
             [-self.wheelbase * (1 - self.cg_bias), -self.rear_track / 2, -self.cg_height]])  # [FL, FR, RL, RR] relative to CG

        # ======================
        # ===== Suspension =====
        # ======================

        self.decoupled = ToggleParameter(True)  # boolean

        # Decoupled rate inputs
        self.front_heave_springrate = ConstantParameter(400 * 175.127)  # N/m
        self.front_roll_springrate = ConstantParameter(400 * 175.127)  # N/m
        self.rear_heave_springrate = ConstantParameter(400 * 175.127)  # N/m
        self.rear_roll_springrate = ConstantParameter(400 * 175.127)  # N/m

        self.front_heave_MR = ConstantParameter(
            1 / 1.382)  # unitless, but make these curve parameters once we add functionality
        self.front_roll_MR = ConstantParameter(
            1 / 1.137)  # unitless, but make these curve parameters once we add functionality

        self.rear_heave_MR = ConstantParameter(
            1 / 1.271)  # unitless, but make these curve parameters once we add functionality
        self.rear_roll_MR = ConstantParameter(
            1 / 1.034)  # unitless, but make these curve parameters once we add functionality

        # Coupled rate inputs
        self.front_springrate = ConstantParameter(0)  # N/m  TODO temp 0
        self.rear_springrate = ConstantParameter(0)  # N/m  TODO temp 0

        self.front_ARB_rate = ConstantParameter(0)  # Nm/rad  TODO temp 0
        self.rear_ARB_rate = ConstantParameter(0)  # Nm/rad  TODO temp 0

        self.front_ARB_MR = ConstantParameter(0)  # unitless  TODO temp 0
        self.rear_ARB_MR = ConstantParameter(0)  # unitless  TODO temp 0

        self.front_MR = ConstantParameter(1)
        self.rear_MR = ConstantParameter(1)

        self.front_MR = CurveParameter(from_function=lambda x: 0, x_min=0, x_max=0, x_samples=0)  # unitless  TODO temp 0
        self.rear_MR = CurveParameter(from_function=lambda x: 0, x_min=0, x_max=0, x_samples=0)  # unitless  TODO temp 0

        # Shared inputs

        self.front_anti = ConstantParameter(0.20)  # percent
        self.rear_anti = ConstantParameter(0.00)  # percent

        self.ackermann = ConstantParameter(0)  # percent TODO temp 0

        # Kinematics

        self.static_IAs = ConstantParameter(
            [1 * 0.01745, -1 * 0.01745, 1 * 0.01745, -1 * 0.01745])  # rad, list[float] (FL, FR, RL, RR)
        self.roll_IA_gain = ConstantParameter([0, 0, 0, 0])
        # deg/rad list[float] (FL, FR, RL, RR), but make these curve parameters once we add functionality
        self.heave_IA_gain = ConstantParameter([0, 0, 0, 0])
        # deg/m list[float] (FL, FR, RL, RR), but make these curve parameters once we add functionality

        self.front_KPI = ConstantParameter(0)  # rad TODO temp 0
        self.rear_KPI = ConstantParameter(0)  # rad TODO temp 0
        self.front_caster = ConstantParameter(0)  # rad TODO temp 0
        self.rear_caster = ConstantParameter(0)  # rad TODO temp 0

        self.toe_angles = ConstantParameter([0, 0, 0, 0])  # rad, list[float] (FL, FR, RL, RR)
        self.front_bump_steer = ConstantParameter(0)  # rad/m, but make these curve parameters once we add functionality
        self.front_roll_steer = ConstantParameter(0)  # rad/rad, but make these curve parameters once we add functionality
        self.rear_bump_steer = ConstantParameter(0)  # rad/m, but make these curve parameters once we add functionality
        self.rear_roll_steer = ConstantParameter(0)  # rad/rad, but make these curve parameters once we add functionality

        self.front_roll_center_height = CurveParameter(from_function=lambda x: 0,
                                                       x_min=0, x_max=1, x_samples=20)  # m TODO temp 0
        self.rear_roll_center_height = CurveParameter(from_function=lambda x: 0,
                                                      x_min=0, x_max=1, x_samples=20)  # m TODO temp 0

        self.rack_clevis_to_kingpin_y = ConstantParameter(16.115 * 0.0254)  # m
        self.rack_clevis_to_kingpin_x = ConstantParameter(2.203 * 0.0254)  # m
        self.tie_linkage_length = ConstantParameter(15.555 * 0.0254)  # m
        self.steering_arm = ConstantParameter(2.207 * 0.0254)  # m
        self.c_factor = ConstantParameter(3.46 * 0.0254)  # m/rev

        # Tires
        self.front_tire_coeff_Fy = ConstantParameter(
            [0.349, -0.00115, 8.760, 730.300, 1745.322, 0.0139, -0.000277, 1.02025435, 0, 0, 0, 0, 0, 0, 0, 0.00362,
             -0.0143, -0.0116])  # coeffs
        self.front_tire_coeff_Fx = ConstantParameter(
            [0.46024966176377113, 4000.509873697152, 1097.1712081460967, 202.18848632159495, 100.8812198037175,
             -0.2557010431649166, 0.3066955241461764, 0.011822770671297778, -1.9521015799737094, 0, 0, 0, 0,
             0])  # coeffs
        self.rear_tire_coeff_Fy = ConstantParameter(
            [0.349, -0.00115, 8.760, 730.300, 1745.322, 0.0139, -0.000277, 1.02025435, 0, 0, 0, 0, 0, 0, 0, 0.00362,
             -0.0143, -0.0116])  # coeffs
        self.rear_tire_coeff_Fx = ConstantParameter(
            [0.46024966176377113, 4000.509873697152, 1097.1712081460967, 202.18848632159495, 100.8812198037175,
             -0.2557010431649166, 0.3066955241461764, 0.011822770671297778, -1.9521015799737094, 0, 0, 0, 0,
             0])  # coeffs

        self.front_tire_vertical_rate = ConstantParameter(725 * 175.127)  # N/m, but make these curve parameters once we add functionality
        self.rear_tire_vertical_rate = ConstantParameter(725 * 175.127)  # N/m, but make these curve parameters once we add functionality

        # ======================
        # ===== Powertrain =====
        # ======================

        # ----- Battery -----
        self.hv_battery_capacity = ConstantParameter(300000)  # [As]
        self.hv_battery_nominal_voltage = ConstantParameter(350)  # [V]
        self.hv_battery_open_circuit_voltage = CurveParameter(from_function=lambda soc: soc * 100 + 320,
                                                              x_min=0, x_max=1, x_samples=20)  # [V] = f(SoC [As], temp [C])
        self.hv_battery_internal_resistance = CurveParameter(from_function=lambda soc: 0.1,
                                                             x_min=0, x_max=1, x_samples=20)  # [Ohms] = f(SoC [As], temp [C])
        self.hv_battery_thermal_resistance = ConstantParameter(0)  # [C/W]
        self.lv_battery_capacity = ConstantParameter(50000)  # [As]
        self.lv_battery_open_circuit_voltage = CurveParameter(from_function=lambda soc: soc * 4 + 22,
                                                              x_min=0, x_max=1, x_samples=20)  # [V] = f(SoC [As], temp [C])
        self.lv_battery_internal_resistance = ConstantParameter(0.15)  # [Ohms]
        self.lv_system_constant_power_draw = ConstantParameter(50)  # [W]
        self.has_dcdc = ToggleParameter(False)  # [true/false]
        self.dcdc_efficiency = ConstantParameter(0.90)  # [%]

        # ----- Cooling -----
        self.coolant_amount = ConstantParameter(0)  # [kg]
        self.coolant_thermal_resistance = ConstantParameter(0)  # [C/W]
        self.cooling_power_draw = CurveParameter(from_function=lambda c: 0,
                                                 x_min=0, x_max=1, x_samples=20)  # [W] = f(cooling [%])
        self.coolant_area_cooling = ConstantParameter(0)  # [m^2]
        self.coolant_area_hv_battery = ConstantParameter(0)  # [m^2]
        self.coolant_area_inverter = ConstantParameter(0)  # [m^2]
        self.coolant_area_motor = ConstantParameter(0)  # [m^2]

        # ----- Drivetrain -----
        self.regen_enabled = ToggleParameter(False)  # [true/false]
        self.power_limit = ConstantParameter(80000)  # [W]
        self.motor_peak_torque = CurveParameter(from_function=lambda rpm: 230,
                                                x_min=0, x_max=6500, x_samples=20)  # [Nm] = f(RPM [RPM])
        self.motor_peak_current = ConstantParameter(380)  # [A]
        self.motor_winding_resistance = ConstantParameter(0.00706)  # [Ohms]
        self.motor_kt = ConstantParameter(0.61)  # [Nm/A]
        self.motor_induced_voltage = ConstantParameter(14.753)  # [RPM/Vp]
        self.motor_efficiency = CurveParameter(from_function=lambda t: 0.95,
                                               x_min=0, x_max=230, x_samples=20)  # [%] = f(torque [Nm], RPM [RPM])
        self.motor_thermal_resistance = ConstantParameter(0)  # [C/W]
        self.inverter_efficiency = ConstantParameter(0.90)  # [%]
        self.inverter_thermal_resistance = ConstantParameter(0)  # [C/W]
        self.drivetrain_efficiency = ConstantParameter(0.97)  # [%]
        self.drivetrain_moment_of_inertia = ConstantParameter(1)  # [kgm^2] TODO lots of ptn are placeholders

        # ======================
        # ==== Aerodynamics ====
        # ======================

        self.air_temperature = ConstantParameter(33.8889) # degrees C
        self.air_density = ConstantParameter(1.225 - 0.003975 * (self.air_temperature - 15)) # kg/m^3
        self.ClA_tot = ConstantParameter(4.384)
        self.CdA_tot = ConstantParameter(1.028)
        self.CsA_tot = ConstantParameter(5.673)
        self.CdA0 = ConstantParameter(0.7155) # Drag coeff from non-aero
        self.static_ride_height = ConstantParameter(0.0762) # m
        self.CsA0 = ConstantParameter(8.43) # Sideforce coeff from non-aero

        # Distribution of downforce across components
        self.ClA_dist = ConstantParameter([0.474, 0.289, 0.236]) # [front, undertray, rear]
        self.CdA_dist = ConstantParameter([0.425, 0.178, 0.396]) # [front, undertray, rear]
        self.CsA_dist = ConstantParameter([0.666, 0.000, 0.333]) # [front, undertray, rear]

        # Pitch, body_slip, and roll sensitivities,
        self.p_sens	= ConstantParameter([[[ -15.7,  -10.6], [ -7.5,  -12.5], [0,0]],
                                         [[  7.39,  -7.19], [ 10.1, -15.15], [0,0]],
                                         [[ -0.72,  -4.06], [ 2.02,  -5.94], [0,0]]])
        self.bs_sens = ConstantParameter([[-2.4,   -0.7, 0], [  0.61, 0.89, 0], [-0.96,  0.33, 0]])
        self.r_sens	= ConstantParameter([[-10.1, -13.1, 0], [-1.37,     0, 0], [-4.24, -2.02, 0]])

        # CoP positions from vehicle origin (CAD)
        # Front, Undertray and Rear [x , y , z] (Inches)
        self.CoP = ConstantParameter([[x[0] * 0.0254, x[1] * 0.0254, x[2] * 0.0254] for x in [[23.65,  0, 9.30],
                                                                                              [-43.5,  0, 7.13],
                                                                                              [-67.6,  0, 42.91]]])

        # Heave sensitivity regression fitted to undertray data
        self.h_sens_coefficients = ConstantParameter([[-19.4, 1.89, 0.949],  [15.7, -6.22, 1.27]])
