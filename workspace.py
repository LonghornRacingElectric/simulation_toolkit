"""

This file is where you can mess with vehicle parameters and run simulations. You shouldn't have to work on any other
file in most cases.

"""
import numpy as np

from sim.model_parameters.cars.lady_luck import LadyLuck
from sim.model_parameters.drivers.ben_huff import BenHuff
from sim.model_parameters.drivers.rylan_hanks import RylanHanks
from sim.model_parameters.parameters import *
from sim.model_parameters.telemetry.lady_luck_telemetry import LadyLuckTelemetry
from sim.model_parameters.vcu.lady_luck_vcu import LadyLuckVcu
from sim.simulations.mmm_solver import MmmSolver
from sim.simulations.gg_generation import GGGeneration
from sim.simulations.transient_sim import TransientSimulation
from sim.util.analysis.mmm_sweeper import MmmSweeper
from sim.util.analysis.coeff_gen import CoeffSolver


car = LadyLuck()


def general_transient_sim():
    driver = BenHuff()
    telemetry = LadyLuckTelemetry()
    vcu = LadyLuckVcu()

    transient_sim = TransientSimulation(duration=10, time_step=0.003,
                                        car=car, driver=driver, telemetry=telemetry, vcu=vcu)
    transient_sim.run()

    transient_sim.plot_state("body_slip")
    transient_sim.plot_driver_control("steering_angle")
    transient_sim.plot_observable("slip_angles")
    transient_sim.plot_state("yaw_rate")
    transient_sim.plot_state("yaw")
    transient_sim.plot_state("wheel_angular_velocities")
    transient_sim.plot_observable("long_accel")
    transient_sim.plot_observable("lateral_accel")

    transient_sim.plot_map()

    transient_sim.print_key_points()


def floor_it_sim():
    driver = RylanHanks()
    telemetry = LadyLuckTelemetry()
    vcu = LadyLuckVcu()

    transient_sim = TransientSimulation(duration=10, time_step=0.003,
                                        car=car, driver=driver, telemetry=telemetry, vcu=vcu)
    transient_sim.run()

    transient_sim.plot_driver_control("accel_pedal_pct")
    transient_sim.plot_vcu_output("torque_request")
    transient_sim.plot_state("motor_rpm")
    transient_sim.plot_state("speed")
    transient_sim.plot_state("wheel_angular_velocities")

    transient_sim.plot_map()

    transient_sim.print_key_points()


def general_MMM():
    mmm_solver = MmmSolver(car=car, mesh=21, velocity=25)
    mmm_solver.solve()
    mmm_solver.print_key_points()
    mmm_solver.plot()


def general_GG():
    gg_generator = GGGeneration(car=car, mesh=5, velocity=30)
    gg_generator.solve()
    gg_generator.print_key_points()
    gg_generator.plot()


def general_competition():
    pass  # TODO


def aero_coefficients_MMM_sweep():
    mmm_sweeper = MmmSweeper(car=car, mesh=21, velocity=30)

    ClA = np.linspace(1.0, 5.0, 11)
    CdA = 0.187 * ((ClA - 1) ** 1.5) + 0.5

    mmm_sweeper.add_dimension("ClA_tot", ClA)
    mmm_sweeper.add_dimension("CdA_tot", CdA, couple_to="ClA_tot")
    mmm_sweeper.solve_all()
    mmm_sweeper.plot_convex_hull("ClA_tot")


def aero_CoP_MMM_sweep():
    mmm_sweeper = MmmSweeper(car=car, mesh=21, velocity=15)

    CoP0 = np.array([[0.51, 0, 0], [0.51, 0, 0], [0.51, 0, 0]]) * -car.wheelbase
    CoP1 = np.array([[0.54, 0, 0], [0.54, 0, 0], [0.54, 0, 0]]) * -car.wheelbase
    CoP = np.linspace(CoP0, CoP1, 4)
    mmm_sweeper.add_dimension("CoP", CoP)

    mmm_sweeper.solve_all()
    mmm_sweeper.plot_convex_hull("CoP")
    mmm_sweeper.plot_key_points("CoP")

    mmm_sweeper.update_velocity(30)
    mmm_sweeper.solve_all()
    mmm_sweeper.plot_convex_hull("CoP")
    mmm_sweeper.plot_key_points("CoP")


def compare_gg_regen():
    gg_generator = GGGeneration(car=car, mesh=9, velocity=15)

    car.regen_enabled = ToggleParameter(False)
    car.brake_bias = ConstantParameter(0.56)

    print("======== without regen ========")
    gg_generator.solve()
    gg_generator.plot()
    gg_generator.print_key_points()

    car.regen_enabled = ToggleParameter(True)
    car.brake_bias = ConstantParameter(0.70)

    print("\n======== with regen ========")
    gg_generator.solve()
    gg_generator.plot()
    gg_generator.print_key_points()

def lat_scaling_match():
    lat_coeff_solver = CoeffSolver(car)

    coeffs_start = [1.3527376401722393, -2.5036312664108062, 0.6505973463169893, 9.423768050812576, 0.17849509152594265, -0.04752524063648042, 0.8079559806081511, 42.81076780141361, 31.076122612890995, 1.2651199149814782, 0.6105325571031571, -0.0025491228717106594, 0.001096211018526508, -0.06944567882483914, 0.012229623569108243, -0.018945437981311805, -1.4250325348174189, -1.0850605408574026]

    coeffs_end = [1.3621019747547105, -2.435216396304886, 0.65, 9.423678477311629, 0.14799132747634844, -0.062214373627922905, 0.8075703695312021, 42.81227067232975, 22.19707765833606, 0.904357602882356, 0.6108200318681702, -0.0026565847602868942, -0.0020774251539237916, -0.07082967061402329, 0.009860302616368338, -0.030866236429790993, -1.8590779944132787, -1.519168529358441]

    scaling_coeffs1 = [1 for x in range(28)]
    scaling_coeffs2 = [1 for x in range(28)]

    lat_coeff_solver.import_lat_data("./data/tires/Hoosier_20.5x7.0-13_R20_7_cornering.csv")
    results = lat_coeff_solver.match_pure_lat(coeffs_start, coeffs_end, scaling_coeffs1, scaling_coeffs2)

    print(f"Lat Scaling Coeffs: {list(results[0])}\
        \n Residual: {results[1]}")

def lat_coeff_gen():
    lat_coeff_solver = CoeffSolver(car)
    results = lat_coeff_solver.lat_coeff_solve("./data/tires/Hoosier_16x7.5-10_R20_7_cornering.csv")

    print(f"Lateral Coefficients: {list(results[0])}\
          \n Residual: {results[1]}")
    
def long_coeff_gen():
    long_coeff_solver = CoeffSolver(car)
    results = long_coeff_solver.long_coeff_solve("./data/tires/Hoosier_20.5x7.0-13_R20_7_braking.csv")

    print(f"Longitudinal Coefficients: {list(results[0])}\
          \n Residual: {results[1]}")
    
def aligning_coeff_gen():
    aligning_coeff_solver = CoeffSolver(car)
    results = aligning_coeff_solver.aligning_coeff_solve("./data/tires/Hoosier_18x6.0-10_R20_7_cornering.csv")

    print(f"Aligning Coefficients: {list(results[0])}\
          \n Residual: {results[1]}")



# general_GG()
# general_MMM()
# general_transient_sim()
# floor_it_sim()
# compare_gg_regen()
lat_coeff_gen()
# long_coeff_gen()
# aligning_coeff_gen()
# lat_scaling_match()