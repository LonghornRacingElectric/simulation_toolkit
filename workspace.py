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
from sim.system_models.vehicle_systems.tire_model52 import TireModel


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

def coeff_solving():
    new_tire = TireModel()
    new_solver = CoeffSolver(initial_tire = new_tire, 
                             lat_file = "./data/tires/Hoosier_18x6.0-10_R20_7_cornering.csv", 
                             long_combined_file = "./data/tires/Hoosier_18x6.0-10_R20_7_braking.csv")

    # lat_coeff_soln = new_solver.pure_lat_coeff_solve()
    # print(lat_coeff_soln)

    long_coeff_soln = new_solver.pure_long_coeff_solve()
    print(long_coeff_soln)

def plot_tire():
    new_tire = TireModel(pure_lat_coeffs = [1.4275324693483036, -2.255746043949771, 0.7531915873378026, 0.12956669501732465, 0.6813284445097716, 0.23885218635804512, 0.05489785167312618, -0.014852214405782035, 28.317328600336044, 1.618426294523789, 1.8793303226731732, -0.006541062184730178, -0.0066976720294467055, 0.027924392505731175, 0.03860784157420319, 0.11652979343774711, 0.2922171302702214, -0.10336365034336442],
                         pure_long_coeffs =  [1.2508706151229338, 2.435564830808038, -0.04367455517295262, 19.00247289648425, -0.012580943211965998, -0.03669280427661592, -0.019371568620225068, -225.03268133075804, 37.51437918736448, 34.36793489388575, -1.9611441049072624, 0.007481806347840445, 0.012639590647437653, -0.43169693558375566, -0.762112962304972])
    
    # new_tire.plot(plot_type = "pure_lat", scatter = "./data/tires/Hoosier_18x6.0-10_R20_7_cornering.csv")
    new_tire.plot(plot_type = "pure_long", scatter = "./data/tires/Hoosier_18x6.0-10_R20_7_braking.csv")



# general_GG()
# general_MMM()
# general_transient_sim()
# floor_it_sim()
# compare_gg_regen()
# coeff_solving()
plot_tire()
# long_coeff_gen()
# aligning_coeff_gen()
# lat_scaling_match()