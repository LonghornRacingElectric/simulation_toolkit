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
from sim.simulations.ggv_generation import GGVGeneration
from sim.simulations.transient_sim import TransientSimulation
from sim.util.analysis.mmm_sweeper import MmmSweeper
from sim.util.analysis.coeff_gen import CoeffSolver
from sim.system_models.vehicle_systems.tire_model52 import TireModel
from sim.simulations.track_generation import Track
from sim.simulations.competition_sim import Competition


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


def general_GGV():
    ggv_generator = GGVGeneration(car=car, mesh=5, velocity_range=[5, 30], aero = True)
    ggv_generator.solve()
    ggv_generator.plot()


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
    new_tire = TireModel(pure_lat_coeffs =  [1.390302979409026, -2.183130940604851, 0.548943874299006, 5.7346976356720605, 0.6591127616852288, 0.2108868940546528, -0.16978285364274096, 1.539745087368218, 86.36379798361102, 3.0874245794244293, 1.1683485341193884, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    new_solver = CoeffSolver(initial_tire = new_tire, 
                            # lat_file = "./data/tires/Hoosier_18x6.0-10_R20_7_cornering.csv", 
                            lat_file = "./data/tires/Hoosier_20.5x7.0-13_R20_7_cornering.csv", 
                            # long_combined_file = "./data/tires/Hoosier_18x6.0-10_R20_7_braking.csv",
                            # long_combined_file = "./data/tires/Hoosier_20.5x7.0-13_R20_7_braking.csv",
                            acc_coeff_dev = 4,
                            iterations = 3,
                            sample_rate = 200)

    lat_coeff_soln = new_solver.pure_lat_coeff_solve()
    print(lat_coeff_soln[0])

    new_tire.pure_lat_coeffs = lat_coeff_soln[1]
    new_tire.plot(plot_type = "pure_lat", scatter = "./data/tires/Hoosier_20.5x7.0-13_R20_7_cornering.csv")

    # long_coeff_soln = new_solver.pure_long_coeff_solve()
    # print(long_coeff_soln[0])

    # new_tire.pure_long_coeffs = long_coeff_soln[1]
    # # new_tire.plot(plot_type = "pure_long", scatter = "./data/tires/Hoosier_18x6.0-10_R20_7_braking.csv")
    # new_tire.plot(plot_type = "pure_long", scatter = "./data/tires/Hoosier_20.5x7.0-13_R20_7_braking.csv")


def plot_tire():
    _18_inch_tire = TireModel(pure_lat_coeffs =  [1.5689052132298509, -2.4058598416972226, 0.500156429884467, 13.427132334274686, 0.6030685283044229, -0.3031176170452415, -0.05961420326250624, 2.637666248764366, 23.01777294060937, 0.9599873277568288, 0.8725064904171003, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        pure_long_coeffs = [1.3319154356008962, 2.1355685230690997, -0.6089128738140297, 16.999877639402516, -2.401069740012681, -8.186229136023606, -5.801347204360553, 0.04062524474313689, 38.993849158010825, 33.999086974794245, -1.9500611470974205, 0.0, 0.0, 0.0, 0.0])
    
    _20_inch_tire = TireModel(pure_lat_coeffs = [1.4579832334404343, -2.257149805835121, 0.5001299359434984, 0.13480368684784994, 0.49260419403807976, -0.23141039356465773, -0.12717227128530795, -0.007260679456500245, 29.062566726568917, 1.0052213903362235, 1.1475190011442866, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        pure_long_coeffs = [1.1884007780685448, 2.2208124291941913, -0.5006725603155423, 16.730232816471055, -2.8986667813496356, -8.709222427202736, -5.652873185408563, 0.14175799805971884, 38.691920445935104, 33.591246797001, -2.21971922589285, 0.0, 0.0, 0.0, 0.0])
    
    _18_inch_tire.plot(plot_type = "pure_lat", scatter = "./data/tires/Hoosier_18x6.0-10_R20_7_cornering.csv")
    _20_inch_tire.plot(plot_type = "pure_lat", scatter = "./data/tires/Hoosier_20.5x7.0-13_R20_7_cornering.csv")
    # _18_inch_tire.plot(plot_type = "pure_long", scatter = "./data/tires/Hoosier_18x6.0-10_R20_7_braking.csv")
    # _20_inch_tire.plot(plot_type = "pure_long", scatter = "./data/tires/Hoosier_20.5x7.0-13_R20_7_braking.csv")


def track_generation():
    new_track = Track("./data/tracks/test_track.csv", 1, car.front_track, 100)
    new_track._optimal_line()
    new_track.plot()


# general_GG()
# general_GGV()
# general_MMM()
# general_transient_sim()
floor_it_sim()
# compare_gg_regen()
# coeff_solving()
# plot_tire()
# track_generation()
