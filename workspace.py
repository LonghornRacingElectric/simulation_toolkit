"""

This file is where you can mess with vehicle parameters and run simulations. You shouldn't have to work on any other
file in most cases.

"""
import numpy as np

from sim.model_parameters.cars.lady_luck import LadyLuck
from sim.model_parameters.drivers.ben_huff import BenHuff
from sim.model_parameters.parameters import *
from sim.model_parameters.telemetry.lady_luck_telemetry import LadyLuckTelemetry
from sim.model_parameters.vcu.lady_luck_vcu import LadyLuckVcu
from sim.simulations.mmm_solver import MmmSolver
from sim.simulations.gg_generation import GGGeneration
from sim.simulations.transient_sim import TransientSimulation
from sim.util.analysis.mmm_sweeper import MmmSweeper


# create parameter models
car = LadyLuck()
driver = BenHuff()
telemetry = LadyLuckTelemetry()
vcu = LadyLuckVcu()


def general_transient_sim():
    # car.aero = ToggleParameter(False)

    transient_sim = TransientSimulation(duration=8, time_step=0.003,  # TODO change to 0.003 internally
                                        car=car, driver=driver, telemetry=telemetry, vcu=vcu)
    transient_sim.run()
    # transient_sim.plot_driver_control("brake_pedal_pct")
    # transient_sim.plot_driver_control("drive_switch")
    # transient_sim.plot_vcu_output("park_or_drive")
    # transient_sim.plot_vcu_output("r2d_buzzer")
    # transient_sim.plot_driver_control("accel_pedal_pct")
    # transient_sim.plot_vcu_output("torque_request")
    transient_sim.plot_driver_control("steering_angle")
    transient_sim.plot_driver_control("accel_pedal_pct")
    transient_sim.plot_state("motor_rpm")
    transient_sim.plot_state("wheel_angular_velocities")
    transient_sim.plot_observable("slip_angles")
    transient_sim.plot_state("yaw_rate")
    transient_sim.plot_state("yaw")
    # transient_sim.plot_state_dot("hv_battery_current")
    # transient_sim.plot_observable("hv_battery_terminal_voltage")

    transient_sim.plot_map()


def general_MMM():
    mmm_solver = MmmSolver(car=car, mesh=21, velocity=15)
    mmm_solver.solve()
    mmm_solver.print_key_points()
    mmm_solver.plot()


def general_GGV():
    gg_generator = GGGeneration(car=car, mesh=5, velocity=15)
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


# general_GGV()
# general_MMM()
general_transient_sim()
