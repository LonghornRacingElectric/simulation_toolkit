"""

This file is where you can mess with vehicle parameters and run simulations. You shouldn't have to work on any other
file in most cases.

"""
import numpy as np

from sim.model_parameters.cars.lady_luck import LadyLuck
from sim.model_parameters.drivers.ben_huff import BenHuff
from sim.model_parameters.parameters.constant_parameter import ConstantParameter
from sim.model_parameters.telemetry.lady_luck_telemetry import LadyLuckTelemetry
from sim.model_parameters.vcu.lady_luck_vcu import LadyLuckVcu
from sim.simulations.mmm_solver import MmmSolver
from sim.simulations.transient_sim import TransientSimulation
from sim.util.analysis.mmm_sweeper import MmmSweeper

# create parameter models
car = LadyLuck()
driver = BenHuff()
telemetry = LadyLuckTelemetry()
vcu = LadyLuckVcu()

# simulate scenario
# transient_sim = TransientSimulation(duration=5, time_step=0.01, car=car, driver=driver, telemetry=telemetry, vcu=vcu)
# transient_sim.run()
# transient_sim.plot_state("motor_rpm")
# transient_sim.plot_state_dot("hv_battery_current")
# transient_sim.plot_observable("hv_battery_terminal_voltage")

# generate MMM
# mmm_solver = MmmSolver(car=car, mesh=21, velocity=25, aero=True)
# mmm_solver.solve()
# mmm_solver.print_key_points()
# mmm_solver.plot()

# cg bias sweep
mmm_sweeper = MmmSweeper(car=car, mesh=21, velocity=25, aero=True)
mmm_sweeper.add_dimension("cg_bias", np.linspace(0.46, 0.62, 9))
mmm_sweeper.solve_all()
mmm_sweeper.plot_convex_hull("cg_bias")

# simulate competition
# TODO

# modify car parameters
car.hv_battery_capacity = ConstantParameter(250000)  # Coulombs

# simulate scenario again
# TODO

# simulate competition again
# TODO
