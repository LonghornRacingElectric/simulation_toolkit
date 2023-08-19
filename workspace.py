"""

This file is where you can mess with vehicle parameters and run simulations. You shouldn't have to work on any other
file in most cases.

"""

from sim.model_parameters.cars.lady_luck import LadyLuck
from sim.model_parameters.drivers.ben_huff import BenHuff
from sim.model_parameters.parameters.constant_parameter import ConstantParameter
from sim.model_parameters.telemetry.lady_luck_telemetry import LadyLuckTelemetry
from sim.model_parameters.vcu.lady_luck_vcu import LadyLuckVcu
from sim.simulations.transient_sim import TransientSimulation

# create parameter models
car = LadyLuck()
driver = BenHuff()
telemetry = LadyLuckTelemetry()
vcu = LadyLuckVcu()

# simulate scenario
sim = TransientSimulation(duration=5, time_step=0.01, car=car, driver=driver, telemetry=telemetry, vcu=vcu)
sim.run()
sim.plot_state("motor_rpm")
sim.plot_state_dot("hv_battery_current")

# simulate competition
# TODO

# modify car parameters
car.hv_battery_capacity = ConstantParameter(250000)  # Coulombs

# simulate scenario again
# TODO

# simulate competition again
# TODO
