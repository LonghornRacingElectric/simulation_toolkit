"""

This file is where you can mess with vehicle parameters and run simulations. You shouldn't have to work on any other
file in most cases.

"""

from sim.model_parameters.cars.lady_luck import LadyLuck
from sim.model_parameters.drivers.ben_huff import BenHuff
from sim.model_parameters.parameters.constant_parameter import ConstantParameter
from sim.model_parameters.telemetry.lady_luck_telemetry import LadyLuckTelemetry
from sim.model_parameters.vcu.lady_luck_vcu import LadyLuckVcu
from sim.simulations.scenario_sim import ScenarioSimulation

# create parameter models
car = LadyLuck()
driver = BenHuff()
telemetry = LadyLuckTelemetry()
vcu = LadyLuckVcu()

# simulate scenario
sim = ScenarioSimulation(duration=2, time_step=0.01, car=car, driver=driver, telemetry=telemetry, vcu=vcu)
sim.run()

# simulate competition
# TODO

# modify car parameters
car.max_torque = ConstantParameter(5)

# simulate scenario again
# TODO

# simulate competition again
# TODO
