"""

This file is where you can mess with vehicle parameters and run simulations. You shouldn't have to work on any other
file in most cases.

"""

from sim.model_parameters.cars.lady_luck import LadyLuck
from sim.model_parameters.drivers.ben_huff import BenHuff
from sim.model_parameters.parameters.constant_parameter import ConstantParameter
from sim.model_parameters.telemetry.lady_luck_telemetry import LadyLuckTelemetry
from sim.model_parameters.vcu.lady_luck_vcu import LadyLuckVcu

# create parameter models
car = LadyLuck()
driver = BenHuff()
telemetry = LadyLuckTelemetry()
vcu = LadyLuckVcu()

# simulate scenario
# TODO

# simulate competition
# TODO

# modify car parameters
car.max_acceleration = ConstantParameter(20)

# simulate scenario again
# TODO

# simulate competition again
# TODO
