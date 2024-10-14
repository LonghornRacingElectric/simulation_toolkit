from powertrain_model import PowertrainModel
import pandas as pd
import numpy as np
import yaml
import pickle as pkl
import os
import sys

# make everything in cd accessible
for dirpath, dirnames, filenames in os.walk(os.curdir):
    sys.path.append(dirpath)

# create powertrain parameters
with open('powertrain_parameters.yml', 'r') as file:
    parameters = yaml.safe_load(file)

with open('powertrain_LUTs.pkl', 'rb') as file:
    luts = pkl.load(file)

powertrain_parameters = {**parameters, **luts}

# print(parameters)

class Car:
    def __init__(self, powertrain_params):
        self.parameters = {'powertrain': powertrain_params}

test_car = Car(powertrain_parameters)

test_car.state_in = {
    'wheel_angular_velocities': [150]*4,  # [rad/s] ~ 30 m/s
    'soc': 0.4,
    # avg_cell_temp: 30
    'tire_torques': [0, 0, 100, 100],

    'motor_rpm': None,
    'motor_torque': None,
    'motor_power': None,
    'battery_terminal_power': None,
    'battery_terminal_voltage': None,
    'battery_current': None,
    'battery_power': None
}

test_car.controls = {
    'torque_request': float(220)  # [Nm]
}

test_ptn = PowertrainModel(Car(powertrain_parameters))

state = test_ptn.eval(test_car.state_in, test_car.controls)

# socs = np.linspace(0.1, 1, 10)
# for soc in socs:
#     test_car.state_in['soc'] = soc
#     print('SOC:', soc)
#     test_ptn.eval(test_car.state_in, test_car.controls)

for param in list(state.keys()):
    if np.shape(state[param]) == (1 or () or (1,)) or np.size(state[param]) == 1:
        print(param, state[param])  # fix!
    else:
        print(param, state[param])
print('end')
