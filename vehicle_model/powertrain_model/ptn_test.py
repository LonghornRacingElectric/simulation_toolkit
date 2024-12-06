from powertrain_model import PowertrainModel
import pandas as pd
import numpy as np
import yaml
import pickle as pkl
import os
import sys
import matplotlib.pyplot as plt

# make everything in cd accessible
for dirpath, dirnames, filenames in os.walk(os.curdir):
    sys.path.append(dirpath)

# create powertrain parameters
with open('2025_powertrain_parameters.yml', 'r') as file:
    parameters = yaml.safe_load(file)

with open('2025_powertrain_LUTs.pkl', 'rb') as file:
    luts = pkl.load(file)

powertrain_parameters = {**parameters, **luts}

# print(parameters)

class Car:
    def __init__(self, powertrain_params):
        self.parameters = {'powertrain': powertrain_params}

test_car = Car(powertrain_parameters)

test_car.state_in = {
    'wheel_angular_velocities': [30]*4,  # rad/s
    'soc': .99,
    # avg_cell_temp: 30
    'tire_torques': [0, 0, 1000, 1000],
    'motor_rpm': None,
    'motor_torque': None,
    'motor_power': None,
    'battery_terminal_power': None,
    'battery_terminal_voltage': None,
    'battery_current': None,
    'battery_power': None,
    'motor_efficiency': None,
    'inverter_efficiency': None,
    'battery_efficiency': None

}

test_car.controls = {
    'torque_request': float(0)  # [Nm]
}

############## check cell soc ##############

powertrain_parameters['s_count']['value'] = 7
test_ptn = PowertrainModel(Car(powertrain_parameters))
state = test_ptn.eval(test_car.state_in, test_car.controls)
print('soc:', state['soc'], ';', 'battery_power:', state['battery_power'], ';',
      'battery_terminal_power', state['battery_terminal_power'], ';', 'motor_torque', state['motor_torque'],
      ';', 'motor_power', state['motor_power'], ';', 'battery_current', state['battery_current'], ';',
      'battery_terminal_voltage', state['battery_terminal_voltage'], ';', 'battery_ocv', state['battery_ocv'],
      ';' 'motor_rpm', state['motor_rpm'], ';', 'ptn torques', state['powertrain_torques'])

##############################################

def plot2(x, y1, y2, xlabel, ylabel1, ylabel2, title):
    # Create a figure and axis
    fig, ax1 = plt.subplots(figsize=(8, 6))  # Higher resolution plot

    # Plot data on the left y-axis
    ax1.plot(x, y1, color='r', label=xlabel)
    ax1.set_xlabel(xlabel)
    ax1.set_ylabel(ylabel1, color='r')
    ax1.tick_params(axis='y', labelcolor='r')

    # Create a second y-axis
    ax2 = ax1.twinx()

    # Plot data on the right y-axis
    ax2.plot(x, y2, color='b', label=ylabel2)
    ax2.set_ylabel(ylabel2, color='b')
    ax2.tick_params(axis='y', labelcolor='b')

    # Add a title
    plt.title(title)

    # Display the plot
    # plt.show()

print('end')
