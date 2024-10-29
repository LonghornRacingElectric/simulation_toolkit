from simulation_toolkit.vehicle_model.powertrain_model.powertrain_model import PowertrainModel
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
with open('../2025_powertrain_parameters.yml', 'r') as file:
    parameters = yaml.safe_load(file)

with open('../2025_powertrain_LUTs.pkl', 'rb') as file:
    luts = pkl.load(file)

powertrain_parameters = {**parameters, **luts}

# print(parameters)

class Car:
    def __init__(self, powertrain_params):
        self.parameters = {'powertrain': powertrain_params}

#  ------------------------------------------------------  #
#  ~ Inputs ~  #

torque_request = -220  # [Nm]
soc = 0.8  # [%]
mesh = 100

#  ------------------------------------------------------  #

test_car = Car(powertrain_parameters)

test_car.state_in = {
    'wheel_angular_velocities': [30]*4,  # rad/s
    'soc': soc,
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
    'torque_request': float(torque_request)  # [Nm]
}


torque = []
vels = np.linspace(start=0, stop=170, num=mesh)
rpm = []
power = []
fig, ax1 = plt.subplots(figsize=(16, 8))
for vel in vels:
    test_car.state_in['wheel_angular_velocities'] = [vel] * 4
    test_ptn = PowertrainModel(Car(powertrain_parameters))
    state = test_ptn.eval(test_car.state_in, test_car.controls)
    print('soc:', state['soc'], ';', 'battery_power:', state['battery_power'], ';',
          'battery_terminal_power', state['battery_terminal_power'], ';', 'motor_torque', state['motor_torque'],
          ';', 'motor_power', state['motor_power'], ';', 'battery_current', state['battery_current'], ';',
          'battery_terminal_voltage', state['battery_terminal_voltage'], ';', 'battery_ocv', state['battery_ocv'],
          ';' 'motor_rpm', state['motor_rpm'], ';', 'ptn torques', state['powertrain_torques'])
    torque = np.append(torque, state['motor_torque'])
    power = np.append(power, state['motor_power']/1000)
    rpm = np.append(rpm, state['motor_rpm'])


ax1.plot(rpm, torque, label='Torque [Nm]', color='b')
ax1.set_xlabel('RPM')
ax1.set_ylabel('Torque [Nm]', color='b')
ax1.tick_params(axis='y', labelcolor='b')

# Create a second y-axis
ax2 = ax1.twinx()

# Plot data on the right y-axis
ax2.plot(rpm, power, label='Torque [Nm]', color='r')
ax2.set_xlabel('RPM')
ax2.set_ylabel('Power [kW]', color='r')
ax2.tick_params(axis='y', labelcolor='r')

# Add a title
plt.title('Torque_Power(RPM)')
# ax1.legend()
plt.show()
