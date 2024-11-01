from simulation_toolkit.vehicle_model.powertrain_model.powertrain_model import PowertrainModel
import pandas as pd
import numpy as np
import yaml
import pickle as pkl
import os
import sys
import matplotlib.pyplot as plt
from matplotlib import colors

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

test_car = Car(powertrain_parameters)

test_car.state_in = {
    'wheel_angular_velocities': [30]*4,  # rad/s
    'soc': None,
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
    'torque_request': float(220)  # [Nm]
}


"""
wheel_torque ~f(speed, soc)
"""

fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_title('wheel torque ~f(velocity, soc)  ~ type shit ~')
ax.set_xlabel('velocity [m/s]')
ax.set_ylabel('soc [%]')

torque_requests = [-220, 220]

for torque_set in torque_requests:
    test_car.controls['torque_request'] = torque_set
    torques = {}
    vels = np.linspace(0, 350, 100)
    rpms = {}
    powers = {}
    socs = np.arange(0.05, 1., .04)
    wheel_speeds = {}
    tire_radius = 8*0.0254  # [m]
    output = {}
    for soc in socs:
        test_car.state_in['soc'] = soc
        torque = np.array([])
        power = np.array([])
        rpm = np.array([])
        wheel_speed = np.array([])
        iteration = {}
        for vel in vels:
            test_car.state_in['wheel_angular_velocities'] = [vel] * 4
            test_ptn = PowertrainModel(Car(powertrain_parameters))
            state = test_ptn.eval(test_car.state_in, test_car.controls)
            # print('soc:', state['soc'], ';', 'battery_power:', state['battery_power'], ';',
            #       'battery_terminal_power', state['battery_terminal_power'], ';', 'motor_torque', state['motor_torque'],
            #       ';', 'motor_power', state['motor_power'], ';', 'battery_current', state['battery_current'], ';',
            #       'battery_terminal_voltage', state['battery_terminal_voltage'], ';', 'battery_ocv', state['battery_ocv'],
            #       ';' 'motor_rpm', state['motor_rpm'], ';', 'ptn torques', state['powertrain torques'])
            # torque = np.append(torque, state['motor_torque'])
            torque = np.append(torque, state['powertrain_torques'][2]+state['powertrain_torques'][3])
            power = np.append(power, state['motor_power']/1000)
            # rpm = np.append(rpm, state['motor_rpm'])
            wheel_speed = np.append(wheel_speed, vel*tire_radius)
            if state['powertrain_torques'][2] == 0:
                break
        iteration['torque']: torque
        iteration['power'] = power
        iteration['rpm'] = rpm
        iteration['wheel_speed'] = wheel_speed
        output[soc] = iteration

        print(soc)
        torques[soc] = torque
        powers[soc] = power
        rpms[soc] = rpm
        wheel_speeds[soc] = wheel_speed

    # 3d plot
    speed_grid, soc_grid = np.meshgrid(wheel_speeds[socs[0]], socs)
    torque_grid = np.array([torques[soc] for soc in socs])
    if np.average(torque_grid) < 0:
        cmap = 'viridis_r'
    else:
        cmap = 'viridis'
    surf = ax.plot_surface(speed_grid, soc_grid, torque_grid, cmap=cmap)

os.chdir('outputs')
with open('combined_wheel_torque~f(velocity, soc).pkl', 'wb') as file:
    pkl.dump(fig, file)
plt.show()

# 2d plot
# for soc in socs:
#     plt.plot(wheel_speeds[soc], torques[soc], color='k', label=soc)
# plt.xlabel('Velocity [m/s]')
# plt.ylabel('Torque [Nm]')
# plt.title('Wheel Torque ~f(speed, soc)')
# plt.show()

print('end')