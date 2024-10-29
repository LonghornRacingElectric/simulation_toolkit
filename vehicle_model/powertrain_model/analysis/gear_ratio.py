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

test_car = Car(powertrain_parameters)

test_car.state_in = {
    'wheel_angular_velocities': [30]*4,  # rad/s
    'soc': 0.1,
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
gear_ratio sweep
"""

torques = {}
vels = np.linspace(0, 350, 50)
rpms = {}
powers = {}
gear_ratios = [
               2.307692308, 2.384615385, 2.461538462, 2.538461538, 2.615384615, 2.692307692, 2.769230769,
               2.846153846, 2.923076923, 3, 3.076923077, 3.153846154, 3.230769231, 3.307692308, 3.384615385,
               3.461538462, 3.538461538, 3.615384615, 3.692307692, 3.769230769, 3.846153846, 3.923076923, 4,
               4.076923077, 4.153846154, 4.230769231, 4.307692308, 4.384615385, 4.461538462
               ]
wheel_speeds = {}
tire_radius = 8*0.0254  # [m]
fig, ax1 = plt.subplots(figsize=(8, 6))
output = {}
# for i in range(len(gear_ratios)):
for i in range(len(gear_ratios)):
    powertrain_parameters['gear_ratio']['value'] = gear_ratios[i]
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
    output[gear_ratios[i]] = iteration
    print(gear_ratios[i])
    torques[gear_ratios[i]] = torque
    powers[gear_ratios[i]] = power
    rpms[gear_ratios[i]] = rpm
    wheel_speeds[gear_ratios[i]] = wheel_speed

# output_df = pd.DataFrame(output)
# os.chdir(r'C:\Users\andre\Desktop\gear ratio sweep')
# output_df.to_csv('gear_ratio_sweep.csv', index=False)

for gr in gear_ratios:
    plt.plot(wheel_speeds[gr], torques[gr], color='k', label=gr)
plt.xlabel('Velocity [m/s]')
plt.ylabel('Torque [Nm]')
plt.title('Wheel Torque Speed Curve - Gear Ratio Sweep')
plt.show()

