from simulation_toolkit.vehicle_model.powertrain_model.powertrain_model import PowertrainModel
import pandas as pd
import numpy as np
import yaml
import pickle as pkl
import os
import sys
import matplotlib.pyplot as plt
from matplotlib import colors


# ------------------- inputs -------------------- #
# ------------------- ------ -------------------- #

torque_requests = np.array([-220, 220])  # [Nm]
soc_mesh = 25
soc_range = np.array([0.01, 0.99])  # [%]
velocity_mesh = 130
velocity_range = np.array([0, 35])  # [m/s]
tire_radius = 8  # [in]
gear_ratio = 3.30769

# ------------------- ------ -------------------- #
# ------------------- ------ -------------------- #



# make everything in cd accessible
for dirpath, dirnames, filenames in os.walk(os.curdir):
    sys.path.append(dirpath)

# create powertrain parameters
with open('../2025_powertrain_parameters.yml', 'r') as file:
    parameters = yaml.safe_load(file)

with open('../2025_powertrain_LUTs.pkl', 'rb') as file:
    luts = pkl.load(file)

powertrain_parameters = {**parameters, **luts}
powertrain_parameters['gear_ratio']['value'] = gear_ratio  # edit gear_ratio
powertrain_parameters['gear_ratio']['value']

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
    'battery_efficiency': None,
    'powertrain_torques': None,
    'differential_torque': None

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
ax.set_zlabel('wheel torque [Nm]')

tire_radius *= 0.0254  # [m]
velocities = np.linspace(velocity_range[0], velocity_range[1], velocity_mesh)  # [m/s]
ang_velocities = velocities/tire_radius  # [rad/s]
socs = np.linspace(soc_range[0], soc_range[1], soc_mesh)  # [%]


# setup output structure
output = {
    'data': {},
    'topology': {}
}
output['topology'] = {
        'torque_requests': {
            'values': torque_requests,
            'units': '[Nm]'
        },
        'socs': {
            'values': socs,
            'units': '[%]'
        },
        'velocities': {
            'values': velocities,
            'units': '[m/s]'
        }
    }

for torque_set in torque_requests:
    test_car.controls['torque_request'] = torque_set
    torque_state = {}
    for soc in socs:
        test_car.state_in['soc'] = soc
        soc_state = {}
        for i, ang_vel in enumerate(ang_velocities):
            test_car.state_in['wheel_angular_velocities'] = [ang_vel] * 4
            test_ptn = PowertrainModel(Car(powertrain_parameters))
            state_out = test_ptn.eval(test_car.state_in, test_car.controls)

            vel_state = test_car.state_in.copy()

            # print('soc:', state['soc'], ';', 'battery_power:', state['battery_power'], ';',
            #       'battery_terminal_power', state['battery_terminal_power'], ';', 'motor_torque', state['motor_torque'],
            #       ';', 'motor_power', state['motor_power'], ';', 'battery_current', state['battery_current'], ';',
            #       'battery_terminal_voltage', state['battery_terminal_voltage'], ';', 'battery_ocv', state['battery_ocv'],
            #       ';' 'motor_rpm', state['motor_rpm'], ';', 'ptn torques', state['powertrain torques'])
            # torque = np.append(torque, state['motor_torque'])
            # torque = np.append(torque, state['powertrain_torques'][2]+state['powertrain_torques'][3])
            # power = np.append(power, state['motor_power']/1000)
            # # rpm = np.append(rpm, state['motor_rpm'])
            # wheel_speed = np.append(wheel_speed, vel)  # [rad/s]
            # save_state = np.append(save_state, state)

            for state in list(vel_state.keys()):
                vel_state[state] = state_out[state]

            velocity = velocities[i]
            soc_state[velocity] = vel_state
        print(f'completed {soc*100:.2f}% soc')

        torque_state[soc] = soc_state

    output['data'][torque_set] = torque_state

# save that shit
save_data = {}
save_name = str(gear_ratio) + ''
# os.makedirs('outputs/' + 'gear_ratio_results', exist_ok=True)
# os.chdir('outputs/' + 'gear_ratio_results')
save_data['output'] = output
save_data['parameters_file'] = powertrain_parameters
save_data['combined_wheel_torque~f(velocity, soc)'] = fig
with open(save_name + '.pkl', 'wb') as file:
    pkl.dump(save_data, file)

# 3d plot that shit
torques = output['topology']['torque_requests']['values']
socs = output['topology']['socs']['values']
velocities = output['topology']['velocities']['values']
for torque_req in torques:
    data = output['data'][torque_req]
    speed_grid, soc_grid = np.meshgrid(velocities, socs)
    torque_grid = np.array([[data[soc][velocity]['differential_torque'] for velocity in velocities]
                            for soc in socs])
    if np.average(torque_grid) < 0:
        cmap = 'viridis_r'
    else:
        cmap = 'viridis'
    surf = ax.plot_surface(speed_grid, soc_grid, torque_grid, cmap=cmap)
plt.show()

print('siqmukmau')