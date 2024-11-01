import pickle as pkl
import matplotlib.pyplot as plt
import os

os.chdir(r'C:\Users\andre\Documents\GitHub\simulation_toolkit\vehicle_model\powertrain_model\analysis\outputs')

filename = 'combined_wheel_torque~f(velocity, soc).pkl'
with open(filename, 'rb') as file:
    data = pkl.load(file)  # Load the figure directly

plt.show()

print('end')