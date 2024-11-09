import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import scipy as sp

file_name = 'typeshit.csv'
with open(file_name) as f:
    data = pd.read_csv(f)

rpm_d = data['Differential RPM']
torque_d = data['Combined Axle Torque [Nm]']

points = np.column_stack((rpm_d, torque_d))
hull = sp.spatial.ConvexHull(points)

boundary = points[hull.vertices]
upper = np.array([])
for i in range(1,len(boundary)-2):
    p0, p1 = boundary[i], boundary[i-1]
    vect = [*(p0 - p1), 0]
    direction = np.cross([0, 0, 1], vect)
    if direction[1] < 0:
        upper = np.append(upper, i)
        upper = np.append(upper, i-1)

upper = boundary[np.unique(upper.astype(int))]
upper = upper[np.argsort(upper[:, 0])]

mesh = 200
rpms = np.linspace(0, max(rpm_d), mesh)
interp = sp.interpolate.interp1d(upper[:, 0], upper[:, 1], fill_value='extrapolate')
upper = interp(rpms)
upper = np.column_stack((rpms, upper))
pwr_lim = [80000/(rpms[i]*2*np.pi/60) for i in range(len(rpms))]
target = np.minimum(upper[:, 1], pwr_lim)
rpm2vel = 2*np.pi*8*0.0254/60
# peak_ind = int(np.where(target == max(target))[0][0])

plt.figure(figsize=(10, 5))
# plt.plot(boundary[np.arange(0, 5, 1), 0], boundary[np.arange(0, 5, 1), 1], color='r')
plt.scatter(rpm_d*rpm2vel, torque_d, label='torque potential states', c='k', s=3)
plt.plot(rpms*rpm2vel, target, color=(1, 0, 0), label='target', linewidth=5)
plt.plot(upper[:,0]*rpm2vel, upper[:,1], color='b', linewidth=3, label='upper hull', linestyle='--', alpha=0.7)
# plt.plot(boundary[:, 0], boundary[:, 1], color='k', label='convex hull')
plt.plot(rpms*rpm2vel, pwr_lim, color='g', label='100% efficiency + power limit', linewidth=3, linestyle='--',
         alpha=0.7)
plt.ylim(min(upper[:, 1]*0.8), max(upper[:, 1])*1.3)
plt.xlabel('velocity [m/s]')
plt.ylabel('torque [Nm]')
plt.legend()
plt.show()

output = np.column_stack((rpms*rpm2vel, target))
output = pd.DataFrame(output, columns=['velocity', 'torque'])

file_name = 'torque_speed_req.csv'
output.to_csv(file_name,index=False)

print('siqmukmau')
