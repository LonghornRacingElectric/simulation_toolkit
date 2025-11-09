import numpy as np
import matplotlib.pyplot as plt
from copy import deepcopy
from src.vehicle_model.suspension_model.suspension import Suspension
from src.vehicle_model.suspension_model.suspension_data import SuspensionData

sus_data = SuspensionData(path="./src/_1_model_inputs/Nightwatch.yml")
base_susp = Suspension(sus_data=sus_data)

roll_range_deg = 5
n_points = 100
roll_angles = np.linspace(-roll_range_deg, roll_range_deg, n_points)

results = {
    'roll': roll_angles,
    'camber_FL': [], 'camber_FR': [],
    'toe_FL': [], 'toe_FR': [],
    'caster_FL': [], 'caster_FR': [],
    'kpi_FL': [], 'kpi_FR': [],
    'RCy_Front': [], 'RCz_Front': [],
    'RCy_Rear': [], 'RCz_Rear': []
}

for roll_angle in roll_angles:
    suspension = deepcopy(base_susp)
    suspension.roll(roll=roll_angle)
    results['camber_FL'].append(suspension.FL_gamma)
    results['camber_FR'].append(suspension.FR_gamma)
    results['toe_FL'].append(suspension.FL_delta)
    results['toe_FR'].append(suspension.FR_delta)
    results['caster_FL'].append(suspension.FL_caster)
    results['caster_FR'].append(suspension.FR_caster)
    results['kpi_FL'].append(suspension.FL_kpi)
    results['kpi_FR'].append(suspension.FR_kpi)
    results['RCy_Front'].append(suspension.Fr_RC[1])
    results['RCz_Front'].append(suspension.Fr_RC[2])
    results['RCy_Rear'].append(suspension.Rr_RC[1])
    results['RCz_Rear'].append(suspension.Rr_RC[2])

for key in results:
    results[key] = np.array(results[key])

plt.figure()
plt.plot(results['roll'], results['camber_FL'], label='Camber FL')
plt.plot(results['roll'], results['camber_FR'], label='Camber FR')
plt.xlabel('Roll Angle [deg]')
plt.ylabel('Camber [deg]')
plt.title('Camber vs Roll')
plt.grid(True)
plt.legend()
plt.show()

plt.figure()
plt.plot(results['roll'], results['toe_FL'], label='Toe FL')
plt.plot(results['roll'], results['toe_FR'], label='Toe FR')
plt.xlabel('Roll Angle [deg]')
plt.ylabel('Toe [deg]')
plt.title('Toe vs Roll')
plt.grid(True)
plt.legend()
plt.show()

plt.figure()
plt.plot(results['roll'], results['caster_FL'], label='Caster FL')
plt.plot(results['roll'], results['caster_FR'], label='Caster FR')
plt.xlabel('Roll Angle [deg]')
plt.ylabel('Caster [deg]')
plt.title('Caster vs Roll')
plt.grid(True)
plt.legend()
plt.show()

plt.figure()
plt.plot(results['roll'], results['kpi_FL'], label='KPI FL')
plt.plot(results['roll'], results['kpi_FR'], label='KPI FR')
plt.xlabel('Roll Angle [deg]')
plt.ylabel('KPI [deg]')
plt.title('KPI vs Roll')
plt.grid(True)
plt.legend()
plt.show()

plt.figure()
plt.plot(results['roll'], results['RCy_Front'] * 1000, label='Front RC Y')
plt.plot(results['roll'], results['RCy_Rear'] * 1000, label='Rear RC Y')
plt.xlabel('Roll Angle [deg]')
plt.ylabel('RC Y [mm]')
plt.title('Roll Center Y vs Roll')
plt.grid(True)
plt.legend()
plt.show()

plt.figure()
plt.plot(results['roll'], results['RCz_Front'] * 1000, label='Front RC Z')
plt.plot(results['roll'], results['RCz_Rear'] * 1000, label='Rear RC Z')
plt.xlabel('Roll Angle [deg]')
plt.ylabel('RC Z [mm]')
plt.title('Roll Center Z vs Roll')
plt.grid(True)
plt.legend()
plt.show()
