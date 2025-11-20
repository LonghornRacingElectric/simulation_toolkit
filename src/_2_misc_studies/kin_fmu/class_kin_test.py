from src._3_custom_libraries.fmu_simulator import FMUSimulator
from src._2_misc_studies.kin_fmu.Kinematics import Kinematics_FMU
import numpy as np
import matplotlib.pyplot as plt

kin = Kinematics_FMU(fmu_path="./src/_2_misc_studies/kin_fmu/Kinematics2.fmu")
kin.run_simulation_jounce()


camber = kin.get_output_jounce('camber', degrees = True)
jounce_interp = np.interp(kin.time, kin.input_data['time'], kin.input_data['jounce'])

mask = (kin.time >= 0.5) & (kin.time <= 1.5)

camber = camber[mask]
jounce_interp = jounce_interp[mask]



#the class works


kin.run_simulation_roll()


# mask = (kin.rt >= 0.5) & (kin.rt <= 1.5)

# static_FL_camber_deg = -1
# camber_roll = kin.get_output_roll('camber_deg') + static_FL_camber_deg
# camber_roll = camber_roll[mask]

# toe_roll = kin.get_output_roll('toe_deg')
# toe_roll = toe_roll[mask]

# roll_deg = kin.roll_deg_fmu
# roll_deg = roll_deg[mask] # masked roll_interp
# kpi_roll = kin.get_output_roll('kpi_deg')[mask] + roll_deg

roll_deg = kin.roll_deg_fmu
roll_deg = roll_deg[mask] # masked roll_interp
trail_roll = kin.get_output_roll('trail_mm')[mask]






plt.figure()
plt.grid()
plt.plot(roll_deg, trail_roll)
plt.show()




# rt = roll_res['time']
# mask_r = (rt >= 0.5) & (rt <= 1.5)
# roll_deg = np.interp(rt[mask_r], roll_time, roll_deg_profile)


"""
Notes:
For Roll:
Camber => does not match
Toe => Matches
Caster => Matches
KPI => raw value + roll_deg = Matches
Mech Trail => need to figure out how to fix

"""
