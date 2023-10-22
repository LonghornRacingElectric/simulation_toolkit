import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

tire_scaling = 0.55

lat_coeffs = [0.349, -0.00115, 8.760, 730.300, 1745.322, 0.0139, -0.000277, 1.02025435, 0, 0, 0, 0, 0, 0, 0, 0.00362, -0.0143, -0.0116]

long_coeffs = [0.46024966176377113, 4000.509873697152, 1097.1712081460967, 202.18848632159495, 100.8812198037175, -0.2557010431649166, 0.3066955241461764, 0.011822770671297778, -1.9521015799737094, 0, 0, 0, 0, 0]


def _com_long(SA: float, SR: float, FX: float, FY: float, Ca: float):
    FY = abs(FY)
    try:
        if (SR**2 * FY**2 + FX**2 * (np.tan(SA))**2 == 0 or Ca == 0):
            SA += 1e-64
            SR += 1e-64
            FX += 1e-64
            FY += 1e-64
            Ca += 1e-64
    except:
        pass

    adjusted_FX = ((FX * FY) / np.sqrt(SR**2 * FY**2 + FX**2 * (np.tan(SA))**2)) * \
            (np.sqrt(SR**2 * Ca**2 + (1 - SR)**2 * (np.cos(SA))**2 * FX**2) / Ca)
    
    return adjusted_FX

def _com_lat(SA: float, SR: float, FX: float, FY: float, Cs: float):
    FX = abs(FX)
    try:
        if (SR**2 * FY**2 + FX**2 * (np.tan(SA))**2 == 0 or Cs == 0):
            SA += 1e-64
            SR += 1e-64
            FX += 1e-64
            FY += 1e-64
            Cs += 1e-64
    except:
        pass

    adjusted_FY = ((FX * FY) / np.sqrt(SR**2 * FY**2 + FX**2 * (np.tan(SA))**2)) * \
        (np.sqrt((1 - SR)**2 * (np.cos(SA))**2 * FY**2 + (np.sin(SA))**2 * Cs**2) / (Cs * np.cos(SA)))
        
    return adjusted_FY

def _long_pacejka(data: list[float]):
    FZ = data[0] / 1000
    SR = data[1] * 100
    [b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12, b13] = long_coeffs

    # if FZ <= 0:
    #     return 0
    # else:
    C = b0
    D = FZ * (b1 * FZ + b2)
    
    BCD = (b3 * FZ**2 + b4 * FZ) * np.exp(-1 * b5 * FZ)
    B = BCD / (C * D)
    H = b9 * FZ + b10

    E = (b6 * FZ**2 + b7 * FZ + b8) * (1 - b13 * np.sign(SR + H))

    V = b11 * FZ + b12
    Bx1 = B * (SR + H)

    return tire_scaling * (D * np.sin(C * np.arctan(Bx1 - E * (Bx1 - np.arctan(Bx1)))) + V)

def _lat_pacejka(data: list[float]):
    FZ = data[0]
    SA = data[1] * 180 / np.pi
    IA = data[2] * 180 / np.pi
    [a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17] = lat_coeffs

    # if FZ <= 0:
    #     return 0
    # else:
    C = a0
    D = FZ * (a1 * FZ + a2) * (1 - a15 * IA**2)
    
    BCD = a3 * np.sin(np.arctan(FZ / a4) * 2) * (1 - a5 * abs(IA))
    B = BCD / (C * D)
    H = a8 * FZ + a9 + a10 * IA

    E = (a6 * FZ + a7) * (1 - (a16 * IA + a17) * np.sign(SA + H))

    V = a11 * FZ + a12 + (a13 * FZ + a14) * IA * FZ
    Bx1 = B * (SA + H)

    return tire_scaling * (D * np.sin(C * np.arctan(Bx1 - E * (Bx1 - np.arctan(Bx1)))) + V)

def _zero_protection(SA: float, SR: float):
    if type(SA) == float or type(SA) == int:
        if SA == 0:
            SA += 1e-64
    else:
        if SA.any() == 0:
            SA += 1e-64
        
    if type(SR) == float or type(SR) == int:
        if SR == 0:
            SR += 1e-64
    else:
        if SR.any() == 0:
            SR += 1e-64

    return [SA, SR]

def _get_comstock_forces(SA: float, SR: float, FZ: float, IA: float):
    SA, SR = _zero_protection(SA, SR)

    FX = _long_pacejka([FZ, SR])
    FY = _lat_pacejka([FZ, SA, IA])

    Ca = (_long_pacejka([FZ, 1 / 100]) - _long_pacejka([FZ, 0])) * (180 / np.pi) # slip stiffness
    Cs = (_lat_pacejka([FZ, 1 * np.pi / 180, IA]) - _lat_pacejka([FZ, 0, IA])) * 100 # cornering stiffness

    adj_FX = _com_long(SA, SR, FX, FY, Ca)
    adj_FY = _com_lat(SA, SR, FX, FY, Cs)

    return [adj_FX, adj_FY, FZ]

# model_SA_data = np.linspace(-np.pi / 2, np.pi / 2, 1000)
# model_FZ_data = np.linspace(500, 3000, 1000)
# # model_SR_data = np.linspace(0, 0, 1000)

# SA, FZ = np.meshgrid(model_SA_data, model_FZ_data)

# # FY1 = _get_comstock_forces(SA, 0, FZ, 0)[1]
# FY1 = _lat_pacejka([FZ, SA, 0])

# fig = plt.figure()
# ax = Axes3D(fig, auto_add_to_figure=False)

# ax = plt.axes(projection='3d')

# fig.add_axes(ax)
# ax.plot_surface(FZ, SA, FY1)
# # ax.plot_surface(FZ, SA, FY2)
# # ax.plot_surface(FZ, SA, FY3)
# # ax.plot_surface(FZ, SA, FY4)
# # ax.plot_surface(SR, SA, FY2)
# # ax.plot_surface(SR, SA, FY3)

# ax.set_xlabel('Normal Force (N)')
# ax.set_ylabel('Slip Angle (rad)')
# ax.set_zlabel('Lateral Friction Cofficient')
# ax.set_title('Normal Load Sensitivity')

# plt.show()

model_SR_data = np.linspace(-1, 1, 1000)
model_FZ_data = np.linspace(300, 1200, 1000)
# model_SR_data = np.linspace(0, 0, 1000)

SR, FZ = np.meshgrid(model_SR_data, model_FZ_data)

FX1 = _get_comstock_forces(0, SR, FZ, 0)[0]
# FY2 = _get_comstock_forces(SA, 0, FZ, 1 * np.pi / 180)[1]
# FY3 = _get_comstock_forces(SA, 0, FZ, 2 * np.pi / 180)[1]
# FY4 = _get_comstock_forces(SA, 0, FZ, 5 * np.pi / 180)[1]

# fig = plt.figure()
# ax = Axes3D(fig, auto_add_to_figure=False)

# ax = plt.axes(projection='3d')

# fig.add_axes(ax)
# ax.plot_surface(FZ, SR, FX1)
# # ax.plot_surface(FZ, SA, FY2)
# # ax.plot_surface(FZ, SA, FY3)
# # ax.plot_surface(FZ, SA, FY4)
# # ax.plot_surface(SR, SA, FY2)
# # ax.plot_surface(SR, SA, FY3)

# ax.set_xlabel('Normal Force (N)')
# ax.set_ylabel('Slip Ratio')
# ax.set_zlabel('Longitudinal Force (N)')
# ax.set_title('FX(SR, SA, FZ, IA)')

# Setup

FZ = 700

model_SA_data = np.linspace(0, np.pi / 2, 1000)
model_SR_data = np.linspace(0, 1, 1000)

SA, SR = np.meshgrid(model_SA_data, model_SR_data)

FX = _get_comstock_forces(SA, SR, FZ, 0)[0]
FY = _get_comstock_forces(SA, SR, FZ, 0)[1]

# FX2 = tire_model._get_comstock_forces(SA, SR, 500, 0 * np.pi / 180)[0]
# FY2 = tire_model._get_comstock_forces(SA, SR, 500, 0 * np.pi / 180)[1]

# FX3 = tire_model._get_comstock_forces(SA, SR, 500, -5 * np.pi / 180)[0]
# FY3 = tire_model._get_comstock_forces(SA, SR, 500, -5 * np.pi / 180)[1]

# Long

fig = plt.figure()
ax = Axes3D(fig, auto_add_to_figure=False)

ax = plt.axes(projection='3d')

fig.add_axes(ax)
ax.plot_surface(SR, SA, FX)
# ax.plot_surface(SR, SA, FX2)

ax.set_xlabel('Slip Ratio')
ax.set_ylabel('Slip Angle (rad)')
ax.set_zlabel('Longitudinal Force (N)')
ax.set_title('FX(SR, SA, FZ, IA)')
ax.invert_xaxis()

plt.show()