import sympy as sp
import numpy as np


# Upper inboard
UFIx, UFIy, UFIz = [2, 0, 1]
UAIx, UAIy, UAIz = [0, 0, 1]

# Upper outboard
OUx, OUy, OUz = [1, 3, 1]

# Lower inboard
LFIx, LFIy, LFIz = [2, 0, 0.25]
LAIx, LAIy, LAIz = [0, 0, 0]

# Lower outboard
OLx, OLy, OLz = [1, 3, 0]

# System parameters
dz, thetaL, thetaU = sp.symbols("dz thetaL thetaU", real=True)

# Vectors
UFI = sp.Matrix([UFIx, UFIy, UFIz])
UAI = sp.Matrix([UAIx, UAIy, UAIz])
OU = sp.Matrix([OUx, OUy, OUz])
LFI = sp.Matrix([LFIx, LFIy, LFIz])
LAI = sp.Matrix([LAIx, LAIy, LAIz])
OL = sp.Matrix([OLx, OLy, OLz])

# Lower wishbone rotation
lower_vec: sp.MutableDenseMatrix = LFI - LAI
u1x, u1y, u1z = lower_vec / lower_vec.norm()
cos_t_1 = sp.cos(thetaL)
sin_t_1 = sp.sin(thetaL)

R1 = sp.Matrix([
    [u1x**2 * (1 - cos_t_1) + cos_t_1, u1x * u1y * (1 - cos_t_1) - u1z * sin_t_1, u1x * u1z * (1 - cos_t_1) + u1y * sin_t_1], 
    [u1x * u1y * (1 - cos_t_1) + u1z * sin_t_1, u1y**2 * (1 - cos_t_1) + cos_t_1, u1y * u1z * (1 - cos_t_1) - u1x * sin_t_1], 
    [u1x * u1z * (1 - cos_t_1) - u1y * sin_t_1, u1y * u1z * (1 - cos_t_1) + u1x * sin_t_1, u1z**2 * (1 - cos_t_1) + cos_t_1]
])

# Upper wishbone rotation
upper_vec: sp.MutableDenseMatrix = UFI - UAI
u2x, u2y, u2z = upper_vec / upper_vec.norm()

cos_t_2 = sp.cos(thetaU)
sin_t_2 = sp.sin(thetaU)

R2 = sp.Matrix([
    [u2x**2 * (1 - cos_t_2) + cos_t_2, u2x * u2y * (1 - cos_t_2) - u2z * sin_t_2, u2x * u2z * (1 - cos_t_2) + u2y * sin_t_2],
    [u2x * u2y * (1 - cos_t_2) + u2z * sin_t_2, u2y**2 * (1 - cos_t_2) + cos_t_2, u2y * u2z * (1 - cos_t_2) - u2x * sin_t_2], 
    [u2x * u2z * (1 - cos_t_2) - u2y * sin_t_2, u2y * u2z * (1 - cos_t_2) + u2x * sin_t_2, u2z**2 * (1 - cos_t_2) + cos_t_2]
])

# Apply lower transformation
LI_T = LFI - LFI
OL_T = OL - LFI
OL_new = sp.MatMul(R1, OL_T) + LFI

# Apply upper transformation
UI_T = UFI - UFI
OU_T = OU - UFI
OU_new = sp.MatMul(R2, OU_T) + UFI

# Jounce and length constraints
jounce_constraint = OL_new[2] - dz
length_constraint = (OU_new - OL_new).norm() - (OU - OL).norm()

# Solve for theta vals symbolically
soln = sp.solve([jounce_constraint, length_constraint], (thetaL, thetaU))
# solution_thetaL = sp.solve(jounce_constraint, thetaL, domain=sp.S.Reals)
# solution_theta1, solution_theta2 = sp.solve([jounce_constraint, length_constraint], (thetaL, thetaU), domain=sp.S.Reals)

# Print the solution for theta2
eqn1 = sp.lambdify(dz, soln[0])
eqn2 = sp.lambdify(dz, soln[1])
eqn3 = sp.lambdify(dz, soln[2])
eqn4 = sp.lambdify(dz, soln[3])

print(np.array(eqn1(0.25)) * (180 / np.pi))
print(np.array(eqn2(0.25)) * (180 / np.pi))
print(np.array(eqn3(0.25)) * (180 / np.pi))
print(np.array(eqn4(0.25)) * (180 / np.pi))