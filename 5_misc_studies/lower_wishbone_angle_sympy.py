# import sympy as sp

# # Define the variables
# X1, Y1, Z1, Z2, ux, uy, uz, theta = sp.symbols('X1 Y1 Z1 Z2 ux uy uz theta')

# # Define intermediate terms
# A = X1 * uy - Y1 * ux
# B = ux * X1 + uy * Y1 + uz * Z1

# # Define the equation for Z2
# equation = Z2 - uz * B - (Z1 - uz * B) * sp.cos(theta) - A * sp.sin(theta)

# # Solve for theta
# solutions = sp.solve(equation, theta)
# print(solutions)

# import sympy as sp

# # Define the variables
# X1, Y1, Z1, Z2, ux, uy, uz, theta = sp.symbols('X1 Y1 Z1 Z2 ux uy uz theta')

# # Define the expression provided by SymPy
# numerator = (-X1*uy + Y1*ux + sp.sqrt(X1**2*uy**2 - 2*X1*Y1*ux*uy - 2*X1*Z1*ux*uz + 2*X1*Z2*ux*uz + Y1**2*ux**2 - 2*Y1*Z1*uy*uz + 2*Y1*Z2*uy*uz - 2*Z1**2*uz**2 + Z1**2 + 2*Z1*Z2*uz**2 - Z2**2))
# denominator = (2*X1*ux*uz + 2*Y1*uy*uz + 2*Z1*uz**2 - Z1 - Z2)

# # Compute the two solutions for theta
# theta1 = 2 * sp.atan(numerator / denominator)
# theta2 = -2 * sp.atan(numerator / denominator)

# # Print both solutions
# print(theta1)
# print()
# print(theta2)

import numpy as np

X1, Y1, Z1 = [0, 2, 0]
ux, uy, uz = [1, 0, 0]
Z2=0.25437

theta1 = 2*np.atan((-X1*uy + Y1*ux + np.sqrt(X1**2*uy**2 - 2*X1*Y1*ux*uy - 2*X1*Z1*ux*uz + 2*X1*Z2*ux*uz + Y1**2*ux**2 - 2*Y1*Z1*uy*uz + 2*Y1*Z2*uy*uz - 2*Z1**2*uz**2 + Z1**2 + 2*Z1*Z2*uz**2 - Z2**2))/(2*X1*ux*uz + 2*Y1*uy*uz + 2*Z1*uz**2 - Z1 - Z2))

theta2 = -2*np.atan((-X1*uy + Y1*ux + np.sqrt(X1**2*uy**2 - 2*X1*Y1*ux*uy - 2*X1*Z1*ux*uz + 2*X1*Z2*ux*uz + Y1**2*ux**2 - 2*Y1*Z1*uy*uz + 2*Y1*Z2*uy*uz - 2*Z1**2*uz**2 + Z1**2 + 2*Z1*Z2*uz**2 - Z2**2))/(2*X1*ux*uz + 2*Y1*uy*uz + 2*Z1*uz**2 - Z1 - Z2))

solns = sorted([theta1, theta2])

if Z2 < 0:
    soln = solns[0]
else:
    soln = solns[1]

if abs(soln) > np.pi / 2:
    soln = np.sign(soln) * np.pi - soln

print(soln * 180 / np.pi)