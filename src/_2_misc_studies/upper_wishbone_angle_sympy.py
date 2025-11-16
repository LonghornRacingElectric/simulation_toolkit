import sympy as sp

# Define the symbolic components
L1x, L1y, L1z, L2x, L2y, L2z, ux, uy, uz, theta2 = sp.symbols('L1x L1y L1z L2x L2y L2z ux uy uz theta2')

# Define L1 and L2 as vectors
L1 = sp.Matrix([L1x, L1y, L1z])
L2 = sp.Matrix([L2x, L2y, L2z])

# Normalize the axis of rotation vector
u = sp.Matrix([ux, uy, uz])
u_norm = u / sp.sqrt(u.dot(u))

# Rotation matrix for the second vector L2 around the axis u by angle theta2
L2_rot = L2 * sp.cos(theta2) + L2.cross(u_norm) * sp.sin(theta2) + u_norm * (u_norm.dot(L2)) * (1 - sp.cos(theta2))

# Distance constraint between the free ends of L1 and L2
distance_constraint = sp.sqrt((L1[0] - L2_rot[0])**2 + (L1[1] - L2_rot[1])**2 + (L1[2] - L2_rot[2])**2) - sp.sqrt(L1x**2 + L1y**2 + L1z**2)

# Solve for theta2 symbolically
solution_theta2 = sp.solve(distance_constraint, theta2)

# Print the solution for theta2
print(solution_theta2)
