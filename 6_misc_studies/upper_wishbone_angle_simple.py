import sympy as sp

# Define symbols
theta2 = sp.symbols('theta2')

# Prescribed components for the vectors L1 and L2 (these are arbitrary fixed values)
L1x, L1y, L1z = 1, 2, 3  # Example values for L1
L2x, L2y, L2z = 4, 5, 6  # Example values for L2

# Define vectors L1 and L2
L1 = sp.Matrix([L1x, L1y, L1z])
L2 = sp.Matrix([L2x, L2y, L2z])

# Rotation matrix for rotation about the x-axis by angle theta2
R_x2 = sp.Matrix([[1, 0, 0],
                  [0, sp.cos(theta2), -sp.sin(theta2)],
                  [0, sp.sin(theta2), sp.cos(theta2)]])

# Apply the rotation matrix to the vector L1 (L1 is not rotated since we are only rotating L2)
L1_rot = L1  # L1 remains the same since we are only rotating L2

# Apply the rotation to the vector L2
L2_rot = R_x2 * L2

# Set the distance constraint (distance between the free ends of L1 and L2)
distance = sp.sqrt((L1_rot[0] - L2_rot[0])**2 + (L1_rot[1] - L2_rot[1])**2 + (L1_rot[2] - L2_rot[2])**2)

# The distance constraint is constant, set it to some value 'd'
d = sp.symbols('d')
constraint = sp.Eq(distance, d)

# Solve for theta2 (rotation angle of L2) to meet the distance constraint
solution = sp.solve(constraint, theta2)
print("Solution for theta2:", solution)
