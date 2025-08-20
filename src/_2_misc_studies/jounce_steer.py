import sympy as sp


# Upper outboard
UOx, UOy, UOz = sp.symbols("UOx UOy UOz", real=True)

# Lower outboard
LOx, LOy, LOz = sp.symbols("LOx LOy LOz", real=True)

# Tie inboard
TIx, TIy, TIz = sp.symbols("TIx TIy TIz", real=True)

# Tie outboard
TOx, TOy, TOz = sp.symbols("TOx TOy TOz", real=True)

# System parameters
delta = sp.symbols("delta", real=True)

# Vectors
UO = sp.Matrix([UOx, UOy, UOz])
LO = sp.Matrix([LOx, LOy, LOz])
TI = sp.Matrix([TIx, TIy, TIz])
TO = sp.Matrix([TOx, TOy, TOz])

# Wheel rotation
kingpin_vec: sp.MutableDenseMatrix = UO - LO
u1x, u1y, u1z = kingpin_vec / kingpin_vec.norm()
cos_t_1 = sp.cos(delta)
sin_t_1 = sp.sin(delta)

rot = sp.Matrix([
    [u1x**2 * (1 - cos_t_1) + cos_t_1, u1x * u1y * (1 - cos_t_1) - u1z * sin_t_1, u1x * u1z * (1 - cos_t_1) + u1y * sin_t_1], 
    [u1x * u1y * (1 - cos_t_1) + u1z * sin_t_1, u1y**2 * (1 - cos_t_1) + cos_t_1, u1y * u1z * (1 - cos_t_1) - u1x * sin_t_1], 
    [u1x * u1z * (1 - cos_t_1) - u1y * sin_t_1, u1y * u1z * (1 - cos_t_1) + u1x * sin_t_1, u1z**2 * (1 - cos_t_1) + cos_t_1]
])

# Apply transformation
TO_T = TO - LO
TO_new = sp.MatMul(rot, TO_T) + LO

# Constraints
new_length: sp.MutableDenseMatrix = TO_new - TI
initial_length: sp.MutableDenseMatrix = TO - TI
length_constraint = sp.simplify(new_length.norm() - initial_length.norm())

soln = sp.solve(length_constraint, delta, domain=sp.S.Reals)