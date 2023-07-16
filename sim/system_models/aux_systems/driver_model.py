"""

TODO x, x_dot in --> driver controls out

Remember that the driver needs to know the racing line to try to match it in a transient sim.
For a simple scenario (like a single turn) the driver can be trying to follow a simple spline instead.
For simple scenarios, we can alternatively have the driver be following "instructions" instead of a path,
like "accelerate for 1 second, then turn steering wheel 10 degrees to the right for 1 second".

Driver also needs to know the performance envelope so they can try to operate near its boundaries.
This implies that the quasi-steady state sim needs to be done before trying to model laps in a transient sim.

"""
