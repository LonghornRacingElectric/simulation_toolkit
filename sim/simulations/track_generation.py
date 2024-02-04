import pandas as pd
import numpy as np
import warnings
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from scipy.optimize import fsolve
from scipy.optimize import basinhopping

class Track:
    def __init__(self, points, width, car_track, mesh):
        self.points = [list(row) for index, row in pd.read_csv(points).iterrows()]
        self.width = width
        self.car_track = car_track
        self.mesh = mesh

        self.parameter = [x for x in range(len(self.points))]
        self.spline_x = CubicSpline(self.parameter, [point[0] for point in self.points], bc_type = "periodic")
        self.spline_y = CubicSpline(self.parameter, [point[1] for point in self.points], bc_type = "periodic")

        self.arc_length_integral = self._arc_length(x_t = self.spline_x, y_t = self.spline_y)
        self.equidistant_params = self._equal_spacing(self.arc_length_integral)

        self.gates = [Gate(eq_p, self.spline_y.derivative(1)(eq_p) / self.spline_x.derivative(1)(eq_p), self.spline_x, self.spline_y) for eq_p in self.equidistant_params]

        # Optimal path generation
        # self.optimal = Path(self.gates, self.eq_t, width)

    def generate_track():
        pass

    def _arc_length(self, x_t: CubicSpline, y_t: CubicSpline):
        # Defines a list of t for calculations with discrete points
        discrete_t = np.linspace(0, self.parameter[-1], self.mesh)
        
        # Takes derivative of parameterized input splines
        dxdt = x_t.derivative(1)
        dydt = y_t.derivative(1)

        # Discretizes given splines
        discretized_dxdt = dxdt(discrete_t)
        discretized_dydt = dydt(discrete_t)

        # Performs the square operations in the arc length formula
        discretized_dxdt_squared = [dxdt**2 for dxdt in discretized_dxdt]
        discretized_dydt_squared = [dydt**2 for dydt in discretized_dydt]

        # Performs the sum in the arc length formula
        discretized_summation = [dxdt + dydt for (dxdt, dydt) in zip(discretized_dxdt_squared, discretized_dydt_squared)]

        # Performs the sqrt operation in the arc length formula
        discretized_integrand = [x**(1/2) for x in discretized_summation]

        return CubicSpline(discrete_t, discretized_integrand).antiderivative(1)
    
    def _equal_spacing(self, arc_length_integral):
        desired_spacing = arc_length_integral(self.parameter[-1]) / (arc_length_integral(self.parameter[-1]) // self.car_track)
        param_vals = [0]

        while arc_length_integral(param_vals[-1]) < arc_length_integral(self.parameter[-1]) - desired_spacing:
            param_vals.append(fsolve(lambda x: arc_length_integral(x) - arc_length_integral(param_vals[-1]) - desired_spacing, param_vals[-1])[0])
        
        return param_vals

    def _curvature_calc(self, spline_x: CubicSpline, spline_y: CubicSpline, param):
        # let r(t) = (x(t), y(t))

        r_dot_x, r_dot_y = spline_x.derivative(1)(param), spline_y.derivative(1)(param)

        r_double_dot_x, r_double_dot_y = spline_x.derivative(2)(param), spline_y.derivative(2)(param)

        # k = ||r'(t) x r''(t)|| / ||r'(t)||**3

        cross = abs(np.cross((r_dot_x, r_dot_y), (r_double_dot_x, r_double_dot_y)))

        return (cross / ((r_dot_x)**2 + (r_dot_y)**2)**(3/2))
    
    def _optimal_line(self):
        optimal_traversal = basinhopping(self._cost, [0 for x in range(len(self.equidistant_params))], niter = 2).x

        self.optimal_traversal = list(optimal_traversal)
    
    def _cost(self, initial_guess: list):
        iterated_points = []
        for i in range(len(self.gates)):
            iterated_points.append(self.gates[i].gate_traverse(initial_guess[i]))

        iterated_points.append(iterated_points[0])

        iterated_params = [x for x in range(len(iterated_points))]

        iterated_spline_x = CubicSpline(iterated_params, [point[0] for point in iterated_points], bc_type = "periodic")
        iterated_spline_y = CubicSpline(iterated_params, [point[1] for point in iterated_points], bc_type = "periodic")

        iterated_arc_length_integral = self._arc_length(x_t = iterated_spline_x, y_t = iterated_spline_y)
        iterated_equidistant_params = self._equal_spacing(iterated_arc_length_integral)

        cost = sum(np.abs([self._curvature_calc(iterated_spline_x, iterated_spline_y, param_val) for param_val in np.linspace(0, iterated_arc_length_integral(iterated_equidistant_params[-1]), self.mesh)]))

        for val in initial_guess:
            if abs(val) > self.car_track / 2:
                return cost**3

            return cost
    
    def plot(self):
        fig, ax = plt.subplots()

        plt.plot(self.spline_x(np.linspace(0, self.parameter[-1], self.mesh)), self.spline_y(np.linspace(0, self.parameter[-1], self.mesh)))
        # plt.scatter(self.spline_x(self.equidistant_params), self.spline_y(self.equidistant_params))

        for gate in self.gates:
            point = gate.bounds(self.car_track)
            plt.scatter(point[0][0], point[0][1], c = 'r')
            plt.scatter(point[1][0], point[1][1], c = 'r')

        optimal_points = []
        for i in range(len(self.gates)):
            optimal_points.append(self.gates[i].gate_traverse(self.optimal_traversal[i]))
        
        optimal_points.append(optimal_points[0])

        optimal_params = [x for x in range(len(optimal_points))]

        optimal_spline_x = CubicSpline(optimal_params, [point[0] for point in optimal_points], bc_type = "natural")
        optimal_spline_y = CubicSpline(optimal_params, [point[1] for point in optimal_points], bc_type = "natural")

        plt.plot(optimal_spline_x(np.linspace(0, optimal_params[-1], self.mesh)), optimal_spline_y(np.linspace(0, optimal_params[-1], self.mesh)))

        ax.set_aspect('equal')

        plt.show()

# This is kinda dumb, but I'll change it if this works
class Gate():
    def __init__(self, param, slope, spline_x, spline_y):
        self.spline_x = spline_x
        self.spline_y = spline_y
        self.param = param
        self.direction = self.unit_vector(-1 / slope)
        
    def unit_vector(self, slope):
        magnitude = (1 + slope**2)**(1/2)
        x = 1 / magnitude
        y = slope / magnitude

        return [x, y]
    
    # Defines the bounds of the gate
    def bounds(self, distance):
        start_x, start_y = self.spline_x(self.param), self.spline_y(self.param)

        x_1, y_1 = start_x + self.direction[0] * distance / 2, start_y + self.direction[1] * distance / 2

        x_2, y_2 = start_x - self.direction[0] * distance / 2, start_y - self.direction[1] * distance / 2

        return [x_1, y_1], [x_2, y_2]
    
    # Returns certesian point a certain distance along gate
    def gate_traverse(self, distance):
        start_x, start_y = self.spline_x(self.param), self.spline_y(self.param)
        x_1, y_1 = start_x + self.direction[0] * distance, start_y + self.direction[1] * distance

        return [x_1, y_1]