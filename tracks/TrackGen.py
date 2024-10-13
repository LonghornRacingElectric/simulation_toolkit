import numpy as np
from scipy.interpolate import CubicSpline
from scipy.integrate import quad
from scipy.optimize import minimize
import matplotlib.pyplot as plt

class Track():
    """## Track Object

    Parameters
    ----------
    track_points_inner : list
        list of cartesian coordinates representing inner cones of a given track
    track_points_outer : list
        list of cartesian coordinates representing outer cones of a given track
    trackwidth : float
        average distance between left and right contact patches
    """
    def __init__(self, track_points_inner: list, track_points_outer: list, trackwidth: float) -> None:
     
        self.track_points_inner = track_points_inner
        self.track_points_outer = track_points_outer
        self.trackwidth = trackwidth
        track_offset = trackwidth/2
  
    # Sequential calling of methods to generate optimal line 
    def run_optimize(self):

        # Get midpoints between inner and outer cones
        self.midpoints, self.trackx_mid, self.tracky_mid = self.get_midpoints(self.track_points_inner, self.track_points_outer)
        
        # Define list of ts for Cubic Splines to be defined parametrically
        cubic_spline_ts = [x for x in range(1, len(self.track_points_inner) + 1)]

        # Cubic splines for x(t) and y(t)
        x_spline = CubicSpline(cubic_spline_ts, self.trackx_mid)
        y_spline = CubicSpline(cubic_spline_ts, self.tracky_mid)

        # First and second derivatives of the splines 
        x_prime = x_spline.derivative(1)
        x_double_prime = x_spline.derivative(2)
        y_prime = y_spline.derivative(1)
        y_double_prime = y_spline.derivative(2)

        # Integrate curvature to get initial total curvature
        self.total_curvature_initial, _ = quad(func=self.curvature, a=cubic_spline_ts[0], b=cubic_spline_ts[-1], args=(x_prime, y_prime, x_double_prime, y_double_prime))

        # Integrate to find initial total arc length
        self.curve_length_total_initial, _ = quad(self.curve_length, cubic_spline_ts[0], cubic_spline_ts[-1], args=(x_prime, y_prime))
       
        # Compute initial average curvature
        self.average_curvature_initial = self.total_curvature_initial / self.curve_length_total_initial

        # Initial guess of offsets
        offsets_list = np.zeros(len(self.track_points_inner))
        
        # Use minimize function to solve for optimal line
        self.final_offsets = minimize(fun=self._optimize_helper, x0=offsets_list, method='SLSQP').x
        trackx_opt, tracky_opt =  map(list, zip(*[self.displace_midpoint(inner, outer, optimal) for inner, outer, optimal in zip(self.track_points_inner, self.track_points_outer, self.final_offsets)]))
        self.track_points_opt = [list(pair) for pair in zip(trackx_opt, tracky_opt)]
        self.track_points_opt[0] = self.track_points_opt[-1]

        # Generate CubicSplines of optimal solution
        x_spline = CubicSpline(cubic_spline_ts, trackx_opt)
        y_spline = CubicSpline(cubic_spline_ts, tracky_opt)

        # First and second derivatives of the splines 
        x_prime = x_spline.derivative(1)
        x_double_prime = x_spline.derivative(2)
        y_prime = y_spline.derivative(1)
        y_double_prime = y_spline.derivative(2)

        # Integrate curvature to get optimal total curvature
        self.total_curvature_opt, _ = quad(func=self.curvature, a=cubic_spline_ts[0], b=cubic_spline_ts[-1], args=(x_prime, y_prime, x_double_prime, y_double_prime))
        
        # Integrate to find the optimal total arc length
        self.curve_length_total_opt, _ = quad(self.curve_length, cubic_spline_ts[0], cubic_spline_ts[-1], args=(x_prime, y_prime))
        
        # Compute the optimal average curvature
        self.average_curvature_opt = self.total_curvature_opt / self.curve_length_total_opt
        return

    # Plot lines connecting inner and outer cones of the track
    def connect_cones(self, track_points_inner, track_points_outer):
        for (xi,yi), (xo,yo) in zip(track_points_inner, track_points_outer):
            plt.plot([xi,xo],[yi,yo], 'k-')
        return
    
    # Given two lists of cartesian points, return a list of the midpoints
    def get_midpoints(self, track_points_inner, track_points_outer):
        midpoints = []
        for (xi,yi), (xo,yo) in zip(track_points_inner, track_points_outer):
            midpoint_x = (xi + xo) / 2
            midpoint_y = (yi + yo) / 2
            midpoints.append((midpoint_x, midpoint_y))
        mid_x = [x[0] for x in midpoints]
        mid_y = [y[1] for y in midpoints]
        return(midpoints, mid_x, mid_y)      

    # Generate a parametric CubicSpline through a set of given cartesian points
    def track_limits(self, track_points):
        x = [coordinate[0] for coordinate in track_points]
        y = [coordinate[1] for coordinate in track_points]
        cubic_spline_ts = [x for x in range(1, len(track_points) + 1)]
        xi = CubicSpline(cubic_spline_ts, x, bc_type='periodic')
        yi = CubicSpline(cubic_spline_ts, y, bc_type='periodic')
        ts = np.arange(1, len(track_points), 0.01)
        xi = xi(ts)
        yi = yi(ts)
        return xi, yi

    # Calculate curvature 
    def curvature(self, t, x_prime, y_prime, x_double_prime, y_double_prime):
        dx_dt = x_prime(t)
        dy_dt = y_prime(t)
        d2x_dt2 = x_double_prime(t)
        d2y_dt2 = y_double_prime(t)
        
        numerator = np.abs(dx_dt * d2y_dt2 - dy_dt * d2x_dt2)
        denominator = (dx_dt**2 + dy_dt**2)**(3/2)
        
        return (numerator / denominator)

    # Calculate arc length of a curve
    def curve_length(self, t, x_prime, y_prime):
        dx_dt = x_prime(t)
        dy_dt = y_prime(t)
        return np.sqrt(dx_dt**2 + dy_dt**2)

    #Shift the midpoint of a line along that line by a scalar value. Shift is bounded by original two points
    def displace_midpoint(self,inner, outer, offsets_list):
        x1, y1 = inner
        x2, y2 = outer
        xm = (x1 + x2) / 2
        ym = (y1 + y2) / 2
        vx = x2 - x1
        vy = y2 - y1
        length = np.sqrt(vx**2 + vy**2)
        vx_norm = vx / length
        vy_norm = vy / length
        new_xm = xm + offsets_list * vx_norm
        new_ym = ym + offsets_list * vy_norm
        
        min_x = min(x1, x2)
        max_x = max(x1, x2)
        min_y = min(y1, y2)
        max_y = max(y1, y2)
        bounded_xm = max(min_x, min(new_xm, max_x))
        bounded_ym = max(min_y, min(new_ym, max_y))
        
        return bounded_xm, bounded_ym
    
    # Function to call using minimize function
    def _optimize_helper(self, x):
        trackx_mid1, tracky_mid1 = map(list, zip(*[self.displace_midpoint(inner, outer, offsets) for inner, outer, offsets in zip(self.track_points_inner, self.track_points_outer, x)]))
        trackx_mid1[0] = trackx_mid1[len(trackx_mid1)-1]
        tracky_mid1[0] = tracky_mid1[len(tracky_mid1)-1]

        cubic_spline_ts = [x for x in range(1, len(self.track_points_inner) + 1)]

        # Cubic splines for x(t) and y(t)
        x_spline = CubicSpline(cubic_spline_ts, trackx_mid1)
        y_spline = CubicSpline(cubic_spline_ts, tracky_mid1)

        # First and second derivatives of the splines 
        x_prime = x_spline.derivative(1)
        x_double_prime = x_spline.derivative(2)
        y_prime = y_spline.derivative(1)
        y_double_prime = y_spline.derivative(2)

        # Integrate curvature to get the total curvature
        total_curvature, _ = quad(func=self.curvature, a=cubic_spline_ts[0], b=cubic_spline_ts[-1], args=(x_prime, y_prime, x_double_prime, y_double_prime))

        # Integrate to find the total arc length
        curve_length_total, _ = quad(self.curve_length, cubic_spline_ts[0], cubic_spline_ts[-1], args=(x_prime, y_prime))

        # Compute average curvature
        average_curvature = total_curvature / curve_length_total

        return total_curvature

    # Plot solution on track
    def plot(self, axes, aspect_ratio_bool, legend_bool):
        # Calculate track limits
        xi, yi = self.track_limits(self.track_points_inner)
        xo, yo = self.track_limits(self.track_points_outer)
        xm, ym = self.track_limits(self.midpoints)
        xmf, ymf = self.track_limits(self.track_points_opt)

        # Unzip the track points
        trackx_inner, tracky_inner = map(list, zip(*self.track_points_inner))
        trackx_outer, tracky_outer = map(list, zip(*self.track_points_outer))

        # Plot on the provided axes
        axes.plot(trackx_inner, tracky_inner, 'bo', label='Inner Track Points')
        axes.plot(trackx_outer, tracky_outer, 'ro', label='Outer Track Points')
        axes.plot(self.trackx_mid, self.tracky_mid, 'mo', label='Midpoints')
        axes.plot(xi, yi, '-b', label='Inner Limit')
        axes.plot(xo, yo, '-r', label='Outer Limit')
        axes.plot(xm, ym, '-m', label='Midpoint Line')
        axes.plot(xmf, ymf, '-g', label='Optimized Track Points')
        
        # Add legend 
        
        if aspect_ratio_bool == True:
            # Ensure equal aspect ratio for the plot
            axes.set_aspect('equal')
        if legend_bool == True:
            axes.legend(loc='lower right')
        
        # Draw connections between cones
        # self.connect_cones(self.track_points_inner, self.track_points_outer)
        for (xi,yi), (xo,yo) in zip(self.track_points_inner, self.track_points_outer):
            axes.plot([xi,xo],[yi,yo], 'k-')

        # Return the figure object
        return axes.figure, axes.legend

      