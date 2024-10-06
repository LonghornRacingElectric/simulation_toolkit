import numpy as np
from scipy.interpolate import CubicSpline
from scipy.integrate import quad
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from TrackGen import Track as Track

if __name__ == "__main__":
    # T-Shape
    track_points_inner = [[0,0], [1,0], [1.5,0], [2,0], [3,0], [3,0.5], [3,1], [2,1], [1.5,1], [1,1], [0,1], [-1,2], [-2,2], [-2,1], [-2,0.5], [-2,0],
                        [-2,-1], [-1,-1], [0,0]]
    track_points_outer = [[0,-1], [1, -1], [1.5,-1], [2,-1], [3.75,-0.5], [4, 0.5], [3.75,1.5], [2, 2], [1.5,2], [1, 2], [0,2], [-1,3],
                        [-2.5,2.5], [-3,1], [-3,0.5], [-3,0], [-2.5,-1.5], [-1,-2], [0,-1]]
    
    new_Track = Track(track_points_inner, track_points_outer, trackwidth=0)
    new_Track.run_optimize()
    new_Track.plot()
    print(f'Initial Total Curvature {new_Track.curve_length_total_initial}')
    print(f'Optimal Total Curvature {new_Track.curve_length_total_opt}')

   

    




