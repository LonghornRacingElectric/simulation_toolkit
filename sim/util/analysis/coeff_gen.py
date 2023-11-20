import warnings

import pandas as pd
import numpy as np
from scipy.optimize import basinhopping
from sim.system_models.vehicle_systems.tire_model import TireModel


class CoeffSolver():
    def __init__(self):
        self.new_tire = TireModel()
        self.new_tire.tire_scaling = 1
        self.lat_coeffs = []
        self.long_coeffs = []
        self.count = 0

    def _lat_tire_eval(self, lat_coeffs, FZ, SA, IA):
        self.new_tire.lat_coeffs = lat_coeffs

        FY = self.new_tire._lat_pacejka(data = [FZ, SA, IA])

        return FY
    
    def _long_tire_eval(self, long_coeffs, FZ, SR):
        self.new_tire.long_coeffs = long_coeffs

        FX = self.new_tire._long_pacejka(data = [FZ, SR])

        return FX
    
    def _lat_residual_calc(self, lat_coeffs):

        residuals = []
        
        self.count += 1
        for i in range(len(self.SA)):

            current_step = self._lat_tire_eval(lat_coeffs, self.FZ[i], self.SA[i], self.IA[i])

            residuals.append(self.FY[i] - current_step)

        print(f"\rCurrent Residual Norm: {np.linalg.norm(residuals)}", end = '')
        print("\n" + str(list(lat_coeffs)))
        
        return np.linalg.norm(residuals)
    
    def _long_residual_calc(self, long_coeffs):

        residuals = []
        
        self.count += 1
        for i in range(len(self.SR)):

            current_step = self._long_tire_eval(long_coeffs, self.FZ[i], self.SR[i])

            residuals.append(self.FX[i] - current_step)

            # print(self.FX[i] - current_step)

        print(f"\rCurrent Residual Norm: {np.linalg.norm(residuals)}", end = '')
        print("\n" + str(list(long_coeffs)))
        
        return np.linalg.norm(residuals)

    def lat_coeff_solve(self, file):
        data = pd.read_csv(file)

        bounds = [[-1112, [-2776, 2718]], [-890, [-2270, 2455]], [-667, [-1740, 1710]], [-445, [-1199, 1175]], [-222, [-1066, 612]]]

        velocity = 25 * 1.60934
        pressure = 12 * 6.89476

        data = data[(data["velocity"] == velocity) & (data["pressure"] == pressure)]

        self.FZ = list(abs(data["FZ"]))
        self.SA = list(data["SA"] * np.pi / 180)
        self.IA = list(data["IA"] * np.pi / 180)
        self.FY = list(data["FY"] * -1)

        # Hacky solution to force desired behavior
        for bound in bounds:
            for i in range(1000):
                self.FZ.append(abs(bound[0]))
                self.SA.append(np.pi / 2)
                self.IA.append(0)
                self.FY.append(bound[1][0] * -1 * 0.70)
                
                self.FZ.append(abs(bound[0]))
                self.SA.append(-np.pi / 2)
                self.IA.append(0)
                self.FY.append(bound[1][1] * -1 * 0.70)

        for bound in bounds:
            for i in range(1000):
                self.FZ.append(abs(bound[0]))
                self.SA.append(np.pi / 4)
                self.IA.append(0)
                self.FY.append(bound[1][0] * -1 * 0.85)
                
                self.FZ.append(abs(bound[0]))
                self.SA.append(-np.pi / 4)
                self.IA.append(0)
                self.FY.append(bound[1][1] * -1 * 0.85)

        initial_guess = [0.349, -0.00115, 8.760, 730.300, 1745.322, 0.0139, -0.000277, 1.02025435, 0, 0, 0, 0, 0, 0, 0, 0.00362,
             -0.0143, -0.0116]
        
        # initial_guess = [0.00015426295838667575, -0.8448024800316304, 10284.996684596968, 21.974135503451595, 1586.1840487533805, 0.0165196091528276, -24.1870599222551, 123427.91522721709, -7.088465549749836e-05, 0.08572686889617373, -0.12196001044217282, -0.021123587066649917, 8.720677811019806, 2.0088692429980664e-05, -0.0017158961857240327, 0.003467137744138384, -0.05201974356941957, 0.08541692573710813]

        # lat_coeff_soln = basinhopping(self._lat_residual_calc, [0 for x in range(18)])
        lat_coeff_soln = basinhopping(self._lat_residual_calc, initial_guess, niter = 1)
        lat_coeffs = lat_coeff_soln.x
        lat_residuals = lat_coeff_soln.fun

        return [lat_coeffs, lat_residuals]

    def long_coeff_solve(self, file):
        data = pd.read_csv(file)

        bounds = [[-1112, [-3130, 3027]], [-890, [-2657, 2540]], [-667, [-2069, 1961]], [-445, [-1199, 1175]], [-222, [-889, 771]]]

        self.FZ = list(abs(data["load"]))
        self.SR = list(data["SL"])
        self.FX = list(data["FX"])

        # Hacky solution to force desired behavior
        for bound in bounds:
            for i in range(25):
                self.FZ.append(abs(bound[0]))
                self.SR.append(1)
                self.FX.append(bound[1][1] * 0.70)
                
                self.FZ.append(abs(bound[0]))
                self.SR.append(-1)
                self.FX.append(bound[1][0] * 0.70)

        # for bound in bounds:
        #     for i in range(25):
        #         self.FZ.append(abs(bound[0]))
        #         self.SR.append(0.50)
        #         self.FX.append(bound[1][1] * 0.85)
                
        #         self.FZ.append(abs(bound[0]))
        #         self.SR.append(-0.50)
        #         self.FX.append(bound[1][0] * 0.85)

        initial_guess = [0.46024966176377113, 4000.509873697152, 1097.1712081460967, 202.18848632159495, 100.8812198037175, -0.2557010431649166, 0.3066955241461764, 0.011822770671297778, -1.9521015799737094, 0, 0, 0, 0, 0]
        
        long_coeff_soln = basinhopping(self._long_residual_calc, initial_guess, niter = 10)
        long_coeffs = long_coeff_soln.x
        long_residuals = long_coeff_soln.fun

        return [long_coeffs, long_residuals]