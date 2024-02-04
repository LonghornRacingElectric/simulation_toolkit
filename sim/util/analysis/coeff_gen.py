import pandas as pd
import numpy as np
import copy
from scipy.optimize import basinhopping

from sim.system_models.vehicle_systems.tire_model52 import TireModel

class CoeffSolver:
    def __init__(self, initial_tire: TireModel = None, final_tire: TireModel = None, lat_file: str = "", long_combined_file: str = "", simplified: bool = True, iterations: int = 1):
        self.initial_tire = initial_tire
        
        self.initial_tire_copy = copy.deepcopy(initial_tire)
        self.final_tire = copy.deepcopy(final_tire)

        self.lat_data = lat_file
        self.long_combined_data = long_combined_file

        # Number of iterations for basin hopping algorithm
        self.iterations = iterations

        # Decreases number of data points to decrease runtime
        self.simplified = simplified
        self.combined = False

    def pure_lat_coeff_solve(self):
        data = pd.read_csv(self.lat_data)

        velocity = 25 * 1.60934
        pressure = 12 * 6.89476

        data = data[(data["velocity"] == velocity) & (data["pressure"] == pressure)]

        if self.simplified:
            n_skip = 50
        else:
            n_skip = 1

        self.FZ = list(data["FZ"] * -1)[::n_skip]
        self.SA = list(data["SA"] * np.pi / 180)[::n_skip]
        self.SR = list(0 for x in range(len(self.FZ)))
        self.IA = list(data["IA"] * np.pi / 180)[::n_skip]
        self.FY = list(data["FY"] * -1)[::n_skip]

        initial_guess = self.initial_tire_copy.pure_lat_coeffs

        self.combined = False

        lat_coeff_soln = basinhopping(self._lat_residual_calc, initial_guess, niter = self.iterations)
        lat_coeffs = lat_coeff_soln.x
        lat_residuals = lat_coeff_soln.fun

        return f"\nPure Lateral Coefficients: {list(lat_coeffs)}\nResidual: {lat_residuals}\n"
    
    def pure_long_coeff_solve(self):
        data = pd.read_csv(self.long_combined_data)

        velocity = 25 * 1.60934
        pressure = 12 * 6.89476
        slip_angle = 0

        data = data[(data["pressure"] == pressure) & (data["velocity"] == velocity) & (data["slip"] == slip_angle)]

        if self.simplified:
            n_skip = 50
        else:
            n_skip = 1

        self.FZ = list(data["FZ"] * -1)[::n_skip]
        self.SA = list(0 for x in range(len(self.FZ)))
        self.SR = list(data["SL"])[::n_skip]
        self.IA = list(data["IA"] * np.pi / 180)[::n_skip]
        self.FX = list(data["FX"])[::n_skip]

        self.combined = False

        initial_guess = self.initial_tire_copy.pure_long_coeffs

        long_coeff_soln = basinhopping(self._long_residual_calc, initial_guess, niter = self.iterations)
        long_coeffs = long_coeff_soln.x
        long_residuals = long_coeff_soln.fun

        return f"\nPure Longitudinal Coefficients: {list(long_coeffs)}\nResidual: {long_residuals}\n"
    
    def pure_aligning_coeff_solve(self):
        data = pd.read_csv(self.lat_data)

        velocity = 25 * 1.60934
        pressure = 12 * 6.89476

        data = data[(data["velocity"] == velocity) & (data["pressure"] == pressure) & (data["SA"] < 10) & (data["SA"] > -10)]

        if self.simplified:
            n_skip = 50
        else:
            n_skip = 1

        self.FZ = list(data["FZ"] * -1)[::n_skip]
        self.SA = list(data["SA"] * np.pi / 180 * -1)[::n_skip]
        self.SR = list(0 for x in range(len(self.FZ)))
        self.IA = list(data["IA"] * np.pi / 180)[::n_skip]
        self.MZ = list(data["MZ"] * -1)[::n_skip]

        initial_guess = self.initial_tire_copy.pure_aligning_coeffs

        aligning_coeff_soln = basinhopping(self._aligning_residual_calc, initial_guess, niter = self.iterations)
        aligning_coeffs = aligning_coeff_soln.x
        aligning_residuals = aligning_coeff_soln.fun

        return f"\nPure Aligning Moment Coefficients: {list(aligning_coeffs)}\nResidual: {aligning_residuals}\n"

    def _lat_tire_eval(self, lat_coeffs, FZ, SA, SR, IA):
        if self.combined:
            self.initial_tire_copy.combined_lat_coeffs = lat_coeffs
            FY = self.initial_tire_copy._combined_lat(data = [FZ, SA, SR, IA])
        else:
            self.initial_tire_copy.pure_lat_coeffs = lat_coeffs
            FY = self.initial_tire_copy._pure_lat(data = [FZ, SA, IA])

        return FY
    
    def _long_tire_eval(self, long_coeffs, FZ, SA, SR, IA):
        if self.combined:
            self.initial_tire_copy.combined_long_coeffs = long_coeffs
            FX = self.initial_tire_copy._combined_long(data = [FZ, SA, SR, IA])
        else:
            self.initial_tire_copy.pure_long_coeffs = long_coeffs
            FX = self.initial_tire_copy._pure_long(data = [FZ, SR, IA])
        
        return FX
    
    def _aligning_tire_eval(self, aligning_coeffs, FZ, SA, SR, IA):
        if self.combined:
            self.initial_tire_copy.combined_aligning_coeffs = aligning_coeffs
            MZ = self.initial_tire_copy._combined_aligning(data = [FZ, SA, SR, IA])
        else:
            self.initial_tire_copy.pure_aligning_coeffs = aligning_coeffs
            MZ = self.initial_tire_copy._pure_aligning(data = [FZ, SR, IA])
        
        return MZ
    
    def _lat_residual_calc(self, lat_coeffs):
        residuals = []
        
        for i in range(len(self.SA)):
            current_step = self._lat_tire_eval(lat_coeffs, self.FZ[i], self.SA[i], self.SR[i], self.IA[i])

            residual = self.FY[i] - current_step

            if self.combined:
                pass
            else:
                if abs(lat_coeffs[2]) < 0.75:
                    residuals.append(residual**2)
                else:
                    residuals.append(residual)

        print(f"\rCurrent Residual Norm: {np.linalg.norm(residuals)}", end = '')
        print("\nLat Coeffs" + str(list(lat_coeffs)))
        
        return np.linalg.norm(residuals)
    
    def _long_residual_calc(self, long_coeffs):
        residuals = []
        
        for i in range(len(self.SR)):
            current_step = self._long_tire_eval(long_coeffs, self.FZ[i], self.SA[i], self.SR[i], self.IA[i])

            residual = self.FX[i] - current_step

            if self.combined:
                pass
            else:
                for i in range(len(long_coeffs)):
                    if self.initial_tire_copy.pure_long_coeffs[i] != 0:
                        if long_coeffs[i] * np.sign(long_coeffs[i]) > 2 * self.initial_tire_copy.pure_long_coeffs[i] * np.sign(self.initial_tire_copy.pure_long_coeffs[i]):
                            residuals.append(residual**2)
                        elif long_coeffs[i] * np.sign(long_coeffs[i]) < 1/2 * self.initial_tire_copy.pure_long_coeffs[i] * np.sign(self.initial_tire_copy.pure_long_coeffs[i]):
                            residuals.append(residual**2)
                    else:
                        continue
                
                residuals.append(residual)

                # if long_coeffs[0] < 0:
                #     residuals.append(residual**2)
                # if long_coeffs[1] < 0:
                #     residuals.append(residual**2)
                # if long_coeffs[2] > 0:
                #     residuals.append(residual**2)
                # elif long_coeffs[3] < 0:
                #     residuals.append(residual**2)
                # elif long_coeffs[4] > 0:
                #     residuals.append(residual**2)
                # elif long_coeffs[5] > 0:
                #     residuals.append(residual**2)
                # elif long_coeffs[6] > 0:
                #     residuals.append(residual**2)
                # elif long_coeffs[7] < 0:
                #     residuals.append(residual**2)
                # elif long_coeffs[8] < 0:
                #     residuals.append(residual**2)
                # elif long_coeffs[9] < 0:
                #     residuals.append(residual**2)

                # if long_coeffs[4] > 0:
                #     residuals.append(residual**2)
                # if long_coeffs[8] < 0:
                #     residuals.append(residual**2)
                # if long_coeffs[9] < 0:
                #     residuals.append(residual**2)
                # if long_coeffs[10] > 0:
                #     residuals.append(residual**2)

                # if abs(long_coeffs[0]) > 2:
                #     residuals.append(residual**abs(long_coeffs[0]))
                # elif long_coeffs[1] > 3:
                #     residuals.append(residual**2)
                # if abs(long_coeffs[0]) > 2:
                #     residuals.append(residual**2)
                # if abs(long_coeffs[2]) < 0.75:
                #     residuals.append(residual**2)
                
                # residuals.append(residual)

        print(f"\rCurrent Residual Norm: {np.format_float_scientific(np.linalg.norm(residuals))}", end = '')
        print("\nLong Coeffs: " + str(list(long_coeffs)), end='')
        
        return np.linalg.norm(residuals)
    
    def _aligning_residual_calc(self, aligning_coeffs):
        residuals = []

        for i in range(len(self.SA)):
            current_step = self._aligning_tire_eval(aligning_coeffs, self.FZ[i], self.SA[i], self.SR[i], self.IA[i])

            residual = self.MZ[i] - current_step

            if (self.SA[i] > 3.5 * np.pi / 180) and (self.SA[i] < 5 * np.pi / 180):
                residuals.append(residual**2)
            elif (self.SA[i] < -3.5 * np.pi / 180) and (self.SA[i] > -5 * np.pi / 180):
                residuals.append(residual**2)
            elif (self.SA[i] < 1 * np.pi / 180) or (self.SA[i] > -1 * np.pi / 180):
                residuals.append(residual**2)
            else:
                residuals.append(residual)

        print(f"\rCurrent Residual Norm: {np.linalg.norm(residuals)}", end = '')
        print("\nAligning Coeffs" + str(list(aligning_coeffs)))

        return np.linalg.norm(residuals)