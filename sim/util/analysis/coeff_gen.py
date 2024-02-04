import pandas as pd
import numpy as np
import copy
from scipy.optimize import basinhopping

from sim.system_models.vehicle_systems.tire_model52 import TireModel

class CoeffSolver:
    def __init__(self, 
                 initial_tire: TireModel = None, 
                 final_tire: TireModel = None, 
                 lat_file: str = "", 
                 long_combined_file: str = "", 
                 sample_rate: int = 50, 
                 iterations: int = 1,
                 FZ_range: list[int] = [100, 3000],
                 acc_coeff_dev: float = 2.0,
                 force_peak: bool = True
                 ):
        
        self.initial_tire = initial_tire
        
        self.initial_tire_copy = copy.deepcopy(initial_tire)
        self.final_tire = copy.deepcopy(final_tire)

        self.lat_data = lat_file
        self.long_combined_data = long_combined_file

        # Number of iterations for basin hopping algorithm
        self.iterations = iterations

        # Decreases number of data points to decrease runtime
        self.sample_rate = sample_rate
        self.combined = False

        # Sets range for valid fit
        self.FZ_range = FZ_range
        self.acc_coeff_dev = acc_coeff_dev
        self.force_peak = force_peak

    def pure_lat_coeff_solve(self):
        data = pd.read_csv(self.lat_data)

        velocity = 25 * 1.60934
        pressure = 12 * 6.89476

        self.data = data[(data["velocity"] == velocity) & (data["pressure"] == pressure)]

        self.FZ = list(self.data["FZ"] * -1)[::self.sample_rate]
        self.SA = list(self.data["SA"] * np.pi / 180)[::self.sample_rate]
        self.SR = list(0 for x in range(len(self.FZ)))
        self.IA = list(self.data["IA"] * np.pi / 180)[::self.sample_rate]
        self.FY = list(self.data["FY"] * -1)[::self.sample_rate]

        ### PLACEHOLDER
        self.bounds = []
        self.load = list(self.data["load"].unique())

        for load in self.load:
            FY_slice = self.data[(self.data["load"] == load)]
            max_FY, min_FY = max(FY_slice["FY"]), min(FY_slice["FY"])

            self.bounds.append([load, [min_FY, max_FY]])

        # Hacky solution to force desired behavior
        for bound in self.bounds:
            for i in range(100):
                self.FZ.append(abs(bound[0]))
                self.SR.append(0)
                self.SA.append(np.pi / 2)
                self.IA.append(0)
                self.FY.append(bound[1][0] * -1 * 0.85)

                self.FZ.append(abs(bound[0]))
                self.SR.append(0)
                self.SA.append(-np.pi / 2)
                self.IA.append(0)
                self.FY.append(bound[1][1] * -1 * 0.85)
        ### PLACEHOLDER

        self.combined = False

        initial_guesses = [self.initial_tire_copy.pure_lat_coeffs]
        for i in range(self.iterations):
            lat_coeff_soln = basinhopping(self._lat_residual_calc, initial_guesses[-1], niter = 1)
            lat_coeffs = lat_coeff_soln.x
            lat_residuals = lat_coeff_soln.fun

            lat_coeffs[-7:] = [0, 0, 0, 0, 0, 0, 0]

            initial_guesses.append(lat_coeffs)

        return [f"\nPure Lateral Coefficients: {list(lat_coeffs)}\nResidual: {lat_residuals}\n", list(lat_coeffs)]
    
    def pure_long_coeff_solve(self):
        data = pd.read_csv(self.long_combined_data)

        velocity = 25 * 1.60934
        pressure = 12 * 6.89476
        slip_angle = 0

        self.data = data[(data["pressure"] == pressure) & (data["velocity"] == velocity) & (data["slip"] == slip_angle)]

        self.FZ = list(self.data["FZ"] * -1)[::self.sample_rate]
        self.SA = list(0 for x in range(len(self.FZ)))
        self.SR = list(self.data["SL"])[::self.sample_rate]
        self.IA = list(self.data["IA"] * np.pi / 180)[::self.sample_rate]
        self.FX = list(self.data["FX"])[::self.sample_rate]

        ### PLACEHOLDER
        self.bounds = []
        self.load = list(self.data["load"].unique())

        for load in self.load:
            FX_slice = self.data[(self.data["load"] == load)]
            max_FX, min_FX = max(FX_slice["FX"]), min(FX_slice["FX"])

            self.bounds.append([load, [min_FX, max_FX]])

        # Hacky solution to force desired behavior
        for bound in self.bounds:
            for i in range(40):
                self.FZ.append(abs(bound[0]))
                self.SR.append(1)
                self.SA.append(0)
                self.IA.append(0)
                self.FX.append(bound[1][0] * -1 * 0.85)

                self.FZ.append(abs(bound[0]))
                self.SR.append(-1)
                self.SA.append(0)
                self.IA.append(0)
                self.FX.append(bound[1][1] * -1 * 0.85)
        ### PLACEHOLDER
                
        self.combined = False

        initial_guesses = [self.initial_tire_copy.pure_long_coeffs]
        for i in range(self.iterations):
            long_coeff_soln = basinhopping(self._long_residual_calc, initial_guesses[-1], niter = 1)
            long_coeffs = long_coeff_soln.x
            long_residuals = long_coeff_soln.fun

            long_coeffs[-4:] = [0, 0, 0, 0]

            initial_guesses.append(long_coeffs)

        return [f"\nPure Longitudinal Coefficients: {list(long_coeffs)}\nResidual: {long_residuals}\n", list(long_coeffs)]
    
    def pure_aligning_coeff_solve(self):
        data = pd.read_csv(self.lat_data)

        velocity = 25 * 1.60934
        pressure = 12 * 6.89476

        self.data = data[(data["velocity"] == velocity) & (data["pressure"] == pressure) & (data["SA"] < 10) & (data["SA"] > -10)]

        self.FZ = list(self.data["FZ"] * -1)[::self.sample_rate]
        self.SA = list(self.data["SA"] * np.pi / 180 * -1)[::self.sample_rate]
        self.SR = list(0 for x in range(len(self.FZ)))
        self.IA = list(self.data["IA"] * np.pi / 180)[::self.sample_rate]
        self.MZ = list(self.data["MZ"] * -1)[::self.sample_rate]

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
        # Disable translations
        lat_coeffs[-7:] = [0, 0, 0, 0, 0, 0, 0]

        residuals = []
        
        for i in range(len(self.SA)):
            current_step = self._lat_tire_eval(lat_coeffs, self.FZ[i], self.SA[i], self.SR[i], self.IA[i])

            residual = self.FY[i] - current_step

            if self.combined:
                pass
            else:
                # Explicit coefficient constraints
                if (lat_coeffs[0] < 1) or (lat_coeffs[0] > 2):
                    residuals.append(residual**2)
                if abs(lat_coeffs[2]) < 0.25:
                    residuals.append(residual**2)

                # Implicit coefficient constraint such that E_x <= 1 on desired domain
                if self.sample_rate >= 10:
                    model_FZ_data = np.linspace(self.FZ_range[0], self.FZ_range[1], 100)
                    model_SA_data = np.linspace(-np.pi / 2, np.pi / 2, 100)

                    X, Y = np.meshgrid(model_FZ_data, model_SA_data)

                    Z = self.initial_tire_copy._pure_lat(data = [X, Y, 0], sign_condition = True)

                    if Z.any():
                        residuals.append(residual**2)
                
                residuals.append(residual)

        print(f"\rCurrent Residual Norm: {np.format_float_scientific(np.linalg.norm(residuals), precision = 10)}     ", end = '')
        # print("\rLong Coeffs: " + str(list(long_coeffs)), end='')
        
        return np.linalg.norm(residuals)
    
    def _long_residual_calc(self, long_coeffs):
        # Disable translations
        long_coeffs[-4:] = [0, 0, 0, 0]

        residuals = []
        
        for i in range(len(self.SR)):
            current_step = self._long_tire_eval(long_coeffs, self.FZ[i], self.SA[i], self.SR[i], self.IA[i])

            residual = self.FX[i] - current_step

            if self.combined:
                pass
            else:
                # Explicit coefficient constraints
                if (long_coeffs[0] < 1) or (long_coeffs[0] > 2):
                    residuals.append(residual**2)
                if abs(long_coeffs[2]) < 0.5:
                    residuals.append(residual**2)

                # Implicit coefficient constraint such that E_x <= 1 on desired domain
                if self.sample_rate >= 10:
                    model_FZ_data = np.linspace(self.FZ_range[0], self.FZ_range[1], 100)
                    model_SR_data = np.linspace(-1, 1, 100)

                    X, Y = np.meshgrid(model_FZ_data, model_SR_data)

                    Z = self.initial_tire_copy._pure_long(data = [X, Y, 0], sign_condition = True)

                    if Z.any():
                        residuals.append(residual**2)
                
                residuals.append(residual)

        print(f"\rCurrent Residual Norm: {np.format_float_scientific(np.linalg.norm(residuals), precision = 10)}     ", end = '')
        # print("\rLong Coeffs: " + str(list(long_coeffs)), end='')
        
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