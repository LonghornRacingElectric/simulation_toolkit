import warnings

import pandas as pd
import numpy as np
from scipy.optimize import basinhopping
from scipy.optimize import minimize
from scipy.optimize import fsolve
from scipy.optimize import curve_fit
from sim.model_parameters.cars.car import Car
from sim.system_models.vehicle_systems.tire_model52 import TireModel
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class CoeffSolver:
    def __init__(self, car: Car):
        self.new_tire = TireModel()
        self.output_df = pd.DataFrame()
        self.output_df["Norm"] = 1
        self.output_df["Value"] = 2
        self.new_tire.tire_scaling = 1
        self.new_tire.pure_lat_coeffs = []
        self.new_tire.pure_lat_coeffs_start = []
        self.new_tire.pure_lat_coeffs_end = []
        self.new_tire.pure_long_coeffs = []
        self.new_tire.combined_long_coeffs = []
        self.new_tire.pure_aligning_coeffs = []
        self.new_tire.scaling_coeffs = [1 for x in range(28)]
        self.new_tire.scaling_coeffs_start = []
        self.new_tire.scaling_coeffs_start = []
        self.R_nom = 8 * 0.0254
        self.FZ_nom = 350 * 4.44822
        self.combined = True
        self.simplified = False
        self.count = 0

    def _lat_tire_eval(self, lat_coeffs, FZ_nom, FZ, SA, SR, IA):
    # def _lat_tire_eval(self, lat_coeffs, FZ_nom, FZ, SA, IA):
        # self.new_tire.pure_lat_coeffs = [1.362104845652448, -2.534964759267956, 0.24936941428001383, 9.423655269024566, 0.16577217779023068, -0.04443738731730766, 0.8075682362122878, 42.81074269203433, 31.075902567400338, 1.266100015792526, 0.610821810411664, -0.002062977533901501, -0.0014836344155698248, -0.07082946407655061, 0.018678104163062363, -0.022050227061089923, -1.4250303758548495, -1.0851392413060768]

        self.new_tire.pure_lat_coeffs = lat_coeffs

        # self.new_tire.combined_lat_coeffs = lat_coeffs

        # if self.combined:
        #     FY = self.new_tire._combined_lat(data = [FZ_nom, FZ, SA, SR, IA])
        # else:
        FY = self.new_tire._pure_lat(data = [FZ_nom, FZ, SA, IA])

        return FY
    
    def _long_tire_eval(self, long_coeffs, FZ_nom, FZ, SA, SR, IA):
        self.new_tire.pure_long_coeffs = [1.1756372529166366, 2.275693693135797, -0.6501009126133899, 17.15239682104082, -2.3290570234385215, -8.321786521216294, -5.708815286024726, 0.17545348128614113, 39.4346378819371, 33.92968608125336, -2.010829751105287, -0.0021163086632554535, -0.002049018916192311, -0.09099235392041341, -0.04812593772160824]
        self.new_tire.combined_long_coeffs = long_coeffs

        # self.new_tire.pure_long_coeffs = long_coeffs
        # self.new_tire.combined_long_coeffs = [27.53602516863092, 21.52884226211935, 1.0183656575102185, 0.500268433631573, 0.0, 0.0]

        if self.combined:
            FX = self.new_tire._combined_long(data = [FZ_nom, FZ, SA, SR, IA])
        else:
            FX = self.new_tire._pure_long(data = [FZ_nom, FZ, SR, IA])
        
        return FX
    
    def _aligning_tire_eval(self, pure_aligning_coeffs, R_nom, FZ_nom, FZ, SA, IA):
        # pure_aligning_coeffs = list(pure_aligning_coeffs) + [0 for x in range(9)]
        self.new_tire.pure_aligning_coeffs = pure_aligning_coeffs

        MZ = self.new_tire._pure_aligning(data = [R_nom, FZ_nom, FZ, SA, IA])

        return MZ
    
    def _lat_residual_calc(self, lat_coeffs):
        residuals = []
        
        self.count += 1
        for i in range(len(self.SA)):
            if not self.combined:
                current_step = self._lat_tire_eval(lat_coeffs, self.FZ_nom, self.FZ[i], self.SA[i], 0, self.IA[i])
            else:
                current_step = self._lat_tire_eval(lat_coeffs, self.FZ_nom, self.FZ[i], self.SA[i], self.SR[i], self.IA[i])

            residual = self.FY[i] - current_step

            if abs(lat_coeffs[2]) < 0.75:
                residuals.append(residual**2)
            else:

            # [7, 2.5, 0, 1, 0, 0, 0.02, 0, 0, 0, -0.2, 14, 1.9, 10]

            # multiplier = 3/4

            # if abs(lat_coeffs[0]) > 14 * multiplier:
            #     residuals.append(residual**3)
            # elif abs(lat_coeffs[1]) > 5 * multiplier:
            #     residuals.append(residual**2)
            # elif abs(lat_coeffs[3]) > 2 * multiplier:
            #     residuals.append(residual**2)
            # elif abs(lat_coeffs[6]) > 0.04 * multiplier:
            #     residuals.append(residual**2)
            # elif abs(lat_coeffs[10]) > 0.04 * multiplier:
            #     residuals.append(residual**2)
            # elif abs(lat_coeffs[11]) > 28 * multiplier:
            #     residuals.append(residual**2)
            # elif abs(lat_coeffs[12]) > 4 * multiplier:
            #     residuals.append(residual**2)
            # elif abs(lat_coeffs[13]) > 20 * multiplier:
            #     residuals.append(residual**2)
            # else:
            #     residuals.append(residual)

                residuals.append(residual)

        print(f"\rCurrent Residual Norm: {np.linalg.norm(residuals)}", end = '')
        print("\n" + str(list(lat_coeffs)))
        
        return np.linalg.norm(residuals)
    
    def _long_residual_calc(self, long_coeffs):
        long_coeffs[-1] = 0
        long_coeffs[-2] = 0

        residuals = []
        
        self.count += 1
        for i in range(len(self.SR)):
            current_step = self._long_tire_eval(long_coeffs, self.FZ_nom, self.FZ[i], self.SA[i], self.SR[i], self.IA[i])

            residual = self.FX[i] - current_step

            if (long_coeffs[0] > 10):
                residuals.append(residual**2)

            # if (long_coeffs[0] < 0):
            #     residuals.append(residual**2)
            # if (long_coeffs[1] < 0):
            #     residuals.append(residual**2)
            # if (abs(long_coeffs[2]) > 1.25):
            #     residuals.append(residual**2)
            # if (abs(long_coeffs[2]) < 0.65):
            #     residuals.append(residual**2)
            # if (long_coeffs[4] > 0):
            #     residuals.append(residual**2)
            # if (long_coeffs[8] < 0):
            #     residuals.append(residual**2)
            # if (long_coeffs[9] < 0):
            #     residuals.append(residual**2)
            # if (long_coeffs[10] > 0):
            #     residuals.append(residual**2)
            # if self.new_tire.shape <= 1:
            #     residuals.append(residual**2)



            # if (long_coeffs[0] < 0):
            #     residuals.append(residual**2)
            # if (long_coeffs[1] < 0):
            #     residuals.append(residual**2)
            # if (abs(long_coeffs[2]) > 2):
            #     residuals.append(residual**2)
            # if (long_coeffs[4] > 0):
            #     residuals.append(residual**2)
            # if (long_coeffs[8] < 0):
            #     residuals.append(residual**2)
            # if (long_coeffs[9] < 0):
            #     residuals.append(residual**2)
            # if (long_coeffs[10] > 0):
            #     residuals.append(residual**2)
            # if (long_coeffs[15] < 0):
            #     residuals.append(residual**2)
            # if (long_coeffs[0] < 0):
            #     residuals.append(residual**2)
            # if (long_coeffs[0] > 8):
            #     residuals.append(residual**3)
            # if (long_coeffs[1] < 0):
            #     residuals.append(residual**2)
            # if (abs(long_coeffs[2]) > 1.5):
            #     residuals.append(residual**3)
            # if abs(self.SA[i]) > (5 * np.pi / 180):
            #     residuals.append(residual**3)

            residuals.append(residual)

        # norm = np.linalg.norm(residuals)

        # max_sweep = np.linspace(0.01, 0.2, 50)

        # all_vals = [self._long_tire_eval(long_coeffs, self.FZ_nom, 2500, 0, x, 0) for x in max_sweep]

        # print(max(all_vals))

        # if max(all_vals) < max(self.FX):
        #     norm = norm**(1 + max(self.FX) - max(all_vals))

        print(f"\rCurrent Residual Norm: {np.format_float_scientific(np.linalg.norm(residuals))}", end = '')
        print("\nPure Long: " + str(list(long_coeffs[0:15])), end='')
        print("\nCombined Long: " + str(list(long_coeffs[15:])))
        
        return np.linalg.norm(residuals)
    
    def _aligning_residual_calc(self, aligning_coeffs):
        residuals = []

        self.count += 1
        for i in range(len(self.SA)):
            current_step = self._aligning_tire_eval(aligning_coeffs, self.R_nom, self.FZ_nom, self.FZ[i], self.SA[i], self.IA[i])
            zero_eval = self._aligning_tire_eval(aligning_coeffs, self.R_nom, self.FZ_nom, self.FZ[i], 0, self.IA[i])

            residual = self.MZ[i] - current_step

            if (self.SA[i] > 3.5 * np.pi / 180) and (self.SA[i] < 5 * np.pi / 180):
                residuals.append(residual**2)
            elif (self.SA[i] < -3.5 * np.pi / 180) and (self.SA[i] > -5 * np.pi / 180):
                residuals.append(residual**2)
            elif (self.SA[i] < 1 * np.pi / 180) or (self.SA[i] > -1 * np.pi / 180):
                residuals.append(residual**2)
            else:
                residuals.append(residual)
            
            residuals.append(zero_eval)

            # residuals.append(self.MZ[i] - current_step)

        print(f"\rCurrent Residual Norm: {np.linalg.norm(residuals)}", end = '')
        print("\n" + str(list(aligning_coeffs)))

        return np.linalg.norm(residuals)


    def lat_coeff_solve(self, file):
        data = pd.read_csv(file)

        velocity = 25 * 1.60934
        pressure = 12 * 6.89476

        data = data[(data["velocity"] == velocity) & (data["pressure"] == pressure)]

        self.FZ = list(data["FZ"] * -1)[::25]
        self.SA = list(data["SA"] * np.pi / 180)[::25]
        # self.SR = list(data["SL"])[::25]
        self.IA = list(data["IA"] * np.pi / 180)[::25]
        self.FY = list(data["FY"] * -1)[::25]

        bounds = []
        self.load = list(data["load"].unique())

        # print(self.load)

        for load in self.load:
            FY_slice = data[(data["load"] == load)]

            max_FY = max(FY_slice["FY"])
            min_FY = min(FY_slice["FY"])

            bounds.append([load, [min_FY, max_FY]])

        # Hacky solution to force desired behavior
        # for bound in bounds:
        #     for i in range(1000):
        #         self.FZ.append(abs(bound[0]))
        #         self.SA.append(np.pi / 2)
        #         self.IA.append(0)
        #         self.FY.append(bound[1][0] * -1 * 0.70)
                
        #         self.FZ.append(abs(bound[0]))
        #         self.SA.append(-np.pi / 2)
        #         self.IA.append(0)
        #         self.FY.append(bound[1][1] * -1 * 0.70)

        # for bound in bounds:
        #     for i in range(1000):
        #         self.FZ.append(abs(bound[0]))
        #         self.SA.append(np.pi / 4)
        #         self.IA.append(0)
        #         self.FY.append(bound[1][0] * -1 * 0.85)
                
        #         self.FZ.append(abs(bound[0]))
        #         self.SA.append(-np.pi / 4)
        #         self.IA.append(0)
        #         self.FY.append(bound[1][1] * -1 * 0.85)
            
        self.combined = False

        # initial_guess = [1.4, -3, 0, 0, 0, 0, 0, 0, 30, 3, 0, 0, 0, 0, 0, 0, 0, 0]

        initial_guess = [1.2109769382543158, -1.9551363824447472, 1.0065226532952785, 0.23993290908458545, -0.060682618982036904, 0.9390048140450117, -0.34565924439354756, 0.16996572682670436, 27.94120400348409, 1.4391249086573694, 1.5656179637223917, -0.01011461701642221, -0.00999935995953132, 0.0960451044989182, 0.06030558250027775, 0.19985062943732748, 0.15367751901909799, 0.014365709801755028]

        # initial_guess = [7, 2.5, 0, 1, 0, 0, 0.02, 0, 0, 0, -0.2, 14, 1.9, 10]

        # initial_guess = [6, 0.14792861134435276, -0.0637696924347148, 1, 0.5, 1, -0.002421973956076131, 2.642340289383334e-05, -0.495451805544889, 0.7335984398197478, 1.6852508522669503, -168.70354847084354, 0.016231498181433743, 8.827354963737646]

        lat_coeff_soln = basinhopping(self._lat_residual_calc, initial_guess, niter = 1)
        lat_coeffs = lat_coeff_soln.x
        lat_residuals = lat_coeff_soln.fun

        return [lat_coeffs, lat_residuals]

    def long_coeff_solve(self, file):
        data = pd.read_csv(file)

        velocity = 25 * 1.60934
        pressure = 12 * 6.89476

        # if self.combined:
        data = data[(data["pressure"] == pressure) & (data["velocity"] == velocity)]
        # else:
        # data = data[(data["pressure"] == pressure) & (data["velocity"] == velocity) & (data["slip"] == 0)]

        self.FZ = list(data["FZ"] * -1)[::50]
        self.SA = list(data["SA"] * np.pi / 180)[::50]
        self.SR = list(data["SL"])[::50]
        self.IA = list(data["IA"] * np.pi / 180)[::50]
        self.FX = list(data["FX"])[::50]

        bounds = []
        self.load = list(data["load"].unique())

        for load in self.load:
            FX_slice = data[(data["load"] == load)]

            max_FX = max(FX_slice["FX"])
            min_FX = min(FX_slice["FX"])

            if ((max_FX - min_FX) / 2 * 1.5 < (max_FX)) or ((max_FX - min_FX) / 2 * 1.5 < (abs(min_FX))):
                continue

            bounds.append([load, [min_FX, max_FX]])

        # Hacky solution to force desired behavior
        # for bound in [max(bounds)]:
        #     for i in range(int(len(data) * 0.10)):
        #         self.FZ.append(abs(bound[0]))
        #         self.SR.append(1)
        #         self.SA.append(0)
        #         self.IA.append(0)
        #         self.FX.append(bound[1][0] * -1 * 0.50)

        #         # self.FZ.append(abs(bound[0]) * 0.75)
        #         # self.SR.append(1)
        #         # self.IA.append(0)
        #         # self.FX.append(bound[1][0] * -1 * 0.70)
                
        #         self.FZ.append(abs(bound[0]))
        #         self.SR.append(-1)
        #         self.SA.append(0)
        #         self.IA.append(0)
        #         self.FX.append(bound[1][1] * -1 * 0.50)

        #         # self.FZ.append(abs(bound[0]) * 0.75)
        #         # self.SR.append(-1)
        #         # self.IA.append(0)
        #         # self.FX.append(bound[1][0] * -1 * 0.70)

        # for bound in bounds:
        #     for i in range(int(len(data) * 0.10)):
        #         self.FZ.append(abs(bound[0]))
        #         self.SR.append(0.5)
        #         self.SA.append(0)
        #         self.IA.append(0)
        #         self.FX.append(bound[1][0] * -1 * 0.70)

        #         # self.FZ.append(abs(bound[0]) * 0.75)
        #         # self.SR.append(0.5)
        #         # self.IA.append(0)
        #         # self.FX.append(bound[1][0] * -1 * 0.85)
                
        #         self.FZ.append(abs(bound[0]))
        #         self.SR.append(-0.5)
        #         self.SA.append(0)
        #         self.IA.append(0)
        #         self.FX.append(bound[1][1] * -1 * 0.70)

        #         # self.FZ.append(abs(bound[0]) * 0.75)
        #         # self.SR.append(-0.5)
        #         # self.IA.append(0)
        #         # self.FX.append(bound[1][0] * -1 * 0.85)

        # initial_guess = [1.65, 1, 0, 0, -0.5, 0, 0, 0, 12, 10, -0.6, 0, 0, 0, 0, 5, 8, 1, 0, 0, 0]
            
        # initial_guess = [1.2052836778254206, 2.328503810186114, -0.5019770586875938, 17.152139694123825, -2.341640260042653, -8.314793720865355, -5.71348330813258, 0.17225406270105056, 39.435944289434715, 33.9291431749275, -2.0199064386201413, 0.0029480466203860136, 0.0221540025409977, -0.08812333786880895, -0.04689109707478187]

        self.combined = True
        self.simplified = True

        # initial_guess = [1.8055116315454263, 2.363795999410438, -0.75, 17.37809643665393, -0.0753077097680414, -3.873260972758899, -5.612943760237467, -0.09369738837852282, 39.22542527659097, 33.031501029806144, -1.5184910009627752, 0.0014788425242747414, -0.01360085381334135, -0.10769786974113124, -0.10628554187682332]

        # final pure long coeffs: [1.2058713315938916, 2.3285399399543425, -0.5035691627682772, 17.152139694123825, -2.3415552327286657, -8.31460427680607, -5.713554778365471, 0.17257167243788674, 39.43580697603098, 33.92908935957883, -2.022397049178798, 0.009656587015086725, 0.0017082941366939487, -0.09099235392041341, -0.04812593772160824]

        # initial_guess = [1.1872303599654122, 2.3012367715221975, -0.66, 17.152277878054125, -2.3335013012301107, -8.319497125266876, -5.710203391539783, 0.1749942580158348, 39.43516806533589, 33.929448280496274, -2.0158208553866985, 0.001059725485965245, 0.003559989204133372, -0.09099235392041341, -0.04812593772160824]

        initial_guess = [6.701679824892427, 8.017951012667416, 0.99, -7.620470441623948, 0.0, 0.0]

        # initial_guess = [1.65, 2.25, 0, 0, -0.5, 0, 0, 0, 25, 10, -0.6, 0, 0, 0, 0]

        # if self.combined:
        #     pass
        # else:
        #     initial_guess = initial_guess[0:15]

        long_coeff_soln = basinhopping(self._long_residual_calc, initial_guess, niter = 1)
        long_coeffs = long_coeff_soln.x
        long_residuals = long_coeff_soln.fun

        return [long_coeffs, long_residuals]
    
    def aligning_coeff_solve(self, file):
        data = pd.read_csv(file)

        self.new_tire.pure_lat_coeffs = [1.462644363336092, -2.4525488894188525, 0.07875259157828247, 11.665598056159695, -0.0007085356603566561, -0.0003936037282383536, 78.65359240952695, -7842.704918479082, 56.50858901077719, 2.42841666180457, 0.6581244608544692, 0.00016212560860950413, -0.0010386186529656692, -0.07982161655436235, -0.0037501542948406765, 0.02152965494317117, -0.7891081372943167, -1.1113888934215281]

        velocity = 25 * 1.60934
        pressure = 12 * 6.89476
        camber = 0

        data = data[(data["velocity"] == velocity) & (data["pressure"] == pressure) & (data["SA"] < 10) & (data["SA"] > -10)]

        self.FZ = list(data["FZ"] * -1)
        self.SA = list(data["SA"] * np.pi / 180)
        self.IA = list(data["IA"] * np.pi / 180)
        self.MZ = list(data["MZ"] * -1)

        # initial_guess = [-8.919284675794422, -1.6314348004139725, -3.3767653589272517, -84.24744790087597, -83.67378462395052, 61442.25201423807, 3345.651148423848, 1.382380605977463, 0.16028108591631818]

        initial_guess = [-9.645786583079877, 5.650486243093343, 8.771555906212312, 115.61127816375134, -115.60898485597117, 61.08571530896941, 3.1449796678478394, 1.3270126298289837, 0.16384794241846673, -0.12769225216682936, -0.1420044172387813, -12.85054065245455, 0.012735955800908277, 0.002477164835327709, -3.422534867913855, 2.523749634198314, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        # initial_guess = initial_guess[:16]

        model_x_data = np.linspace(abs(max(self.FZ)), abs(min(self.FZ)), 1000)
        model_y_data = np.linspace(-10 * np.pi / 180, 10 * np.pi / 180, 1000)

        X, Y = np.meshgrid(model_x_data, model_y_data)

        Z = self._aligning_tire_eval(initial_guess, 8 * 0.0254, 1000, X, Y, 0)

        fig = plt.figure()
        ax = Axes3D(fig, auto_add_to_figure=False)

        ax = plt.axes(projection='3d')

        ax.plot_surface(X, Y, Z)
        ax.scatter3D(self.FZ, self.SA, self.MZ, cmap='Greens')

        fig.add_axes(ax)

        ax.set_xlabel('Normal Load (N)')
        ax.set_ylabel('Slip Angle (rad)')
        ax.set_zlabel('Aligning Moment (Nm)')

        plt.show()
        
        aligning_coeff_soln = basinhopping(self._aligning_residual_calc, initial_guess, niter = 10)
        aligning_coeffs = aligning_coeff_soln.x
        aligning_residuals = aligning_coeff_soln.fun

        return [aligning_coeffs, aligning_residuals]
    
    def _match_pure_lat_residual(self, scaling_coeffs):
        self.new_tire.scaling_coeffs_end[8:15] = scaling_coeffs
        self.new_tire.scaling_coeffs = self.new_tire.scaling_coeffs_start

        mesh = 100
        model_x_data = np.linspace(100, 3000, mesh)
        model_y_data = np.linspace(-np.pi / 2, np.pi / 2, mesh)
        model_z_data = np.linspace(min(self.IA), max(self.IA), mesh)

        X, Y, Z = np.meshgrid(model_x_data, model_y_data, model_z_data)

        pure_lat_fit_start = self._lat_tire_eval(self.new_tire.pure_lat_coeffs_start, self.FZ_nom, X, Y, Z)
        
        self.new_tire.scaling_coeffs = self.new_tire.scaling_coeffs_end
        pure_lat_fit_end = self._lat_tire_eval(self.new_tire.pure_lat_coeffs_end, self.FZ_nom, X, Y, Z)

        residuals = pure_lat_fit_end - pure_lat_fit_start

        print(f"\rCurrent Residual Norm: {np.linalg.norm(residuals)}", end = '')
        print("\n" + str(list(scaling_coeffs)))

        return np.linalg.norm(residuals)

    def match_pure_lat(self, coeffs1, coeffs2, scaling_coeffs1, scaling_coeffs2):
        self.new_tire.pure_lat_coeffs_start = coeffs1
        self.new_tire.pure_lat_coeffs_end = coeffs2
        self.new_tire.scaling_coeffs_start = scaling_coeffs1
        self.new_tire.scaling_coeffs_end = scaling_coeffs2

        initial_guess = self.new_tire.scaling_coeffs_end[8:15]

        lat_scaling_soln = basinhopping(self._match_pure_lat_residual, initial_guess, niter = 1)
        lat_scaling_coeffs = lat_scaling_soln.x
        lat_scaling_residuals = lat_scaling_soln.fun

        return [lat_scaling_coeffs, lat_scaling_residuals]

    def import_lat_data(self, file):
        data = pd.read_csv(file)

        velocity = 25 * 1.60934
        pressure = 12 * 6.89476
        camber = 0

        data = data[(data["velocity"] == velocity) & (data["pressure"] == pressure)]

        self.FZ = list(data["FZ"] * -1)
        self.SA = list(data["SA"] * np.pi / 180)
        self.IA = list(data["IA"] * np.pi / 180)
        self.MZ = list(data["MZ"] * -1)
    
    def temp2(self, aligning_coeffs):
        residuals = []

        self.count += 1
        for i in range(len(self.SA)):
            current_step = self._aligning_tire_eval(aligning_coeffs, self.R_nom, self.FZ_nom, self.FZ[i], self.SA[i], self.IA[i])
            # zero_eval = self._aligning_tire_eval(aligning_coeffs, self.R_nom, self.FZ_nom, self.FZ[i], 0, self.IA[i])

            residual = self.MZ[i] - current_step

            # if (self.SA[i] > 3.5 * np.pi / 180) and (self.SA[i] < 5 * np.pi / 180):
            #     residuals.append(residual**2)
            # elif (self.SA[i] < -3.5 * np.pi / 180) and (self.SA[i] > -5 * np.pi / 180):
            #     residuals.append(residual**2)
            # elif (self.SA[i] < 1 * np.pi / 180) or (self.SA[i] > -1 * np.pi / 180):
            #     residuals.append(residual**2)
            # else:
            residuals.append(residual)
            
            # residuals.append(zero_eval)

            # residuals.append(self.MZ[i] - current_step)

        return residuals