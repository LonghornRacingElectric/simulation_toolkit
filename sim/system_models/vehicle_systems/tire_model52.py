import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class TireModel:
    def __init__(self, 
                 R_nom: float = 8 * 0.0254, 
                 FZ_nom: float = 350 * 4.44822, 
                 scaling_coeffs: list[float] = [1 for x in range(28)], 
                 pure_lat_coeffs: list[float] = [1.4, -3, 0, 0, 0, 0, 0, 0, 30, 3, 0, 0, 0, 0, 0, 0, 0, 0], 
                 pure_long_coeffs: list[float] = [1.65, 1, 0, 0, -0.5, 0, 0, 0, 12, 10, -0.6, 0, 0, 0, 0, 5, 8, 1, 0, 0, 0], 
                 pure_aligning_coeffs: list[float] = [6, -4, 0.6, 0, 0, 0, 0.7, 1.05, 0.12, -0.03, 0, -1, 0, 0, 0.6, 0.2, -10, 0, 0, 0, 0, 0, 0, 0, 0], 
                 combined_lat_coeffs: list[float] = [7, 2.5, 0, 1, 0, 0, 0.02, 0, 0, 0, -0.2, 14, 1.9, 10], 
                 combined_long_coeffs: list[float] = [5, 8, 1, 0, 0, 0], 
                 combined_aligning_coeffs: list[float] = [0, -0.1, -1.0, 0], 
                 overturning_coeffs: list[float] = [0, 0, 0], 
                 rolling_coeffs: list[float] = [0, 0, 0, 0, 25]
                 ):
        
        # Constant Model Parameters
        self.R_nom = R_nom
        self.FZ_nom = FZ_nom

        # Scaling Coeffs
        self.scaling_coeffs = scaling_coeffs

        # Pure Slip Coeffs
        self.pure_lat_coeffs = pure_lat_coeffs
        self.pure_long_coeffs = pure_long_coeffs
        self.pure_aligning_coeffs = pure_aligning_coeffs

        # Combined Slip Coeffs
        self.combined_lat_coeffs = combined_lat_coeffs
        self.combined_long_coeffs = combined_long_coeffs
        self.combined_aligning_coeffs = combined_aligning_coeffs

        self.overturning_coeffs = overturning_coeffs
        self.rolling_coeffs = rolling_coeffs
        
        # self.pure_lat_coeffs_start: list[float] = []
        # self.pure_lat_coeffs_end: list[float] = []

        # self.scaling_coeffs_start: list[float] = []
        # self.scaling_coeffs_end: list[float] = []
    
    def eval(self, FZ, SA, SR, IA):
        FX = self._combined_lat([FZ, SA, SR, IA])
        FY = self._combined_long([FZ, SA, SR, IA])

        MX = self._combined_overturning([FZ, SA, SR, IA])
        MY = self._combined_rolling([FZ, SA, SR, IA])
        MZ = self._combined_aligning([FZ, SA, SR, IA])

        return [FX, FY, FZ, MX, MY, MZ]

    def _pure_long(self, data: list[float]) -> float:
        FZ, SR, IA = data

        [CFX1, CFX2, CFX3, CFX4, CFX5, CFX6, CFX7, CFX8, CFX9, CFX10, CFX11, CFX12, CFX13, CFX14, CFX15] = self.pure_long_coeffs

        IA_x = IA * self.scaling_coeffs[7]
        df_z = (FZ - self.FZ_nom * self.scaling_coeffs[0]) / (self.FZ_nom * self.scaling_coeffs[0])
        mu_x = (CFX2 + CFX3 * df_z) * (1 - CFX4 * IA_x**2) * self.scaling_coeffs[2]

        C_x = CFX1 * self.scaling_coeffs[1]
        D_x = mu_x * FZ
        K_x = FZ * (CFX9 + CFX10 * df_z) * np.exp(CFX11 * df_z) * self.scaling_coeffs[4]
        B_x = K_x / (C_x * D_x)

        S_Hx = (CFX12 + CFX13 * df_z) * self.scaling_coeffs[5]
        S_Vx = FZ * (CFX14 + CFX15 * df_z) * self.scaling_coeffs[6] * self.scaling_coeffs[2]
        SR_x = SR + S_Hx

        E_x = (CFX5 + CFX6 * df_z + CFX7 * df_z**2) * (1 - CFX8 * np.sign(SR_x)) * self.scaling_coeffs[3]
        
        self.shape = B_x * SR_x - E_x * (B_x * SR_x - np.arctan(B_x * SR_x))

        F_X0 = D_x * np.sin(C_x * np.arctan(self.shape)) + S_Vx
        F_X = F_X0
        
        return F_X

    def _pure_lat(self, data: list[float]) -> float:
        FZ, SA, IA = data

        [CFY1, CFY2, CFY3, CFY4, CFY5, CFY6, CFY7, CFY8, CFY9, CFY10, \
         CFY11, CFY12, CFY13, CFY14, CFY15, CFY16, CFY17, CFY18] = self.pure_lat_coeffs
        
        IA_y = IA * self.scaling_coeffs[14]
        df_z = (FZ - self.FZ_nom * self.scaling_coeffs[0]) / (self.FZ_nom * self.scaling_coeffs[0])
        mu_y = (CFY2 + CFY3 * df_z) * (1 - CFY4 * IA_y**2) * self.scaling_coeffs[9]

        C_y = CFY1 * self.scaling_coeffs[8]
        D_y = mu_y * FZ
        K_y = CFY9 * self.FZ_nom * np.sin(2 * np.arctan(FZ / (CFY10 * self.FZ_nom * self.scaling_coeffs[0]))) * \
            (1 - CFY11 * abs(IA_y)) * self.scaling_coeffs[0] * self.scaling_coeffs[11]
        B_y = K_y / (C_y * D_y)

        S_Hy = (CFY12 + CFY13 * df_z) * self.scaling_coeffs[12] + CFY14 * IA_y
        S_Vy = FZ * ((CFY15 + CFY16 * df_z) * self.scaling_coeffs[13] + (CFY17 + CFY18 * df_z) * IA_y) * self.scaling_coeffs[9]
        SA_y = SA + S_Hy

        E_y = (CFY5 + CFY6 * df_z) * (1 - (CFY7 + CFY8 * IA_y) * np.sign(SA_y)) * self.scaling_coeffs[10]

        F_Y0 = D_y * np.sin(C_y * np.arctan(B_y * SA_y - E_y * (B_y * SA_y - np.arctan(B_y * SA_y)))) + S_Vy
        F_Y = F_Y0

        return F_Y
    
    def _pure_aligning(self, data: list[float]) -> float:
        FZ, SA, IA = data

        # Lateral Dependencies
        [CFY1, CFY2, CFY3, CFY4, CFY5, CFY6, CFY7, CFY8, CFY9, CFY10, \
         CFY11, CFY12, CFY13, CFY14, CFY15, CFY16, CFY17, CFY18] = self.pure_lat_coeffs
        
        IA_y = IA * self.scaling_coeffs[14]
        df_z = (FZ - self.FZ_nom * self.scaling_coeffs[0]) / (self.FZ_nom * self.scaling_coeffs[0])
        mu_y = (CFY2 + CFY3 * df_z) * (1 - CFY4 * IA_y**2) * self.scaling_coeffs[9]

        C_y = CFY1 * self.scaling_coeffs[8]
        D_y = mu_y * FZ
        K_y = CFY9 * self.FZ_nom * np.sin(2 * np.arctan(FZ / (CFY10 * self.FZ_nom * self.scaling_coeffs[0]))) * \
            (1 - CFY11 * abs(IA_y)) * self.scaling_coeffs[0] * self.scaling_coeffs[11]
        B_y = K_y / (C_y * D_y)
        F_y0 = self._pure_lat([FZ, SA, IA])

        B_y = K_y / (C_y * D_y)
        S_Hy = (CFY12 + CFY13 * df_z) * self.scaling_coeffs[12] + CFY14 * IA_y
        S_Vy = FZ * ((CFY15 + CFY16 * df_z) * self.scaling_coeffs[13] + (CFY17 + CFY18 * df_z) * IA_y) * self.scaling_coeffs[9]

        # Pure Aligning Moment
        [CMZ1, CMZ2, CMZ3, CMZ4, CMZ5, CMZ6, CMZ7, CMZ8, CMZ9, CMZ10, CMZ11, CMZ12, \
            CMZ13, CMZ14, CMZ15, CMZ16, CMZ17, CMZ18, CMZ19, CMZ20, CMZ21, CMZ22, CMZ23, CMZ24, CMZ25] = self.pure_aligning_coeffs
        
        IA_z = IA * self.scaling_coeffs[17]
        df_z = (FZ - self.FZ_nom * self.scaling_coeffs[0]) / (self.FZ_nom * self.scaling_coeffs[0])

        S_Ht = CMZ22 + CMZ23 * df_z + (CMZ24 + CMZ25 * df_z) * IA_z
        SA_t = SA + S_Ht

        D_r = FZ * ((CMZ13 + CMZ14 * df_z) * self.scaling_coeffs[16] + (CMZ15 + CMZ16 * df_z) * IA_z) * self.R_nom * self.scaling_coeffs[9]
        B_r = CMZ6 * self.scaling_coeffs[11] / self.scaling_coeffs[9] + CMZ7 * B_y * C_y

        D_t = FZ * (CMZ9 + CMZ10 * df_z) * (1 + CMZ11 * IA_z + CMZ12 * IA_z**2) * (self.R_nom / self.FZ_nom) * self.scaling_coeffs[15]
        C_t = CMZ8
        B_t = (CMZ1 + CMZ2 * df_z + CMZ3 * df_z**2) * (1 + CMZ4 * IA_z + CMZ5 * abs(IA_z)) * self.scaling_coeffs[11] / self.scaling_coeffs[9]

        E_t = (CMZ17 + CMZ18 * df_z + CMZ19 * df_z**2) * (1 + (CMZ20 + CMZ21 * IA_z) * (2 / np.pi) * np.arctan(B_t * C_t * SA_t))

        S_Hf = S_Hy + S_Vy / K_y
        
        # Residual Torque

        # Found typo in the delft paper. Hr should be Hf
        SA_r = SA + S_Hf
        M_zr = D_r * np.cos(np.arctan(B_r * SA_r)) * np.cos(SA)

        # Pneumatic trail
        t = D_t * np.cos(C_t * np.arctan(B_t * SA_t - E_t * (B_t * SA_t - np.arctan(B_t * SA_t)))) * np.cos(SA)

        M_Z0 = -t * F_y0 + M_zr
        M_Z = M_Z0

        return M_Z
    
    def _combined_long(self, data: list[float]) -> float:
        FZ, SA, SR, IA = data

        [CCFX1, CCFX2, CCFX3, CCFX4, CCFX5, CCFX6] = self.combined_long_coeffs

        df_z = (FZ - self.FZ_nom * self.scaling_coeffs[0]) / (self.FZ_nom * self.scaling_coeffs[0])
        
        C_xSA = CCFX3
        B_xSA = CCFX1 * np.cos(np.arctan(CCFX2 * SR)) * self.scaling_coeffs[21]
        E_xSA = CCFX4 + CCFX5 * df_z
        S_HxSA = CCFX6

        SA_s = SA + S_HxSA

        # The Delft paper defines FX_adj this way, but we can decompose this into force output from the pure fit * some scaling coefficient
        # D_xSA = self._pure_long([FZ, SR, IA]) / (np.cos(C_xSA * np.arctan(B_xSA * S_HxSA - E_xSA * (B_xSA * S_HxSA - np.arctan(B_xSA * S_HxSA)))))
        # FX_adj = D_xSA * np.cos(C_xSA * np.arctan(B_xSA * SA_s - E_xSA * (B_xSA * SA_s - np.arctan(B_xSA * SA_s))))

        # This is the same calculation, but the variables are shown a more intuitive way
        G_xSA = (np.cos(C_xSA * np.arctan(B_xSA * SA_s - E_xSA * (B_xSA * SA_s - np.arctan(B_xSA * SA_s))))) / (np.cos(C_xSA * np.arctan(B_xSA * S_HxSA - E_xSA * (B_xSA * S_HxSA - np.arctan(B_xSA * S_HxSA)))))
        FX_0 = self._pure_long([FZ, SR, IA])

        FX_adj = FX_0 * G_xSA
        
        return FX_adj
    
    def _combined_lat(self, data: list[float]) -> float:
        FZ, SA, SR, IA = data

        [CCFY1, CCFY2, CCFY3, CCFY4, CCFY5, CCFY6, CCFY7, CCFY8, CCFY9, CCFY10, CCFY11, CCFY12, CCFY13, CCFY14] = self.combined_lat_coeffs
        [CFY1, CFY2, CFY3, CFY4, CFY5, CFY6, CFY7, CFY8, CFY9, CFY10, CFY11, CFY12, CFY13, CFY14, CFY15, CFY16, CFY17, CFY18] = self.pure_lat_coeffs
        
        df_z = (FZ - self.FZ_nom * self.scaling_coeffs[0]) / (self.FZ_nom * self.scaling_coeffs[0])
        IA_y = IA * self.scaling_coeffs[14]
        mu_y = (CFY2 + CFY3 * df_z) * (1 - CFY4 * IA_y**2) * self.scaling_coeffs[9]

        C_ySR = CCFY4
        B_ySR = CCFY1 * np.cos(np.arctan(CCFY2 * (SA - CCFY3))) * self.scaling_coeffs[22]
        E_ySR = CCFY5 + CCFY6 * df_z
        S_HySR = CCFY7 + CCFY8 * df_z

        D_VySR = mu_y * FZ * (CCFY9 + CCFY10 * df_z + CCFY11 * IA) * np.cos(np.arctan(CCFY12 * SA))

        S_VySR = D_VySR * np.sin(CCFY13 * np.arctan(CCFY14 * SR)) * self.scaling_coeffs[23]

        SR_s = SR + S_HySR
        
        # The Delft paper defines FY_adj this way, but we can decompose this into force output from the pure fit * some scaling coefficient
        # D_ySR = self.pure_lat_coeffs([self.FZ_nom, FZ, SA, IA]) / (np.cos(C_ySR * np.arctan(B_ySR * S_HySR - E_ySR * (B_ySR * S_HySR - np.arctan(B_ySR * S_HySR)))))
        # FY_adj = D_ySR * np.cos(C_ySR * np.arctan(B_ySR * SR_s - E_ySR * (B_ySR * SR_s - np.arctan(B_ySR * SR_s)))) + S_VySR

        # This is the same calculation, but the variables are shown a more intuitive way
        G_ySR = (np.cos(C_ySR * np.arctan(B_ySR * SR_s - E_ySR * (B_ySR * SR_s - np.arctan(B_ySR * SR_s))))) / (np.cos(C_ySR * np.arctan(B_ySR * S_HySR - E_ySR * (B_ySR * S_HySR - np.arctan(B_ySR * S_HySR)))))
        FY_0 = self._pure_lat([FZ, SA, IA])

        FY_adj = FY_0 * G_ySR + S_VySR

        return FY_adj

    def _combined_aligning(self, data: list[float]) -> float:
        FZ, SA, SR, IA = data

        CCMZ1, CCMZ2, CCMZ3, CCMZ4 = self.combined_aligning_coeffs
        CMZ1, CMZ2, CMZ3, CMZ4, CMZ5, CMZ6, CMZ7, CMZ8, CMZ9, CMZ10, CMZ11, CMZ12, CMZ13, CMZ14, CMZ15, CMZ16, CMZ17, CMZ18, CMZ19, CMZ20, CMZ21, CMZ22, CMZ23, CMZ24, CMZ25 = self.pure_aligning_coeffs
        CCFY1, CCFY2, CCFY3, CCFY4, CCFY5, CCFY6, CCFY7, CCFY8, CCFY9, CCFY10, CCFY11, CCFY12, CCFY13, CCFY14 = self.combined_lat_coeffs
        CFY1, CFY2, CFY3, CFY4, CFY5, CFY6, CFY7, CFY8, CFY9, CFY10, CFY11, CFY12, CFY13, CFY14, CFY15, CFY16, CFY17, CFY18 = self.pure_lat_coeffs
        CFX1, CFX2, CFX3, CFX4, CFX5, CFX6, CFX7, CFX8, CFX9, CFX10, CFX11, CFX12, CFX13, CFX14, CFX15 = self.pure_long_coeffs

        df_z = (FZ - self.FZ_nom * self.scaling_coeffs[0]) / (self.FZ_nom * self.scaling_coeffs[0])
        IA_z = IA * self.scaling_coeffs[17]

        # Pure slip dependencies
        # Lateral
        IA_y = IA * self.scaling_coeffs[14]
        mu_y = (CFY2 + CFY3 * df_z) * (1 - CFY4 * IA_y**2) * self.scaling_coeffs[9]
        S_Hy = (CFY12 + CFY13 * df_z) * self.scaling_coeffs[12] + CFY14 * IA_y
        S_Vy = FZ * ((CFY15 + CFY16 * df_z) * self.scaling_coeffs[13] + (CFY17 + CFY18 * df_z) * IA_y) * self.scaling_coeffs[9]

        K_y = CFY9 * self.FZ_nom * np.sin(2 * np.arctan(FZ / (CFY10 * self.FZ_nom * self.scaling_coeffs[0]))) * \
            (1 - CFY11 * abs(IA_y)) * self.scaling_coeffs[0] * self.scaling_coeffs[11]
        C_y = CFY1 * self.scaling_coeffs[8]
        D_y = mu_y * FZ
        B_y = K_y / (C_y * D_y)

        # Longitudinal
        K_x = FZ * (CFX9 + CFX10 * df_z) * np.exp(CFX11 * df_z) * self.scaling_coeffs[4]
        
        # Aligning
        D_t = FZ * (CMZ9 + CMZ10 * df_z) * (1 + CMZ11 * IA_z + CMZ12 * IA_z**2) * (self.R_nom / self.FZ_nom) * self.scaling_coeffs[15]
        C_t = CMZ8
        B_t = (CMZ1 + CMZ2 * df_z + CMZ3 * df_z**2) * (1 + CMZ4 * IA_z + CMZ5 * abs(IA_z)) * self.scaling_coeffs[11] / self.scaling_coeffs[9]

        E_t = (CMZ17 + CMZ18 * df_z + CMZ19 * df_z**2) * (1 + (CMZ20 + CMZ21 * IA_z) * (2 / np.pi) * np.arctan(B_t * C_t * SA_t))
        
        D_VySR = mu_y * FZ * (CCFY9 + CCFY10 * df_z + CCFY11 * IA) * np.cos(np.arctan(CCFY12 * SA))
        S_VySR = D_VySR * np.sin(CCFY13 * np.arctan(CCFY14 * SR)) * self.scaling_coeffs[23]

        S_Ht = CMZ22 + CMZ23 * df_z + (CMZ24 + CMZ25 * df_z) * IA_z
        SA_t = SA + S_Ht

        S_Hf = S_Hy + S_Vy / K_y
        SA_r = SA + S_Hf

        # Adjusted SA values
        SA_t_eq = np.arctan(np.sqrt((np.tan(SA_t))**2 + (K_x / K_y)**2 * SR**2)) * np.sign(SA_t)
        SA_r_eq = np.arctan(np.sqrt((np.tan(SA_r))**2 + (K_x / K_y)**2 * SR**2)) * np.sign(SA_r)

        # Pneumatic trail
        t_adj = D_t * np.cos(C_t * np.arctan(B_t * SA_t_eq - E_t * (B_t * SA_t_eq - np.arctan(B_t * SA_t_eq)))) * np.cos(SA)

        FX = self._combined_long([FZ, SA, SR, IA])
        FY = self._combined_lat([FZ, SA, SR, IA])

        F_y_IA_adj = FY - S_VySR

        D_r = FZ * ((CMZ13 + CMZ14 * df_z) * self.scaling_coeffs[16] + (CMZ15 + CMZ16 * df_z) * IA_z) * self.R_nom * self.scaling_coeffs[9]
        B_r = CMZ6 * self.scaling_coeffs[11] / self.scaling_coeffs[9] + CMZ7 * B_y * C_y

        M_zr = D_r * np.cos(np.arctan(B_r * SA_r_eq)) * np.cos(SA)

        s = (CCMZ1 + CCMZ2 * (FY / self.FZ_nom) + (CCMZ3 + CCMZ4 * df_z) * IA) * self.R_nom * self.scaling_coeffs[24]

        MZ_adj = -t_adj * F_y_IA_adj + M_zr + s * FX

        return MZ_adj
    
    def _combined_overturning(self, data: list[float]) -> float:
        FZ, SA, SR, IA = data

        [CCMX1, CCMX2, CCMX3] = self.overturning_coeffs

        FY = self._combined_lat([FZ, SA, SR, IA])

        MX = self.R_nom * FZ * (CCMX1 * self.scaling_coeffs[19] + (-CCMX2 * IA + CCMX3 * FY / self.FZ_nom) * self.scaling_coeffs[18])
        
        return MX
    
    def _combined_rolling(self, data: list[float]) -> float:
        FZ, SA, SR, IA = data

        # TTC doesn't give FY data, so we'll make an estimate rather than using the calculations below
        # [CCMY1, CCMY2, CCMY3, CCMY4, VREF] = self.combined_rolling_coeffs
        # FX = self._combined_long([FZ, SA, SR, IA])
        # MY = self.R_nom * FZ * (CCMY1 + CCMY2 * FX / self.FZ_nom + CCMY3 * np.abs(V_x / VREF) + CCMY4 * (V_x / VREF)**4)

        [CFX1, CFX2, CFX3, CFX4, CFX5, CFX6, CFX7, CFX8, CFX9, CFX10, CFX11, CFX12, CFX13, CFX14, CFX15] = self.pure_long_coeffs

        df_z = (FZ - self.FZ_nom * self.scaling_coeffs[0]) / (self.FZ_nom * self.scaling_coeffs[0])

        K_x = FZ * (CFX9 + CFX10 * df_z) * np.exp(CFX11 * df_z) * self.scaling_coeffs[4]

        S_Hx = (CFX12 + CFX13 * df_z) * self.scaling_coeffs[5]
        S_Vx = FZ * (CFX14 + CFX15 * df_z) * self.scaling_coeffs[6] * self.scaling_coeffs[2]

        MY = self.R_nom * (S_Vx + K_x * S_Hx)

        return MY
    
    def plot(self, 
                 plot_type: str = None,
                 scatter: str = None,
                 velocity: float = 25 * 1.60934,
                 pressure: float = 12 * 6.89476,
                 FZ: float = None,
                 SA: float = None,
                 SR: float = None,
                 IA: float = 0
                 ):

        plot_types = ["pure_lat", "pure_long", "pure_aligning", "combined_lat", "combined_long", "combined_aligning", "overturning", "rolling"]

        fig = plt.figure()
        ax = Axes3D(fig, auto_add_to_figure=False)
        ax = plt.axes(projection='3d')
        
        if scatter:
            data = pd.read_csv(scatter)

            all_conditions = [velocity, pressure, FZ, SA, SR, IA]
            all_categories = ["velocity", "pressure", "FZ", "SA", "SL", "IA"]

            for i in range(len(all_conditions)):
                if all_conditions[i]:
                    data = data[(data[all_categories[i]] == all_conditions[i])]

            FZ = list(data["FZ"] * -1)[::50]
            SA = list(data["SA"] * np.pi / 180)[::50]

            try:
                SR = list(data["SL"])[::50]
            except:
                pass

            IA = list(data["IA"] * np.pi / 180)[::50]
            FX = list(data["FX"])[::50]
            FY = list(data["FY"] * -1)[::50]
            MZ = list(data["MZ"])[::50]
            MX = list(data["MX"])[::50]
        

        if plot_type == plot_types[0]:
            if scatter:
                ax.scatter3D(FZ, SA, FY, cmap='Greens', s = 10)
                fig.add_axes(ax)

                ax.set_xlabel('Normal Load (N)')
                ax.set_ylabel('Slip Angle (rad)')
                ax.set_zlabel('Lateral Force (N)')
            
            model_FZ_data = np.linspace(0, 1200, 1000)
            model_SA_data = np.linspace(-20 * np.pi / 180, 20 * np.pi / 180, 1000)

            X, Y = np.meshgrid(model_FZ_data, model_SA_data)

            Z = self._pure_lat([X, Y, 0])

            ax.plot_surface(X, Y, Z)

            plt.show()

            return "Displaying pure lateral force plot"
        
        elif plot_type == plot_types[1]:
            if scatter:
                ax.scatter3D(FZ, SA, FY, cmap='Greens', s = 10)
                fig.add_axes(ax)

                ax.set_xlabel('Normal Load (N)')
                ax.set_ylabel('Slip Ratio')
                ax.set_zlabel('Longitudinal Force (N)')
            
            model_FZ_data = np.linspace(0, 3000, 1000)
            model_SR_data = np.linspace(-1, 1, 1000)

            X, Y = np.meshgrid(model_FZ_data, model_SR_data)

            Z = self._pure_long([X, Y, 0])

            ax.plot_surface(X, Y, Z)

            plt.show()

            return "Displaying pure longitudinal force plot"
        
        elif plot_type == plot_types[2]:
            if scatter:
                ax.scatter3D(FZ, SA, FY, cmap='Greens', s = 10)
                fig.add_axes(ax)

                ax.set_xlabel('Normal Load (N)')
                ax.set_ylabel('Slip Angle (rad)')
                ax.set_zlabel('Aligning Moment (Nm)')
            
            model_FZ_data = np.linspace(0, 3000, 1000)
            model_SA_data = np.linspace(-90 * np.pi / 180, 90 * np.pi / 180, 1000)

            X, Y = np.meshgrid(model_FZ_data, model_SA_data)

            Z = self._pure_aligning([X, Y, 0])

            ax.plot_surface(X, Y, Z)

            plt.show()

            return "Displaying pure aligning moment plot"
        
        elif plot_type == plot_types[3]:
            if scatter:
                ax.scatter3D(FZ, SA, FY, cmap='Greens', s = 10)
                fig.add_axes(ax)

                ax.set_xlabel('Normal Load (N)')
                ax.set_ylabel('Slip Angle (rad)')
                ax.set_zlabel('Aligning Moment (Nm)')
            
            model_FZ_data = np.linspace(0, 3000, 1000)
            model_SA_data = np.linspace(-90 * np.pi / 180, 90 * np.pi / 180, 1000)

            X, Y = np.meshgrid(model_FZ_data, model_SA_data)

            Z = self._pure_long([X, Y, 0])

            ax.plot_surface(X, Y, Z)

            plt.show()

            return "Displaying combined lateral force plot"
        
        elif plot_type == plot_types[4]:
            plt.show()
            pass
        elif plot_type == plot_types[5]:
            plt.show()
            pass
        elif plot_type == plot_types[6]:
            plt.show()
            pass
        elif plot_type == plot_types[7]:
            plt.show()
            pass
        else:
            return
        f"""Plot type not recognized. Please call one of the following: \n\n
            1. {plot_types[0]} \n
            2. {plot_types[1]} \n
            3. {plot_types[2]} \n
            4. {plot_types[3]} \n
            5. {plot_types[4]} \n
            6. {plot_types[5]} \n
            7. {plot_types[6]} \n
            8. {plot_types[7]} \n"""