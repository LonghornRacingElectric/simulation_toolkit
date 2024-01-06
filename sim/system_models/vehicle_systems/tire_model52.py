import numpy as np

class TireModel:
    def __init__(self):
        self.pure_lat_coeffs: list[float] = []
        self.pure_long_coeffs: list[float] = []
        self.pure_aligning_coeffs: list[float] = []

        self.combined_lat_coeffs: list[float] = []
        self.combined_long_coeffs: list[float] = []
        self.combined_aligning_coeffs: list[float] = []
        self.combined_overturning_coeffs: list[float] = []
        self.combined_rolling_coeffs: list[float] = []
        
        self.scaling_coeffs: list[float] = []
        
        self.pure_lat_coeffs_start: list[float] = []
        self.pure_lat_coeffs_end: list[float] = []

        self.scaling_coeffs_start: list[float] = []
        self.scaling_coeffs_end: list[float] = []

        self.shape = 0

    def _pure_long(self, data: list[float]) -> float:
        FZ_nom = data[0]
        FZ = data[1]
        SR = data[2]
        IA = data[3]

        [CFX1, CFX2, CFX3, CFX4, CFX5, CFX6, CFX7, CFX8, CFX9, CFX10, CFX11, CFX12, CFX13, CFX14, CFX15] = self.pure_long_coeffs

        IA_x = IA * self.scaling_coeffs[7]
        df_z = (FZ - FZ_nom * self.scaling_coeffs[0]) / (FZ_nom * self.scaling_coeffs[0])
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
        FZ_nom = data[0]
        FZ = data[1]
        SA = data[2]
        IA = data[3]

        [CFY1, CFY2, CFY3, CFY4, CFY5, CFY6, CFY7, CFY8, CFY9, CFY10, \
         CFY11, CFY12, CFY13, CFY14, CFY15, CFY16, CFY17, CFY18] = self.pure_lat_coeffs
        
        IA_y = IA * self.scaling_coeffs[14]
        df_z = (FZ - FZ_nom * self.scaling_coeffs[0]) / (FZ_nom * self.scaling_coeffs[0])
        mu_y = (CFY2 + CFY3 * df_z) * (1 - CFY4 * IA_y**2) * self.scaling_coeffs[9]

        C_y = CFY1 * self.scaling_coeffs[8]
        D_y = mu_y * FZ
        K_y = CFY9 * FZ_nom * np.sin(2 * np.arctan(FZ / (CFY10 * FZ_nom * self.scaling_coeffs[0]))) * \
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
        R_nom = data[0]
        FZ_nom = data[1]
        FZ = data[2]
        SA = data[3]
        IA = data[4]

        # Lateral Dependencies
        [CFY1, CFY2, CFY3, CFY4, CFY5, CFY6, CFY7, CFY8, CFY9, CFY10, \
         CFY11, CFY12, CFY13, CFY14, CFY15, CFY16, CFY17, CFY18] = self.pure_lat_coeffs
        
        IA_y = IA * self.scaling_coeffs[14]
        df_z = (FZ - FZ_nom * self.scaling_coeffs[0]) / (FZ_nom * self.scaling_coeffs[0])
        mu_y = (CFY2 + CFY3 * df_z) * (1 - CFY4 * IA_y**2) * self.scaling_coeffs[9]

        C_y = CFY1 * self.scaling_coeffs[8]
        D_y = mu_y * FZ
        K_y = CFY9 * FZ_nom * np.sin(2 * np.arctan(FZ / (CFY10 * FZ_nom * self.scaling_coeffs[0]))) * \
            (1 - CFY11 * abs(IA_y)) * self.scaling_coeffs[0] * self.scaling_coeffs[11]
        B_y = K_y / (C_y * D_y)
        F_y0 = self._pure_lat([FZ_nom, FZ, SA, IA])

        B_y = K_y / (C_y * D_y)
        S_Hy = (CFY12 + CFY13 * df_z) * self.scaling_coeffs[12] + CFY14 * IA_y
        S_Vy = FZ * ((CFY15 + CFY16 * df_z) * self.scaling_coeffs[13] + (CFY17 + CFY18 * df_z) * IA_y) * self.scaling_coeffs[9]

        # Pure Aligning Moment
        [CMZ1, CMZ2, CMZ3, CMZ4, CMZ5, CMZ6, CMZ7, CMZ8, CMZ9, CMZ10, CMZ11, CMZ12, \
            CMZ13, CMZ14, CMZ15, CMZ16, CMZ17, CMZ18, CMZ19, CMZ20, CMZ21, CMZ22, CMZ23, CMZ24, CMZ25] = self.pure_aligning_coeffs
        
        IA_z = IA * self.scaling_coeffs[17]
        df_z = (FZ - FZ_nom * self.scaling_coeffs[0]) / (FZ_nom * self.scaling_coeffs[0])

        S_Ht = CMZ22 + CMZ23 * df_z + (CMZ24 + CMZ25 * df_z) * IA_z
        SA_t = SA + S_Ht

        D_r = FZ * ((CMZ13 + CMZ14 * df_z) * self.scaling_coeffs[16] + (CMZ15 + CMZ16 * df_z) * IA_z) * R_nom * self.scaling_coeffs[9]
        B_r = CMZ6 * self.scaling_coeffs[11] / self.scaling_coeffs[9] + CMZ7 * B_y * C_y

        D_t = FZ * (CMZ9 + CMZ10 * df_z) * (1 + CMZ11 * IA_z + CMZ12 * IA_z**2) * (R_nom / FZ_nom) * self.scaling_coeffs[15]
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
        FZ_nom = data[0]
        FZ = data[1]
        SA = data[2]
        SR = data[3]
        IA = data[4]

        [CCFX1, CCFX2, CCFX3, CCFX4, CCFX5, CCFX6] = self.combined_long_coeffs

        df_z = (FZ - FZ_nom * self.scaling_coeffs[0]) / (FZ_nom * self.scaling_coeffs[0])
        
        C_xSA = CCFX3
        B_xSA = CCFX1 * np.cos(np.arctan(CCFX2 * SR)) * self.scaling_coeffs[21]
        E_xSA = CCFX4 + CCFX5 * df_z
        S_HxSA = CCFX6

        SA_s = SA + S_HxSA

        # The Delft paper defines FX_adj this way, but we can decompose this into force output from the pure fit * some scaling coefficient
        # D_xSA = self._pure_long([FZ_nom, FZ, SR, IA]) / (np.cos(C_xSA * np.arctan(B_xSA * S_HxSA - E_xSA * (B_xSA * S_HxSA - np.arctan(B_xSA * S_HxSA)))))
        # FX_adj = D_xSA * np.cos(C_xSA * np.arctan(B_xSA * SA_s - E_xSA * (B_xSA * SA_s - np.arctan(B_xSA * SA_s))))

        # This is the same calculation, but the variables are shown a more intuitive way
        G_xSA = (np.cos(C_xSA * np.arctan(B_xSA * SA_s - E_xSA * (B_xSA * SA_s - np.arctan(B_xSA * SA_s))))) / (np.cos(C_xSA * np.arctan(B_xSA * S_HxSA - E_xSA * (B_xSA * S_HxSA - np.arctan(B_xSA * S_HxSA)))))
        FX_0 = self._pure_long([FZ_nom, FZ, SR, IA])

        FX_adj = FX_0 * G_xSA
        
        return FX_adj
    
    def _combined_lat(self, data: list[float]) -> float:
        FZ_nom = data[0]
        FZ = data[1]
        SA = data[2]
        SR = data[3]
        IA = data[4]

        [CCFY1, CCFY2, CCFY3, CCFY4, CCFY5, CCFY6, CCFY7, CCFY8, CCFY9, CCFY10, CCFY11, CCFY12, CCFY13, CCFY14] = self.combined_lat_coeffs
        [CFY1, CFY2, CFY3, CFY4, CFY5, CFY6, CFY7, CFY8, CFY9, CFY10, CFY11, CFY12, CFY13, CFY14, CFY15, CFY16, CFY17, CFY18] = self.pure_lat_coeffs
        
        df_z = (FZ - FZ_nom * self.scaling_coeffs[0]) / (FZ_nom * self.scaling_coeffs[0])
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
        # D_ySR = self.pure_lat_coeffs([FZ_nom, FZ, SA, IA]) / (np.cos(C_ySR * np.arctan(B_ySR * S_HySR - E_ySR * (B_ySR * S_HySR - np.arctan(B_ySR * S_HySR)))))
        # FY_adj = D_ySR * np.cos(C_ySR * np.arctan(B_ySR * SR_s - E_ySR * (B_ySR * SR_s - np.arctan(B_ySR * SR_s)))) + S_VySR

        # This is the same calculation, but the variables are shown a more intuitive way
        G_ySR = (np.cos(C_ySR * np.arctan(B_ySR * SR_s - E_ySR * (B_ySR * SR_s - np.arctan(B_ySR * SR_s))))) / (np.cos(C_ySR * np.arctan(B_ySR * S_HySR - E_ySR * (B_ySR * S_HySR - np.arctan(B_ySR * S_HySR)))))
        FY_0 = self._pure_lat([FZ_nom, FZ, SA, IA])

        FY_adj = FY_0 * G_ySR + S_VySR

        return FY_adj

    def _combined_aligning(self, data: list[float]) -> float:
        R_nom = data[0]
        FZ_nom = data[1]
        FZ = data[2]
        SA = data[3]
        SR = data[4]
        IA = data[5]

        [CCMZ1, CCMZ2, CCMZ3, CCMZ4] = self.combined_aligning_coeffs
        [CMZ1, CMZ2, CMZ3, CMZ4, CMZ5, CMZ6, CMZ7, CMZ8, CMZ9, CMZ10, CMZ11, CMZ12, CMZ13, CMZ14, CMZ15, CMZ16, CMZ17, CMZ18, CMZ19, CMZ20, CMZ21, CMZ22, CMZ23, CMZ24, CMZ25] = self.pure_aligning_coeffs
        [CCFY1, CCFY2, CCFY3, CCFY4, CCFY5, CCFY6, CCFY7, CCFY8, CCFY9, CCFY10, CCFY11, CCFY12, CCFY13, CCFY14] = self.combined_lat_coeffs
        [CFY1, CFY2, CFY3, CFY4, CFY5, CFY6, CFY7, CFY8, CFY9, CFY10, CFY11, CFY12, CFY13, CFY14, CFY15, CFY16, CFY17, CFY18] = self.pure_lat_coeffs
        [CFX1, CFX2, CFX3, CFX4, CFX5, CFX6, CFX7, CFX8, CFX9, CFX10, CFX11, CFX12, CFX13, CFX14, CFX15] = self.pure_long_coeffs

        df_z = (FZ - FZ_nom * self.scaling_coeffs[0]) / (FZ_nom * self.scaling_coeffs[0])
        IA_z = IA * self.scaling_coeffs[17]

        # Pure slip dependencies
        # Lateral
        IA_y = IA * self.scaling_coeffs[14]
        mu_y = (CFY2 + CFY3 * df_z) * (1 - CFY4 * IA_y**2) * self.scaling_coeffs[9]
        S_Hy = (CFY12 + CFY13 * df_z) * self.scaling_coeffs[12] + CFY14 * IA_y
        S_Vy = FZ * ((CFY15 + CFY16 * df_z) * self.scaling_coeffs[13] + (CFY17 + CFY18 * df_z) * IA_y) * self.scaling_coeffs[9]

        K_y = CFY9 * FZ_nom * np.sin(2 * np.arctan(FZ / (CFY10 * FZ_nom * self.scaling_coeffs[0]))) * \
            (1 - CFY11 * abs(IA_y)) * self.scaling_coeffs[0] * self.scaling_coeffs[11]
        C_y = CFY1 * self.scaling_coeffs[8]
        D_y = mu_y * FZ
        B_y = K_y / (C_y * D_y)

        # Longitudinal
        K_x = FZ * (CFX9 + CFX10 * df_z) * np.exp(CFX11 * df_z) * self.scaling_coeffs[4]
        
        # Aligning
        D_t = FZ * (CMZ9 + CMZ10 * df_z) * (1 + CMZ11 * IA_z + CMZ12 * IA_z**2) * (R_nom / FZ_nom) * self.scaling_coeffs[15]
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

        FX = self._combined_long([FZ_nom, FZ, SA, SR, IA])
        FY = self._combined_lat([FZ_nom, FZ, SA, SR, IA])

        F_y_IA_adj = FY - S_VySR

        D_r = FZ * ((CMZ13 + CMZ14 * df_z) * self.scaling_coeffs[16] + (CMZ15 + CMZ16 * df_z) * IA_z) * R_nom * self.scaling_coeffs[9]
        B_r = CMZ6 * self.scaling_coeffs[11] / self.scaling_coeffs[9] + CMZ7 * B_y * C_y

        M_zr = D_r * np.cos(np.arctan(B_r * SA_r_eq)) * np.cos(SA)

        s = (CCMZ1 + CCMZ2 * (FY / FZ_nom) + (CCMZ3 + CCMZ4 * df_z) * IA) * R_nom * self.scaling_coeffs[24]

        MZ_adj = -t_adj * F_y_IA_adj + M_zr + s * FX

        return MZ_adj
    
    def _combined_overturning(self, data: list[float]) -> float:
        R_nom = data[0]
        FZ_nom = data[1]
        FZ = data[2]
        SA = data[3]
        SR = data[4]
        IA = data[5]

        [CCMX1, CCMX2, CCMX3] = self.combined_overturning_coeffs

        FY = self._combined_lat([FZ_nom, FZ, SA, IA])

        MX = R_nom * FZ * (CCMX1 * self.scaling_coeffs[19] + (-CCMX2 * IA + CCMX3 * FY / FZ_nom) * self.scaling_coeffs[18])
        
        return MX
    
    def _combined_rolling(self, data: list[float]) -> float:
        R_nom = data[0]
        FZ_nom = data[1]
        FZ = data[2]
        SA = data[3]
        SR = data[4]
        IA = data[5]

        # TTC doesn't give FY data, so we'll make an estimate rather than using the calculations below
        # [CCMY1, CCMY2, CCMY3, CCMY4, VREF] = self.combined_rolling_coeffs
        # FX = self._combined_long([FZ_nom, FZ, SA, SR, IA])
        # MY = R_nom * FZ * (CCMY1 + CCMY2 * FX / FZ_nom + CCMY3 * np.abs(V_x / VREF) + CCMY4 * (V_x / VREF)**4)

        [CFX1, CFX2, CFX3, CFX4, CFX5, CFX6, CFX7, CFX8, CFX9, CFX10, CFX11, CFX12, CFX13, CFX14, CFX15] = self.pure_long_coeffs

        df_z = (FZ - FZ_nom * self.scaling_coeffs[0]) / (FZ_nom * self.scaling_coeffs[0])

        K_x = FZ * (CFX9 + CFX10 * df_z) * np.exp(CFX11 * df_z) * self.scaling_coeffs[4]

        S_Hx = (CFX12 + CFX13 * df_z) * self.scaling_coeffs[5]
        S_Vx = FZ * (CFX14 + CFX15 * df_z) * self.scaling_coeffs[6] * self.scaling_coeffs[2]

        MY = R_nom * (S_Vx + K_x * S_Hx)

        return MY