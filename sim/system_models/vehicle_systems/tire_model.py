import numpy
import numpy as np

class TireModel:
    def __init__(self):
        self.lat_coeffs: list[float] = []
        self.long_coeffs: list[float] = []
        self.tire_scaling = 0.55

    def get_comstock_forces(self, SA: float, SR: float, FZ: float, IA: float, max_force: float = None):
        SA, SR = self._zero_protection(SA, SR)

        # if SR > 0.15:
        #     SR = max(0.15 - (SR - 0.15)/80, 1e-64)

        FX = self._long_pacejka([FZ, SR])
        FY = self._lat_pacejka([FZ, SA, IA])

        if max_force is not None:
            FX = max(min(FX, max_force), 1e-60)

        Ca = (self._long_pacejka([FZ, 1 / 100]) - self._long_pacejka([FZ, 0])) * (180 / np.pi) # slip stiffness
        Cs = (self._lat_pacejka([FZ, 1 * np.pi / 180, IA]) - self._lat_pacejka([FZ, 0, IA])) * 100 # cornering stiffness

        adj_FX = self._com_long(SA, SR, FX, FY, Ca)
        adj_FY = self._com_lat(SA, SR, FX, FY, Cs)

        return [adj_FX, adj_FY, FZ]
    
    def _com_long(self, SA: float, SR: float, FX: float, FY: float, Ca: float) -> float:
        FY = abs(FY)
        try:
            if (SR**2 * FY**2 + FX**2 * (np.tan(SA))**2 == 0 or Ca == 0):
                SA += 1e-64
                SR += 1e-64
                FX += 1e-64
                FY += 1e-64
                Ca += 1e-64
        except:
            pass

        adjusted_FX = ((FX * FY) / np.sqrt(SR**2 * FY**2 + FX**2 * (np.tan(SA))**2)) * \
                (np.sqrt(SR**2 * Ca**2 + (1 - SR)**2 * (np.cos(SA))**2 * FX**2) / Ca)
        
        return adjusted_FX
    
    def _com_lat(self, SA: float, SR: float, FX: float, FY: float, Cs: float) -> float:
        FX = abs(FX)
        try:
            if (SR**2 * FY**2 + FX**2 * (np.tan(SA))**2 == 0 or Cs == 0):
                SA += 1e-64
                SR += 1e-64
                FX += 1e-64
                FY += 1e-64
                Cs += 1e-64
        except:
            pass

        adjusted_FY = ((FX * FY) / np.sqrt(SR**2 * FY**2 + FX**2 * (np.tan(SA))**2)) * \
            (np.sqrt((1 - SR)**2 * (np.cos(SA))**2 * FY**2 + (np.sin(SA))**2 * Cs**2) / (Cs * np.cos(SA)))
            
        return adjusted_FY
    
    def _long_pacejka(self, data: list[float]) -> float:
        FZ = data[0] / 1000
        SR = data[1] * 100
        [b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12, b13] = self.long_coeffs

        if FZ <= 0:
            return 0
        else:
            C = b0
            D = FZ * (b1 * FZ + b2)
            
            BCD = (b3 * FZ**2 + b4 * FZ) * np.exp(-1 * b5 * FZ)
            B = BCD / (C * D)
            H = b9 * FZ + b10

            E = (b6 * FZ**2 + b7 * FZ + b8) * (1 - b13 * np.sign(SR + H))

            V = b11 * FZ + b12
            Bx1 = B * (SR + H)

            return self.tire_scaling * (D * np.sin(C * np.arctan(Bx1 - E * (Bx1 - np.arctan(Bx1)))) + V)

    def _lat_pacejka(self, data: list[float]) -> float:
        FZ = data[0]
        SA = data[1] * 180 / np.pi
        IA = data[2] * 180 / np.pi
        [a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17] = self.lat_coeffs

        if FZ <= 0:
            return 0
        else:
            C = a0
            D = FZ * (a1 * FZ + a2) * (1 - a15 * IA**2)
            
            BCD = a3 * np.sin(np.arctan(FZ / a4) * 2) * (1 - a5 * abs(IA))
            B = BCD / (C * D)
            H = a8 * FZ + a9 + a10 * IA

            E = (a6 * FZ + a7) * (1 - (a16 * IA + a17) * np.sign(SA + H))

            V = a11 * FZ + a12 + (a13 * FZ + a14) * IA * FZ
            Bx1 = B * (SA + H)

            return self.tire_scaling * (D * np.sin(C * np.arctan(Bx1 - E * (Bx1 - np.arctan(Bx1)))) + V)
    
    def _zero_protection(self, SA: float, SR: float):
        if type(SA) == float or type(SA) == int:
            if SA == 0:
                SA += 1e-64
        else:
            if SA.any() == 0:
                SA += 1e-64
            
        if type(SR) == float or type(SR) == int:
            if SR == 0:
                SR += 1e-64
        else:
            if SR.any() == 0:
                SR += 1e-64

        return [SA, SR]