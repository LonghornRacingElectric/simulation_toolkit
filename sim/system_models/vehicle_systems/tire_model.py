from sim.system_models.vehicle_systems.vehicle_system_model import VehicleSystemModel

from sim.model_parameters.cars.car import Car
from sim.system_models.vectors.controls_vector import ControlsVector
from sim.system_models.vectors.observables_vector import ObservablesVector
from sim.system_models.vectors.state_vector import StateVector
from sim.system_models.vectors.state_dot_vector import StateDotVector

import math
import numpy as np

# tire_force * tire_radius = braking_torque + motor torque


class TireModel(VehicleSystemModel):
    def __init__(self):
        super().__init__()

        self.controls_in = [

        ]

        self.state_in = [
            "slip_angle",
            "slip_ratio",
            "normal_force",
            "inclination_angle"
        ]

        self.state_out = [
            "tire_FX",
            "tire_FY",
            "tire_FZ"

            # Will implement all moment calculations at some point. I believe the MF 5.2 paper covers this

            # "tire_MX",
            # "tire_MY",
            # "tire_MZ"
        ]

        self.observables_out = [

        ]

    def eval(self, vehicle_parameters: Car, controls_vector: ControlsVector, state_in_vector: StateVector,
             state_out_vector: StateDotVector, observables_out_vector: ObservablesVector):
        pass

    # Redo cornering stiffness implementation
    def _get_comstock_forces(self, SR, SA, FZ, IA):
        if FZ <= 0.0:
            return np.array([0, 0, 0])
        else:
            FX = self._long_pacejka(FZ, SR)
            FY = self._lat_pacejka(IA, FZ, SA)

            Ca = (self._long_pacejka(FZ, 0.005) - self._long_pacejka(FZ, -0.005)) / (.01)
            Cs = (self._lat_pacejka(IA, FZ, 0.005) - self._lat_pacejka(IA, FZ, -0.005)) / (.01)
            FY_adj = self.com_lat(SA, SR, FX, FY, IA, FZ, Cs) 
            FX_adj = self.com_long(SA, SR, FX, FY, IA, FZ, Ca)
            
        if SA > 0:
            FY_adj = FY_adj * -1

        return np.array([FX_adj, FY_adj, FZ])
    
    # Fits done in a messy way right now. Fix this later.
    def _lat_pacejka(self, inclination_angle:float, normal_force:float, slip_angle:float, tire_scaling: float, 
                         lateral_coeffs: list[float]):
        
        if normal_force == 0:
            return 0
        
        slip_degrees = slip_angle * 180 / math.pi # degrees
        inclination_degrees = inclination_angle * 180 / math.pi # degrees
        
        [a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17] = lateral_coeffs
        C = a0
        D = normal_force * (a1 * normal_force + a2) * (1 - a15 * inclination_degrees ** 2)

        BCD = a3 * math.sin(math.atan(normal_force / a4) * 2) * (1 - a5 * abs(inclination_degrees))
        B = BCD / (C * D)

        H = a8 * normal_force + a9 + a10 * inclination_degrees
        E = (a6 * normal_force + a7) * (1 - (a16 * inclination_degrees + a17) * math.copysign(1, slip_degrees + H))
        V = a11 * normal_force + a12 + (a13 * normal_force + a14) * inclination_degrees * normal_force
        Bx1 = B * (slip_degrees + H)
        
        return tire_scaling * (D * math.sin(C * math.atan(Bx1 - E * (Bx1 - math.atan(Bx1)))) + V)
    
    def _long_pacejka(self, normal_force:float, SR:float):
        if normal_force <= 0:
            return 0
        try:
            SR = SR * 100
            FZ = normal_force / 1000
            [b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12, b13] = self.longitudinal_coeffs
            C = b0
            D = FZ * (b1 * FZ + b2)

            BCD = (b3 * FZ**2 + b4 * FZ) * np.exp(-1 * b5 * FZ)

            B = BCD / (C * D)

            H = b9 * FZ + b10

            E = (b6 * FZ**2 + b7 * FZ + b8) * (1 - b13 * np.sign(SR + H))

            V = b11 * FZ + b12
            Bx1 = B * (SR + H)
            
            return (D * np.sin(C * np.arctan(Bx1 - E * (Bx1 - np.arctan(Bx1)))) + V) * self.tire_scaling
        except:
            return 0
        
    # Zero protection
    def _zero_prot(self, SA, SR, IA, FZ, Cs, Ca, lat_long):
        epsi = 1 / 1e64
        SA_adj = SA + epsi
        SR_adj = SR + epsi
        FX_adj = self._long_pacejka(FZ, SR_adj)
        FY_adj = self._lat_pacejka(IA, FZ, SA_adj)
        if lat_long == "lat":
            calc = ((FX_adj * FY_adj) / np.sqrt(SR_adj**2 * FY_adj**2 + FX_adj**2 * (np.tan(SA_adj))**2)) * (np.sqrt((1 - SR_adj)**2 * (np.cos(SA_adj))**2 * FY_adj**2 + (np.sin(SA_adj))**2 * Cs**2) / (Cs * np.cos(SA_adj)))
            return calc
        else:
            calc = ((FX_adj * FY_adj) / np.sqrt(SR_adj**2 * FY_adj**2 + FX_adj**2 * (np.tan(SA_adj))**2)) * (np.sqrt(SR_adj**2 * Ca**2 + (1 - SR_adj)**2 * (np.cos(SA_adj))**2 * FX_adj**2) / Ca)
            return calc

    # Long and lat formulas from Comstock
    def com_lat(self, SA, SR, FX, FY, IA, FZ, Cs):
        if np.sqrt(SR**2 * FY**2 + FX**2 * (np.tan(SA))**2) == 0:
            calc = self._zero_prot(SA, SR, IA, FZ, Cs, 0, "lat")
            return abs(calc) * -1 if SA < 0 else abs(calc)
        else:
            try:
                calc = ((FX * FY) / np.sqrt(SR**2 * FY**2 + FX**2 * (np.tan(SA))**2)) * (np.sqrt((1 - SR)**2 * (np.cos(SA))**2 * FY**2 + (np.sin(SA))**2 * Cs**2) / (Cs * np.cos(SA)))
                return abs(calc) * -1 if SA < 0 else abs(calc)
            except:
                calc = self._zero_prot(SA, SR, IA, FZ, Cs, 0, "lat")
                return abs(calc) * -1 if SA < 0 else abs(calc)
        
    def com_long(self, SA, SR, FX, FY, IA, FZ, Ca):
        if np.sqrt(SR**2 * FY**2 + FX**2 * (np.tan(SA))**2) == 0:
            calc = self._zero_prot(SA, SR, IA, FZ, 0, Ca, "long")
            return abs(calc) * -1 if SA < 0 else calc
        else:
            try:
                calc = ((FX * FY) / np.sqrt(SR**2 * FY**2 + FX**2 * (np.tan(SA))**2)) * (np.sqrt(SR**2 * Ca**2 + (1 - SR)**2 * (np.cos(SA))**2 * FX**2) / Ca)
                return abs(calc) * -1 if SR < 0 else abs(calc)
            except:
                calc = self._zero_prot(SA, SR, IA, FZ, 0, Ca, "long")
                return abs(calc) * -1 if SA < 0 else calc