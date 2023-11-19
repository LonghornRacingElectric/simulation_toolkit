from sim.system_models.vehicle_systems.vehicle_system_model import VehicleSystemModel

from sim.model_parameters.cars.car import Car
from sim.system_models.vectors.controls_vector import ControlsVector
from sim.system_models.vectors.observables_vector import ObservablesVector
from sim.system_models.vectors.state_vector import StateVector
from sim.system_models.vectors.state_dot_vector import StateDotVector

import math
import numpy as np

# tire_force * tire_radius = braking_torque + motor torque


    # Inputs:
    # SR - Slip Ratio (Longitudinal Slip) [-]
    # SA - Slip Angle (Lateral Slip) [rad]
    # IA - Camber Angle [rad]
    # FZ - Normal Wheel Load [N]
    #
    # Outputs:  
    # Fx - Longitudinal Force [Nm]
    # Fy - Lateral Force [Nm]
    # Mx - Overturning Couple [Nm]
    # My - Rolling Resistance Torque [Nm]
    # Mz - Self-Aligning Torque [Nm]


# Basic Tire Parameters:
# Fz0 - Nominal (rated) Load [N]
# R0 - Unloaded Tire Radius [m]
# mbelt - Tire Belt Mass [kg]
# dfz = (FZ - Fz0adj) / Fz0adj -  Normalized Vertical Load Increment [-]
# Fz0adj = Fz0 * Coeff_Fz0
# Vsx = Vx - AngSpeed * Re - Long slip speed
# Vsy = Vy
# SR = - Vsx/Vx
# SA = arctan(Vsy/abs(Vx))
# Vr = Re*AngSpeed - Linear rolling speed
# rho = FZ / Cz
# Re = R0 - rhoFz0 * (D*arctan(B*rhod) + F * rhod)
#   Re = Vx / AngSpeed  
# rhoFz0 = Fz0 / Cz
# rhod = rho / rhoFz0

# Pure Slip
#=====================================================================================
# Fx = Fx0(SR, FZ)
#   Fx0 = Dx * sin(Cx * arctan(Bx * SRx - Ex*(Bx*SRx - arctan(Bx * SRx)))) + SVx
#       SRx = SR + SHx
#       CamberAngleX = IA*Coeff_CamberAngleX
# Fy = Fy0(SA, FZ)
#   Fy0 = Dy*sin(Cy*arctan(By*SAy - Ey*(By*SAy - arctan(By*SAy)))) + SVy
#       SAy = SA + SHy
#       CamberAngleY = IA*Coeff_CamberAngleY
# Mzadj = Mz0(SA, IA, FZ)
#   Mz0 = -t * Fy0 + Mzr
#   t(SAt) = SA + SHt
#   Mzr(SAr) = Dr * cos(arctan(Br*SAr)) * cos(SA)
#   SAr = SA + SHr
#   SHf = SHy + SVy/Ky
#   CamberAngleZ = IA * Coeff_CamberAngleZ
#  
# Coefficients: 
# Cx = pCx1 * Coeff_Cx
# Dx = mux * FZ
# mux = (pDx1 + pDx2*dfz) * (1-pDx3 * CamberAngleX^2) 
# Ex = (pEx1 + pEx2*dfz + pEx3*dfz^2) * (1 - pEx4*sgn(SRx)) * Coeff_Ex(<=1)
# Kx = FZ * (pKx1 + pKx2*dfz) * exp(pKx3*dfz) * Coeff_Kx
# Bx = Kx / (Cx * Dx)
# SHx = (pHx1 + pHx2 * dfz) * Coeff_Hx
# SVx = FZ * (pVx1 + pVx2 * dfz) * Coeff_Vx * Coeff_mux
# 
# Cy = pCy1 * Coeff_Cy
# Dy = muy * FZ
# muy = (pDy1 + pDy2*dfz) * (1-pDy3 * CamberAngleY^2) * Coeff_muy 
# Ey = (pEy1 + pEy2*dfz + pEy3*dfz^2) * (1 - pEy4*sgn(SAy)) * Coeff_Ex(<=1)
# Ky = pKy1*Fz0*sin(2*arctan(FZ / (pKy2 * Fz0 * Coeff_Fz0))) * (1-pKy3*abs(CamberAngleY)) * Coeff_Fz0 * Coeff_Ky
# By = Ky / (Cy*Dy)
# SHy = (PHy1 + PHy2 * dfz) * Coeff_Hy + PHy3*CamberAngleY
# SVy = FZ*((pVy1 + pVy2*dfz) * Coeff_Vy + (pVy3 + pVy4 * dfz) * CamberAngleY) * Coeff_muy
#
# Bt = (qBz1 + qBz2*dfz + qBz3 * dfz ** 2) * (1 + qBz4 * CamberAngleZ + qBz5 * abs(CamberAngleZ)) * Coeff_Ky/Coeff_muy
# Ct = qCz1
# Dt = FZ * (qDz1 + qDz2 * dfz) * (1 + qDz3 + qDz4 * CamberAngleZ ** 2) * (R0 / Fz0) * Coeff_t
# Et = (qEz1 + qEz2 * dfz + qEz3 * dfz ** 2)
# SHt = qHz1 + qHz2*dfz + (qHz3 + qHz4 * dfz) * CamberAngleZ
# Br = qBz9 * Coeff_Ky / Coeff_muy + qBz10 * By * Cy
# Dr = FZ * ((qDz6 + qDz7 * dfz) * Coeff_r + (qDz8 + qDz9*dfz)* CamberAngleZ) * Ro * Coeff_muy
# Kz = -t * Ky 
#=====================================================================================





# Com_Stock
#=====================================================================================
# Fx = Fx0 * GxSA(SA, SR, FZ)
#   Fx = DxSA * cos(CxSA*arctan(BzSA*SAs ExSA BxSA * SAs - arctan(BxSA * SAs)))
# Fy = Fy0 * GySR + SVySR = 
#   GySR(SA, SR, IA, FZ) = 
#   cos(CySR * arctan(BySR * SRs - EySR*(BySR * SRs - arctan(BySR * SRs)))) /
#   (cos(CySR * arctan(BySR*SHySR - EySR*(BySR*SHySR - arctan(BySR*SHySR) ) )) ) 
# Fy = DySR * cos(CySR * arctan(BySR*SRs - EySR*(BySR * SRs - arctan(BySR*SRs)))) + SVySR
#   SRs = SR + SHySR

# Mx = Ro * FZ * (qSx1 * Coef_Vmx + (-qSx2*IA + qSx3*FZ/Fz0) * Coef_Mx )
# My = Ro * FZ * ( qSy1 + qSy2*Fx/Fz0 + qSy3*abs(Vx/Vref) + qSy4*(Vx/Vref)^4 )
# My = R0 * (SVx + Kx + SHx) - if qSy1 && qSy2 == 0
# Mzadj = -t * FyAdj + Mzr + s*Fx
#   t = Dt*cos(Ct*arctan(Bt*SAteq - Et*(Bt * SAteq - arctan(Bt*SAteq) ) ) ) * cos(SA)
#   FyAdj = Fy - SVySR
#   Mzr = Dr * cos(arctan(Br * SAreq)) * cos(SA)

# Arguments:
# SAteq = arctan(sqrt(tan^2(SAt) + (Kx/Ky)^2 * SR^2) * sgn(SAt)
# SAreq = arctan(sqrt(tan^2(SAr) + (Kx/Ky)^2 * SR^2) * sgn(SAr)
# SAs = SA * SHxSA

# Coefficients:
# BxSA = rBx1 * cos(arctan(rBx2*SR)) * Coeff_xSA
# CxSA = rCx1
# DxSA = Fxo / ( cos(CxSA*arctan(BxSA * SHxSA - ExSA*( BxSA*SHxSA - arctan(BxSA*SHxSA) )) ) )
# ExSA = rEx1 + rEx2 * dfz
# SHxSA = rHx1
#
# BySR = rBy1 * cos(arctan(rBy2(SA - rBy3))) * Coeff_ySR
# CySR = rCy1
# DySR = Fyo / ( cos(CySR*arctan(BySR * SHySR - EySR*( BySR*SHySR - arctan(BySR*SHySR) )) ) )
# EySR = rEy1 + rEy2 * dfz
# SHySR = rHy1 + rHy2*dfz
# SVySR = DVySR * sin(rVy5*arctan(rVy6*SR)) * Coeff_VySR
# DVySR = muy*FZ*(rVy1 + rVy2*dfz+rVy3*IA) * cos(arctan(rVy4*SA))
#=====================================================================================

class TireModel:
    def __init__(self):
        self.scaling_coeffs: list[float] = []
        self.longPure_coeffs: list[float] = []
        self.latPure_coeffs: list[float] = []
        self.alignPure_coeffs: list[float] = []

        self.scalingComp_coeffs: list[float] = []
        self.longComp_coeffs: list[float] = []
        self.latComp_coeffs: list[float] = []
        self.alignComp_coeffs: list[float] = []

        self.tire_scaling = 0.55
        

    def _get_long_pure_slip(self, SR: float, FZ: float, IA: float, normal_force: float):
        [Cx, mux, Dx, Kx, pCx1, pDx1, pDx2, pDx3, 
         pEx1, pEx2, pEx3, pEx4, pKx1,pKx2, pKx3, pHx1, pHx2, pVx1, pVx2] = self.longPure_coeffs
        [Coeff_CamberAngleX, Coeff_Vx, Coeff_mux, Coeff_Cx, Coeff_Kx, Coeff_Hx, Coeff_Ex, Coeff_Fz0] = self.scaling_coeffs
        Fz0 = normal_force
        Fz0adj = Fz0 * Coeff_Fz0
        dfz = (FZ - Fz0adj) / Fz0adj
        SRx = SR + SHx

        CamberAngleX = IA*Coeff_CamberAngleX
        Cx = pCx1 * Coeff_Cx
        mux = (pDx1 + pDx2*dfz) * (1-pDx3 * CamberAngleX*CamberAngleX) 
        Dx = mux * FZ
        Ex = (pEx1 + pEx2*dfz + pEx3*dfz*dfz) * (1 - pEx4*np.sgn(SRx)) * Coeff_Ex 
        Kx = FZ * (pKx1 + pKx2*dfz) * np.exp(pKx3*dfz) * Coeff_Kx
        Bx: float = Kx / (Cx * Dx)
        SHx: float = (pHx1 + pHx2 * dfz) * Coeff_Hx
        SVx: float = FZ * (pVx1 + pVx2 * dfz) * Coeff_Vx * Coeff_mux

        Fx0 = Dx * np.sin(Cx * np.arctan(Bx * SRx - Ex*(Bx*SRx - np.arctan(Bx * SRx)))) + SVx
        Fx = Fx0
        return Fx
    
    def get_lat_pure_slip(self, SA: float, FZ: float, IA: float, normal_force: float):
        [Cy, muy, Dy, Ky, pCy1, pDy1, pDy2, pDy3, 
         pEy1, pEy2, pEy3, pEy4, pKy1, pKy2, pKy3, PHy1, PHy2, PHy3, pVy1, pVy2, pVy3, pVy4] = self.latPure_coeffs
        [Coeff_CamberAngleY, Coeff_Vy, Coeff_muy, Coeff_Cy, Coeff_Ky, Coeff_Hy, Coeff_Ey, Coeff_Fz0] = self.scaling_coeffs
        Fz0 = normal_force
        Fz0adj = Fz0 * Coeff_Fz0
        dfz = (FZ - Fz0adj) / Fz0adj
        CamberAngleY = IA*Coeff_CamberAngleY


        SAy = SA + SHy
        SHy: float = (PHy1 + PHy2 * dfz) * Coeff_Hy + PHy3*CamberAngleY
        Cy = pCy1 * Coeff_Cy
        Dy = muy * FZ
        muy = (pDy1 + pDy2*dfz) * (1-pDy3 * CamberAngleY^2) * Coeff_muy 
        Ey = (pEy1 + pEy2*dfz + pEy3*dfz^2) * (1 - pEy4*np.sgn(SAy)) * Coeff_Ey
        Ky = pKy1*Fz0*np.sin(2*np.arctan(FZ / (pKy2 * Fz0 * Coeff_Fz0))) * (1-pKy3*abs(CamberAngleY)) * Coeff_Fz0 * Coeff_Ky
        By: float = Ky / (Cy*Dy)
        SVy: float = FZ*((pVy1 + pVy2*dfz) * Coeff_Vy + (pVy3 + pVy4 * dfz) * CamberAngleY) * Coeff_muy
        Fy0 = Dy*np.sin(Cy*np.arctan(By*SAy - Ey*(By*SAy - np.arctan(By*SAy)))) + SVy
        Fy = Fy0
        return Fy
    
    def get_pure_aligning_torque(self, SA: float, IA: float, FZ: float, normal_force: float):
        [Cy, muy, Dy, Ky, pCy1, pDy1, pDy2, pDy3, 
         qBz1, qBz2, qBz3, qBz4, qBz5, qCz1, qDz1, qDz2, qDz3, qDz4, qEz1, qEz2, qEz3,
           qHz1, qHz2, qHz3, qHz4, qBz9, qBz10, qDz6, qDz7, qDz8, qDz9, SHr] = self.latPure_coeffs
        [Coeff_CamberAngleZ, Coeff_t, Coeff_muy, Coeff_r, Coeff_Ky, Coeff_Hy, Coeff_Ey, Coeff_Fz0] = self.scaling_coeffs
        Fz0 = normal_force
        Fz0adj = Fz0 * Coeff_Fz0
        dfz = (FZ - Fz0adj) / Fz0adj

        Bt = (qBz1 + qBz2*dfz + qBz3 * dfz ** 2) * (1 + qBz4 * CamberAngleZ + qBz5 * abs(CamberAngleZ)) * Coeff_Ky/Coeff_muy
        Ct = qCz1
        By: float = Ky / (Cy*Dy)
        Dt = FZ * (qDz1 + qDz2 * dfz) * (1 + qDz3 + qDz4 * CamberAngleZ ** 2) * (R0 / Fz0) * Coeff_t
        Et = (qEz1 + qEz2 * dfz + qEz3 * dfz ** 2)
        SHt = qHz1 + qHz2*dfz + (qHz3 + qHz4 * dfz) * CamberAngleZ
        Br = qBz9 * Coeff_Ky / Coeff_muy + qBz10 * By * Cy
        Dr = FZ * ((qDz6 + qDz7 * dfz) * Coeff_r + (qDz8 + qDz9*dfz)* CamberAngleZ) * Ro * Coeff_muy
        Kz = -t * Ky
        
        Mzadj = Mz0(SA, IA, FZ)
        Mz0 = -t * self.get_lat_pure_slip() + Mzr
        SAt = SA + SHt
        t = Dt * np.cos(Ct * np.arctan(Bt*SAt - Et(Bt*SAt-np.arctan(Bt*SAt))))*np.cos(SA)
        Mzr = Dr * np.cos(np.arctan(Br*SAr)) * np.cos(SA)
        SAr = SA + SHr
        SHf = SHy + SVy/Ky
        CamberAngleZ = IA * Coeff_CamberAngleZ