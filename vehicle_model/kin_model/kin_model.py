import yaml
import matplotlib.pyplot as plt
import numpy as np

from scipy.optimize import minimize


class KinModel:
    def __init__(self, vehicle_yaml: str, corner: int = 0) -> None:
        with open(vehicle_yaml, 'r') as file:
            self.params = yaml.safe_load(file)

        vecs = self._initialize_vecs(corner = corner)
        
        self.CP = vecs[0]
        self.vecs = vecs[1]

    def _initialize_vecs(self, corner: int):
        self.hdpts = self.params["Hardpoints"]
        
        scale = np.array(
        [
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        ]
        )

        Fr = self.params["Hardpoints"]["Fr"]
        Rr = self.params["Hardpoints"]["Rr"]

        Fr_CP = self.params["CP"]["Fr"]
        Rr_CP = self.params["CP"]["Rr"]

        # Identify corner of interest and initialize necessary transformation
        if (corner == 0):
            vecs = Fr
            CP_loc = Fr_CP
        elif (corner == 1):
            vecs = Fr
            CP_loc = Fr_CP
            scale[1][1] = -1
        elif (corner == 2):
            vecs = Rr
            CP_loc = Rr_CP
        elif (corner == 3):
            vecs = Rr
            CP_loc = Rr_CP
            scale[1][1] = -1
        else:
            raise(Exception("Please enter a corner value 0 to 3: 0 -> FL, 1 -> FR, 2 -> RL, 3 -> RR"))

        # Apply transformation to each entry
        for key in list(vecs.keys()):
            for i in [0, 1]:
                vec = np.array([vecs[key][i][0], vecs[key][i][1], vecs[key][i][2]])
                vecs[key][i] = np.matmul(scale, vec)

        CP_loc = np.matmul(scale, [CP_loc[0], CP_loc[1], CP_loc[2]])

        return [CP_loc, vecs]

    def jounce(self, magnitude: float = 0):
        
        # store jounce in kinematic model
        self.CP_z = magnitude
        
        # set initial guess
        initial_guess = []
        
        for key in self.vecs.keys():
            initial_guess += list(self.vecs[key][1])
        
        initial_guess += [self.CP[0], self.CP[1]]
        initial_guess = [float(x) for x in initial_guess]

        # save new points
        new_points = minimize(fun = self._jounce_residuals, x0 = initial_guess).x

        # update hdpts and contact patch
        keys = list(self.vecs.keys())
        for key in self.vecs.keys():
            self.vecs[key][1] = new_points[0 + 3 * keys.index(key) : 3 + 3 * keys.index(key)]
        
        self.CP = [new_points[-2], new_points[-1], self.CP_z]

    def _jounce_residuals(self, inputs: list):
        
        FU_x = inputs[0]
        FU_y = inputs[1]
        FU_z = inputs[2]
        RU_x = inputs[3]
        RU_y = inputs[4]
        RU_z = inputs[5]
        FL_x = inputs[6]
        FL_y = inputs[7]
        FL_z = inputs[8]
        RL_x = inputs[9]
        RL_y = inputs[10]
        RL_z = inputs[11]
        tie_x = inputs[12]
        tie_y = inputs[13]
        tie_z = inputs[14]
        CP_x = inputs[15]
        CP_y = inputs[16]
        
        # Transform all iterated vectors to the origin and take their norm
        FU_i = np.linalg.norm(np.array([FU_x, FU_y, FU_z]) - self.vecs["FU"][0])
        RU_i = np.linalg.norm(np.array([RU_x, RU_y, RU_z]) - self.vecs["RU"][0])
        FL_i = np.linalg.norm(np.array([FL_x, FL_y, FL_z]) - self.vecs["FL"][0])
        RL_i = np.linalg.norm(np.array([RL_x, RL_y, RL_z]) - self.vecs["RL"][0])
        
        Tie_i = np.linalg.norm(np.array([tie_x, tie_y, tie_z]) - self.vecs["Tie"][0])

        CP_FU_i = np.linalg.norm(np.array([FU_x, FU_y, FU_z]) - np.array([CP_x, CP_y, self.CP_z]))
        CP_RU_i = np.linalg.norm(np.array([RU_x, RU_y, RU_z]) - np.array([CP_x, CP_y, self.CP_z]))
        CP_FL_i = np.linalg.norm(np.array([FL_x, FL_y, FL_z]) - np.array([CP_x, CP_y, self.CP_z]))
        CP_RL_i = np.linalg.norm(np.array([RL_x, RL_y, RL_z]) - np.array([CP_x, CP_y, self.CP_z]))

        CP_Tie_i = np.linalg.norm(np.array([tie_x, tie_y, tie_z]) - np.array([CP_x, CP_y, self.CP_z]))
        Upr_Tie_i = np.linalg.norm(np.array([FU_x, FU_y, FU_z]) - np.array([tie_x, tie_y, tie_z]))
        Lwr_Tie_i = np.linalg.norm(np.array([FL_x, FL_y, FL_z]) - np.array([tie_x, tie_y, tie_z]))

        outboard_Fr_i = np.linalg.norm(np.array([FU_x, FU_y, FU_z]) - np.array([FL_x, FL_y, FL_z]))
        outboard_Rr_i = np.linalg.norm(np.array([RU_x, RU_y, RU_z]) - np.array([RL_x, RL_y, RL_z]))
        outboard_Upr_i = np.linalg.norm(np.array([FU_x, FU_y, FU_z]) - np.array([RU_x, RU_y, RU_z]))
        outboard_Lwr_i = np.linalg.norm(np.array([FL_x, FL_y, FL_z]) - np.array([RL_x, RL_y, RL_z]))

        # Transform all fixed vectors to the origin and take their norm
        FU_f = np.linalg.norm(self.vecs["FU"][1] - self.vecs["FU"][0])
        RU_f = np.linalg.norm(self.vecs["RU"][1] - self.vecs["RU"][0])
        FL_f = np.linalg.norm(self.vecs["FL"][1] - self.vecs["FL"][0])
        RL_f = np.linalg.norm(self.vecs["RL"][1] - self.vecs["RL"][0])
        
        Tie_f = np.linalg.norm(self.vecs["Tie"][1] - self.vecs["Tie"][0])

        CP_FU_f = np.linalg.norm(self.vecs["FU"][1] - self.CP)
        CP_RU_f = np.linalg.norm(self.vecs["RU"][1] - self.CP)
        CP_FL_f = np.linalg.norm(self.vecs["FL"][1] - self.CP)
        CP_RL_f = np.linalg.norm(self.vecs["RL"][1] - self.CP)

        CP_Tie_f = np.linalg.norm(self.vecs["Tie"][1] - self.CP)
        Upr_Tie_f = np.linalg.norm(self.vecs["FU"][1] - self.vecs["Tie"][1])
        Lwr_Tie_f = np.linalg.norm(self.vecs["FL"][1] - self.vecs["Tie"][1])

        outboard_Fr_f = np.linalg.norm(self.vecs["FU"][1] - self.vecs["FL"][1])
        outboard_Rr_f = np.linalg.norm(self.vecs["RU"][1] - self.vecs["RL"][1])
        outboard_Upr_f = np.linalg.norm(self.vecs["FU"][1] - self.vecs["RU"][1])
        outboard_Lwr_f = np.linalg.norm(self.vecs["FL"][1] - self.vecs["RL"][1])


        # Take all residuals
        resid = [
            FU_i - FU_f,
            RU_i - RU_f,
            FL_i - FL_f,
            RL_i - RL_f,
            Tie_i - Tie_f,
            CP_FU_i - CP_FU_f,
            CP_RU_i - CP_RU_f,
            CP_FL_i - CP_FL_f,
            CP_RL_i - CP_RL_f,
            CP_Tie_i - CP_Tie_f,
            outboard_Fr_i - outboard_Fr_f,
            outboard_Rr_i - outboard_Rr_f,
            outboard_Upr_i - outboard_Upr_f,
            outboard_Lwr_i - outboard_Lwr_f,
            Upr_Tie_i - Upr_Tie_f,
            Lwr_Tie_i - Lwr_Tie_f
        ]

        return np.linalg.norm(resid)

    def plot(self, vecs, ax):

        # plot suspension links
        for key in list(vecs.keys()):
            x_vals = [entry[0] for entry in vecs[key]]
            y_vals = [entry[1] for entry in vecs[key]]
            z_vals = [entry[2] for entry in vecs[key]]

            ax.plot(x_vals, y_vals, z_vals, color = 'm')

        # ax.set_box_aspect((np.ptp(x_vals), np.ptp(y_vals), np.ptp(z_vals)))

        # plot contact patch
        ax.scatter(self.CP[0], self.CP[1], self.CP[2], color = 'r')