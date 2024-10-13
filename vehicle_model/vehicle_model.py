from vehicle_model.suspension_model.suspension_model import SuspensionModel
from vehicle_model.suspension_model.assets.plotter import Plotter
from vehicle_model._assets.processor import Processor
from LHR_tire_toolkit.MF52 import MF52
import numpy as np

class VehicleModel:
    """
    ## 6-DOF? 10-DOF? 14-DOF? Vehicle Model

    General-purpose vehicle model

    Parameters
    ----------
    defn_file_path : str
        File path to vehicle definition yaml
    tir_file_path : str
        File path to tire .tir file (assume all tires are the same for now)
    """
    def __init__(self, defn_file_path: str, tir_file_path: str) -> None:
        self.processor = Processor(file_path=defn_file_path)
        self.params = self.processor.params
        self.tir_file_path = tir_file_path

        self.initialize_suspension()

    def initialize_suspension(self) -> None:
        """
        ## Initialize Suspension

        Initializates suspension

        Parameters
        ----------
        None
        """
        hdpts = self.params['Geometric Properties']
        general = self.params['Suspension']
        self.masses = self.params["Mass Properties"]
        self.environment = self.params["Environment"]

        # Tires
        self.FL_tire = MF52(tire_name="front_left_tire", file_path=self.tir_file_path)
        self.FR_tire = MF52(tire_name="front_right_tire", file_path=self.tir_file_path)
        self.RL_tire = MF52(tire_name="rear_left_tire", file_path=self.tir_file_path)
        self.RR_tire = MF52(tire_name="rear_right_tire", file_path=self.tir_file_path)

        # Force properties
        self.Fr_UM = self.masses["FrUM"]
        self.Rr_UM = self.masses["RrUM"]
        self.SM = self.masses["SM"]
        self.DM = self.masses["DM"]

        self.total_mass = self.Fr_UM + self.Rr_UM + self.SM + self.DM
        self.FL_weight = self.total_mass * self.masses["CGBX"] * self.masses["CGBY"] * self.environment["G"]
        self.FR_weight = self.total_mass * self.masses["CGBX"] * (1 - self.masses["CGBY"]) * self.environment["G"]
        self.RL_weight = self.total_mass * (1 - self.masses["CGBX"]) * self.masses["CGBY"] * self.environment["G"]
        self.RR_weight = self.total_mass * (1 - self.masses["CGBX"]) * (1 - self.masses["CGBY"]) * self.environment["G"]

        # Suspension plotter
        self.suspension_plotter = Plotter()

        # General suspension
        self.suspension = SuspensionModel(
            FL_inboard_points=hdpts["FLIP"],
            FL_outboard_points=hdpts["FLOP"],
            FL_bellcrank_params=hdpts["FLBC"],
            FL_upper=hdpts["FL_Up"],
            FL_contact_patch=hdpts["FLCP"],
            FL_inclination_angle=hdpts["IA"][0],
            FL_toe=hdpts["Toe"][0],
            FL_rate=general["K"][0],
            FL_weight=self.FL_weight,
            
            FR_inboard_points=[[p[0], -1 * p[1], p[2]] for p in hdpts["FLIP"]],
            FR_outboard_points=[[p[0], -1 * p[1], p[2]] for p in hdpts["FLOP"]],
            FR_bellcrank_params=[[p[0], -1 * p[1], p[2]] for p in hdpts["FLBC"]],
            FR_upper=hdpts["FL_Up"],
            FR_contact_patch=[hdpts["FLCP"][0], -1 * hdpts["FLCP"][1], hdpts["FLCP"][2]],
            FR_inclination_angle=hdpts["IA"][0] * -1,
            FR_toe=hdpts["Toe"][0] * -1,
            FR_rate=general["K"][1],
            FR_weight=self.FR_weight,

            Fr_ARBK=general["ARBK"][0],
            Fr_ARBMR=general["ARBMR"][0],
            
            RL_inboard_points=hdpts["RLIP"],
            RL_outboard_points=hdpts["RLOP"],
            RL_bellcrank_params=hdpts["RLBC"],
            RL_upper=hdpts["RL_Up"],
            RL_contact_patch=hdpts["RLCP"],
            RL_inclination_angle=hdpts["IA"][1],
            RL_toe=hdpts["Toe"][1],
            RL_rate=general["K"][2],
            RL_weight=self.RL_weight,
            
            RR_inboard_points=[[p[0], -1 * p[1], p[2]] for p in hdpts["RLIP"]],
            RR_outboard_points=[[p[0], -1 * p[1], p[2]] for p in hdpts["RLOP"]],
            RR_bellcrank_params=[[p[0], -1 * p[1], p[2]] for p in hdpts["RLBC"]],
            RR_upper=hdpts["RL_Up"],
            RR_contact_patch=[hdpts["RLCP"][0], -1 * hdpts["RLCP"][1], hdpts["RLCP"][2]],
            RR_inclination_angle=hdpts["IA"][1] * -1,
            RR_toe=hdpts["Toe"][1] * -1,
            RR_rate=general["K"][3],
            RR_weight=self.RR_weight,

            Rr_ARBK=general["ARBK"][1],
            Rr_ARBMR=general["ARBMR"][1],
            
            tire_radius=self.FL_tire._dimensions["UNLOADED_RADIUS"],
            tire_width=self.FL_tire._dimensions["WIDTH"],
            cg_location=[-1.016, 0, 0.2794],
            show_ICs=False,
            plotter=self.suspension_plotter)
        
        self.suspension.generate_kin()

        # self.suspension.generate_kin_plots(steer_sweep=np.linspace(-1.5, 1.5, 26) * 0.0254, 
        #                                    heave_sweep=np.linspace(-4, 4, 26) * 0.0254,
        #                                    pitch_sweep=np.linspace(-3, 3, 26) * np.pi / 180,
        #                                    roll_sweep=np.linspace(-3, 3, 26) * np.pi / 180)

        self.suspension.plot_elements(plotter=self.suspension_plotter, verbose=False, show_grid=False)
        
        self.suspension_plotter.add_slider(func=self.suspension.roll_slider, title="Roll", bounds=[-15, 15], pos=[[0.75, 0.1], [0.98, 0.1]])
        self.suspension_plotter.add_slider(func=self.suspension.pitch_slider, title="Pitch", bounds=[-10, 10], pos=[[0.75, 0.225], [0.98, 0.225]])
        self.suspension_plotter.add_slider(func=self.suspension.heave_slider, title="Heave", bounds=[-0.0508, 0.0508], pos=[[0.75, 0.350], [0.98, 0.350]])
        self.suspension_plotter.add_slider(func=self.suspension.steer_slider, title="Steer", bounds=[-0.0508, 0.0508], pos=[[0.75, 0.475], [0.98, 0.475]])

        self.suspension_plotter.show()