from vehicle_model.suspension_model.suspension_elements._4_elements.push_pull_rod import PushPullRod
from vehicle_model.suspension_model.suspension_elements._5_elements.quarter_car import QuarterCar
from vehicle_model.suspension_model.suspension_elements._2_elements.bellcrank import Bellcrank
from vehicle_model.suspension_model.suspension_elements._2_elements.wishbone import Wishbone
from vehicle_model.suspension_model.suspension_elements._2_elements.stabar import Stabar
from vehicle_model.suspension_model.suspension_elements._2_elements.spring import Spring
from vehicle_model.suspension_model.suspension_elements._2_elements.tire import Tire
from vehicle_model.suspension_model.suspension_elements._1_elements.link import Link
from vehicle_model.suspension_model.suspension_elements._1_elements.node import Node
from LHR_tire_toolkit.MF52 import MF52 # type: ignore

from dataclasses import dataclass
from typing import Sequence
import pyvista as pv
import numpy as np

@dataclass
class Suspension:
    """
    ## Suspension Model

    Designed to model kinematics and force-based properties

    ###### Note, all conventions comply with SAE-J670 Z-up

    Parameters
    ----------
    """
    # Front Left
    FL_lower_inboard_fore: Node
    FL_lower_inboard_aft: Node
    FL_lower_outboard: Node

    FL_upper_inboard_fore: Node
    FL_upper_inboard_aft: Node
    FL_upper_outboard: Node

    FL_tie_inboard: Node
    FL_tie_outboard: Node

    FL_lower_pushrod: Node
    FL_upper_pushrod: Node
    FL_inboard_shock: Node

    FL_contact_patch: Node

    FL_spring_free_length: float
    FL_spring_rate: float

    # Rear Left
    RL_lower_inboard_fore: Node
    RL_lower_inboard_aft: Node
    RL_lower_outboard: Node

    RL_upper_inboard_fore: Node
    RL_upper_inboard_aft: Node
    RL_upper_outboard: Node

    RL_tie_inboard: Node
    RL_tie_outboard: Node

    RL_lower_pushrod: Node
    RL_upper_pushrod: Node
    RL_inboard_shock: Node

    RL_contact_patch: Node

    RL_spring_free_length: float
    RL_spring_rate: float

    # Rear Stabar
    Rr_stabar_left_arm_end: Node
    Rr_stabar_right_arm_end: Node
    Rr_stabar_left_droplink_end: Node
    Rr_stabar_right_droplink_end: Node
    Rr_stabar_bar_left_end: Node
    Rr_stabar_bar_right_end: Node
    Rr_stabar_torsional_stiffness: float

    def __post_init__(self) -> None:
        ####################
        ### Mirror Nodes ###
        ####################

        # Front Right
        self.FR_lower_inboard_fore = self.FL_lower_inboard_fore.mirrored_xz
        self.FR_lower_inboard_aft = self.FL_lower_inboard_aft.mirrored_xz
        self.FR_lower_outboard = self.FL_lower_outboard.mirrored_xz

        self.FR_upper_inboard_fore = self.FL_upper_inboard_fore.mirrored_xz
        self.FR_upper_inboard_aft = self.FL_upper_inboard_aft.mirrored_xz
        self.FR_upper_outboard = self.FL_upper_outboard.mirrored_xz

        self.FR_tie_inboard = self.FL_tie_inboard.mirrored_xz
        self.FR_tie_outboard = self.FL_tie_outboard.mirrored_xz

        self.FR_lower_pushrod = self.FL_lower_pushrod.mirrored_xz
        self.FR_upper_pushrod = self.FL_upper_pushrod.mirrored_xz
        self.FR_inboard_shock = self.FL_inboard_shock.mirrored_xz

        self.FR_contact_patch = self.FL_contact_patch.mirrored_xz

        self.FR_spring_free_length = self.FL_spring_free_length
        self.FR_spring_rate = self.FL_spring_rate

        # Rear Right    
        self.RR_lower_inboard_fore = self.RL_lower_inboard_fore.mirrored_xz
        self.RR_lower_inboard_aft = self.RL_lower_inboard_aft.mirrored_xz
        self.RR_lower_outboard = self.RL_lower_outboard.mirrored_xz

        self.RR_upper_inboard_fore = self.RL_upper_inboard_fore.mirrored_xz
        self.RR_upper_inboard_aft = self.RL_upper_inboard_aft.mirrored_xz
        self.RR_upper_outboard = self.RL_upper_outboard.mirrored_xz

        self.RR_tie_inboard = self.RL_tie_inboard.mirrored_xz
        self.RR_tie_outboard = self.RL_tie_outboard.mirrored_xz

        self.RR_lower_pushrod = self.RL_lower_pushrod.mirrored_xz
        self.RR_upper_pushrod = self.RL_upper_pushrod.mirrored_xz
        self.RR_inboard_shock = self.RL_inboard_shock.mirrored_xz

        self.RR_contact_patch = self.RL_contact_patch.mirrored_xz

        self.RR_spring_free_length = self.RL_spring_free_length
        self.RR_spring_rate = self.RL_spring_rate

        ##################
        ### Front-Left ###
        ##################

        # FL Links
        FL_lower_fore_link = Link(inboard_node=self.FL_lower_inboard_fore, outboard_node=self.FL_lower_outboard)
        FL_lower_aft_link = Link(inboard_node=self.FL_lower_inboard_aft, outboard_node=self.FL_lower_outboard)

        FL_upper_fore_link = Link(inboard_node=self.FL_upper_inboard_fore, outboard_node=self.FL_upper_outboard)
        FL_upper_aft_link = Link(inboard_node=self.FL_upper_inboard_aft, outboard_node=self.FL_upper_outboard)

        FL_tie_rod = Link(inboard_node=self.FL_tie_inboard, outboard_node=self.FL_tie_outboard)

        FL_pushrod = Link(inboard_node=self.FL_upper_pushrod, outboard_node=self.FL_lower_pushrod)
        FL_spring = Spring(inboard_node=self.FL_inboard_shock, outboard_node=self.FL_upper_pushrod, free_length=self.FL_spring_free_length, rate=self.FL_spring_rate)
        FL_push_pull_rod = PushPullRod(outboard_rod=FL_pushrod, spring=FL_spring)

        # FL Wishbones
        FL_lower_wishbone = Wishbone(fore_link=FL_lower_fore_link, aft_link=FL_lower_aft_link)
        FL_upper_wishbone = Wishbone(fore_link=FL_upper_fore_link, aft_link=FL_upper_aft_link)

        # FL Tire
        FL_mf52 = MF52(tire_name="FL_tire", file_path="./1_model_inputs/Modified_Round_8_Hoosier_R25B_16x7p5_10_on_7in_12psi_PAC02_UM2.tir")
        self.FL_tire = Tire(tire=FL_mf52, contact_patch=self.FL_contact_patch, outer_diameter=16*0.0254, width=7*0.0254, inner_diameter=10*0.0254)

        # Set relation between upper wishbone and pushrod
        self.FL_upper_outboard.add_child(node=self.FL_lower_pushrod)

        self.FL_quarter_car = QuarterCar(tire=self.FL_tire,
                                lower_wishbone=FL_lower_wishbone,
                                upper_wishbone=FL_upper_wishbone,
                                tie_rod=FL_tie_rod,
                                push_pull_rod=FL_push_pull_rod)
        
        #################
        ### Rear-Left ###
        #################

        # RL Links
        RL_lower_fore_link = Link(inboard_node=self.RL_lower_inboard_fore, outboard_node=self.RL_lower_outboard)
        RL_lower_aft_link = Link(inboard_node=self.RL_lower_inboard_aft, outboard_node=self.RL_lower_outboard)

        RL_upper_fore_link = Link(inboard_node=self.RL_upper_inboard_fore, outboard_node=self.RL_upper_outboard)
        RL_upper_aft_link = Link(inboard_node=self.RL_upper_inboard_aft, outboard_node=self.RL_upper_outboard)

        RL_tie_rod = Link(inboard_node=self.RL_tie_inboard, outboard_node=self.RL_tie_outboard)

        RL_pushrod = Link(inboard_node=self.RL_upper_pushrod, outboard_node=self.RL_lower_pushrod)
        RL_spring = Spring(inboard_node=self.RL_inboard_shock, outboard_node=self.RL_upper_pushrod, free_length=self.RL_spring_free_length, rate=self.RL_spring_rate)
        RL_push_pull_rod = PushPullRod(outboard_rod=RL_pushrod, spring=RL_spring)

        # RL Wishbones
        RL_lower_wishbone = Wishbone(fore_link=RL_lower_fore_link, aft_link=RL_lower_aft_link)
        RL_upper_wishbone = Wishbone(fore_link=RL_upper_fore_link, aft_link=RL_upper_aft_link)

        # RL Tire
        RL_mf52 = MF52(tire_name="RL_tire", file_path="./1_model_inputs/Modified_Round_8_Hoosier_R25B_16x7p5_10_on_7in_12psi_PAC02_UM2.tir")
        self.RL_tire = Tire(tire=RL_mf52, contact_patch=self.RL_contact_patch, outer_diameter=16*0.0254, width=7*0.0254, inner_diameter=10*0.0254)

        # Set relation between upper wishbone and pushrod
        self.RL_lower_outboard.add_child(node=self.RL_lower_pushrod)

        self.RL_quarter_car = QuarterCar(tire=self.RL_tire,
                                lower_wishbone=RL_lower_wishbone,
                                upper_wishbone=RL_upper_wishbone,
                                tie_rod=RL_tie_rod,
                                push_pull_rod=RL_push_pull_rod)
        
        ###################
        ### Front-Right ###
        ###################

        # FR Links
        FR_lower_fore_link = Link(inboard_node=self.FR_lower_inboard_fore, outboard_node=self.FR_lower_outboard)
        FR_lower_aft_link = Link(inboard_node=self.FR_lower_inboard_aft, outboard_node=self.FR_lower_outboard)

        FR_upper_fore_link = Link(inboard_node=self.FR_upper_inboard_fore, outboard_node=self.FR_upper_outboard)
        FR_upper_aft_link = Link(inboard_node=self.FR_upper_inboard_aft, outboard_node=self.FR_upper_outboard)

        FR_tie_rod = Link(inboard_node=self.FR_tie_inboard, outboard_node=self.FR_tie_outboard)

        FR_pushrod = Link(inboard_node=self.FR_upper_pushrod, outboard_node=self.FR_lower_pushrod)
        FR_spring = Spring(inboard_node=self.FR_inboard_shock, outboard_node=self.FR_upper_pushrod, free_length=self.FR_spring_free_length, rate=self.FR_spring_rate)
        FR_push_pull_rod = PushPullRod(outboard_rod=FR_pushrod, spring=FR_spring)

        # FR Wishbones
        FR_lower_wishbone = Wishbone(fore_link=FR_lower_fore_link, aft_link=FR_lower_aft_link)
        FR_upper_wishbone = Wishbone(fore_link=FR_upper_fore_link, aft_link=FR_upper_aft_link)

        # FR Tire
        FR_mf52 = MF52(tire_name="FR_tire", file_path="./1_model_inputs/Modified_Round_8_Hoosier_R25B_16x7p5_10_on_7in_12psi_PAC02_UM2.tir")
        self.FR_tire = Tire(tire=FR_mf52, contact_patch=self.FR_contact_patch, outer_diameter=16*0.0254, width=7*0.0254, inner_diameter=10*0.0254)

        # Set relation between upper wishbone and pushrod
        self.FR_upper_outboard.add_child(node=self.FR_lower_pushrod)

        self.FR_quarter_car = QuarterCar(tire=self.FR_tire,
                                lower_wishbone=FR_lower_wishbone,
                                upper_wishbone=FR_upper_wishbone,
                                tie_rod=FR_tie_rod,
                                push_pull_rod=FR_push_pull_rod)
        
        ##################
        ### Rear-Right ###
        ##################

        # RR Links
        RR_lower_fore_link = Link(inboard_node=self.RR_lower_inboard_fore, outboard_node=self.RR_lower_outboard)
        RR_lower_aft_link = Link(inboard_node=self.RR_lower_inboard_aft, outboard_node=self.RR_lower_outboard)

        RR_upper_fore_link = Link(inboard_node=self.RR_upper_inboard_fore, outboard_node=self.RR_upper_outboard)
        RR_upper_aft_link = Link(inboard_node=self.RR_upper_inboard_aft, outboard_node=self.RR_upper_outboard)

        RR_tie_rod = Link(inboard_node=self.RR_tie_inboard, outboard_node=self.RR_tie_outboard)

        RR_pushrod = Link(inboard_node=self.RR_upper_pushrod, outboard_node=self.RR_lower_pushrod)
        RR_spring = Spring(inboard_node=self.RR_inboard_shock, outboard_node=self.RR_upper_pushrod, free_length=self.RR_spring_free_length, rate=self.RR_spring_rate)
        RR_push_pull_rod = PushPullRod(outboard_rod=RR_pushrod, spring=RR_spring)

        # RR Wishbones
        RR_lower_wishbone = Wishbone(fore_link=RR_lower_fore_link, aft_link=RR_lower_aft_link)
        RR_upper_wishbone = Wishbone(fore_link=RR_upper_fore_link, aft_link=RR_upper_aft_link)

        # RR Tire
        RR_mf52 = MF52(tire_name="RR_tire", file_path="./1_model_inputs/Modified_Round_8_Hoosier_R25B_16x7p5_10_on_7in_12psi_PAC02_UM2.tir")
        self.RR_tire = Tire(tire=RR_mf52, contact_patch=self.RR_contact_patch, outer_diameter=16*0.0254, width=7*0.0254, inner_diameter=10*0.0254)

        # Set relation between upper wishbone and pushrod
        self.RR_lower_outboard.add_child(node=self.RR_lower_pushrod)

        self.RR_quarter_car = QuarterCar(tire=self.RR_tire,
                                lower_wishbone=RR_lower_wishbone,
                                upper_wishbone=RR_upper_wishbone,
                                tie_rod=RR_tie_rod,
                                push_pull_rod=RR_push_pull_rod)
        
        ###################
        ### Rear Stabar ###
        ###################

        Rr_stabar = Stabar(left_arm_end=self.Rr_stabar_left_arm_end, 
                           right_arm_end=self.Rr_stabar_right_arm_end, 
                           left_droplink_end=self.Rr_stabar_left_droplink_end, 
                           right_droplink_end=self.Rr_stabar_right_droplink_end, 
                           bar_left_end=self.Rr_stabar_bar_left_end, 
                           bar_right_end=self.Rr_stabar_bar_right_end, 
                           torsional_stiffness=self.Rr_stabar_torsional_stiffness)

        self.Rr_stabar_left_droplink_end.add_listener(Rr_stabar)
        self.Rr_stabar_right_droplink_end.add_listener(Rr_stabar)

        # Couple stabar to upper wishbone
        self.RL_upper_outboard.add_child(node=self.Rr_stabar_left_droplink_end)
        self.RR_upper_outboard.add_child(node=self.Rr_stabar_right_droplink_end)
        
        # Define all nodes, links, and springs for plotting

        # FL_quarter_car.jounce(jounce=2 * 0.0254)
        # RR_quarter_car.jounce(jounce=4 * 0.0254)
        # RL_quarter_car.jounce(jounce=-4 * 0.0254)

        nodes: Sequence[Node] = [self.FL_lower_inboard_fore, self.FL_lower_inboard_aft, self.FL_lower_outboard, self.FL_upper_inboard_fore, self.FL_upper_inboard_aft, self.FL_upper_outboard,
                                 self.FL_tie_inboard, self.FL_tie_outboard, self.FL_lower_pushrod, self.FL_upper_pushrod, self.FL_inboard_shock, self.FL_contact_patch,
                                 
                                 self.FR_lower_inboard_fore, self.FR_lower_inboard_aft, self.FR_lower_outboard, self.FR_upper_inboard_fore, self.FR_upper_inboard_aft, self.FR_upper_outboard,
                                 self.FR_tie_inboard, self.FR_tie_outboard, self.FR_lower_pushrod, self.FR_upper_pushrod, self.FR_inboard_shock, self.FR_contact_patch,
                                 
                                 self.RL_lower_inboard_fore, self.RL_lower_inboard_aft, self.RL_lower_outboard, self.RL_upper_inboard_fore, self.RL_upper_inboard_aft, self.RL_upper_outboard,
                                 self.RL_tie_inboard, self.RL_tie_outboard, self.RL_lower_pushrod, self.RL_upper_pushrod, self.RL_inboard_shock, self.RL_contact_patch,
                                 
                                 self.RR_lower_inboard_fore, self.RR_lower_inboard_aft, self.RR_lower_outboard, self.RR_upper_inboard_fore, self.RR_upper_inboard_aft, self.RR_upper_outboard,
                                 self.RR_tie_inboard, self.RR_tie_outboard, self.RR_lower_pushrod, self.RR_upper_pushrod, self.RR_inboard_shock, self.RR_contact_patch,
                                 
                                 self.Rr_stabar_left_arm_end, self.Rr_stabar_right_arm_end, self.Rr_stabar_left_droplink_end, self.Rr_stabar_right_droplink_end, self.Rr_stabar_bar_left_end, 
                                 self.Rr_stabar_bar_right_end,]

        links: Sequence[Link] = [FL_lower_fore_link, FL_lower_aft_link, FL_upper_fore_link, FL_upper_aft_link, FL_tie_rod, FL_pushrod,
                                 
                                 FR_lower_fore_link, FR_lower_aft_link, FR_upper_fore_link, FR_upper_aft_link, FR_tie_rod, FR_pushrod,
                                 
                                 RL_lower_fore_link, RL_lower_aft_link, RL_upper_fore_link, RL_upper_aft_link, RL_tie_rod, RL_pushrod,
                                 
                                 RR_lower_fore_link, RR_lower_aft_link, RR_upper_fore_link, RR_upper_aft_link, RR_tie_rod, RR_pushrod,
                                 
                                 Rr_stabar.left_arm, Rr_stabar.right_arm, Rr_stabar.left_droplink, Rr_stabar.right_droplink]

        springs: Sequence[Spring] = [FL_spring, FR_spring, RL_spring, RR_spring, Rr_stabar.bar]

        ground_nodes = [self.FL_contact_patch, self.FR_contact_patch, self.RL_contact_patch, self.RR_contact_patch]

        tires = [self.FL_tire, self.FR_tire, self.RL_tire, self.RR_tire]
        
        plotter = pv.Plotter()
        
        for node in nodes:
            sphere = pv.Sphere(radius=0.5 * 0.0254, center=node.position)
            plotter.add_mesh(sphere, color='red')

        for link in links:
            cylinder = pv.Cylinder(center=link.center, direction=link.direction, radius=0.625 / 2 * 0.0254, height=link.length)
            plotter.add_mesh(cylinder, color='gray')

        for spring in springs:
            cylinder = pv.Cylinder(center=spring.center, direction=spring.direction, radius=0.625 / 2 * 0.0254, height=spring.length)
            plotter.add_mesh(cylinder, color='green')
        
        for tire in tires:
            cylinder = pv.Cylinder(center=tire.center.position, direction=tire.direction, radius=tire.outer_diameter / 2, height=tire.width)

        ground_center = sum([np.array(node.initial_position) for node in ground_nodes]) / 4

        plane = pv.Plane(center=ground_center, i_size=(ground_nodes[0] - ground_nodes[2]).position[0], j_size=(ground_nodes[0] - ground_nodes[1]).position[1], i_resolution=10, j_resolution=10)
        plotter.add_mesh(plane, color="lightblue", opacity=0.6)

        plotter.show()
    
    def heave(self, heave: float):
        self.FL_quarter_car.jounce(jounce=heave)
        self.FR_quarter_car.jounce(jounce=heave)
        self.RL_quarter_car.jounce(jounce=heave)
        self.RR_quarter_car.jounce(jounce=heave)

    def generate_kin(self):
        pass