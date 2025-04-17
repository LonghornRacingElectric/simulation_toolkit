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
from typing import Union
import yaml

class SuspensionData:
    """
    ## Suspension Assembler

    Assembles suspension from from vehicle definition yaml

    Parameters
    ----------
    path : str
        File path to vehicle definition yaml
    """
    def __init__(self, path: str):
        with open(path) as f:
            try:
                raw_params: dict[str, dict[str, dict]] = yaml.safe_load(f)
            except yaml.YAMLError as error:
                print("Failed to import yaml file. Reason:\n")
                print(error)

        self.FL_nodes: dict[str, Node] = {}
        self.FR_nodes: dict[str, Node] = {}
        self.RL_nodes: dict[str, Node] = {}
        self.RR_nodes: dict[str, Node] = {}

        self.FL_links: dict[str, Link] = {}
        self.FR_links: dict[str, Link] = {}
        self.RL_links: dict[str, Link] = {}
        self.RR_links: dict[str, Link] = {}

        self.FL_bellcranks: dict[str, Bellcrank] = {}
        self.FR_bellcranks: dict[str, Bellcrank] = {}
        self.RL_bellcranks: dict[str, Bellcrank] = {}
        self.RR_bellcranks: dict[str, Bellcrank] = {}

        self.Fr_springs: dict[str, Link] = {}
        self.Rr_springs: dict[str, Link] = {}

        self.Fr_stabar_links: dict[str, Link] = {}
        self.Rr_stabar_links: dict[str, Link] = {}

        self.Fr_stabar: Union[Stabar, None]
        self.Rr_stabar: Union[Stabar, None]

        if "FL QuarterCar" in raw_params.keys():
            
            #################
            ### FL Corner ###
            #################

            # Tire
            if "tire" in raw_params["FL QuarterCar"].keys():
                FL_tire_params = raw_params["FL QuarterCar"]["tire"]
                FL_tir_file_path = FL_tire_params["tir_path"]["Value"]
                FL_contact_patch = Node(position=FL_tire_params["contact_patch"]["Value"])
                FL_tire_od = FL_tire_params["outer_diameter"]["Value"]
                FL_tire_id = FL_tire_params["inner_diameter"]["Value"]
                FL_tire_width = FL_tire_params["width"]["Value"]
                FL_tire_toe = FL_tire_params["static_toe"]["Value"]
                FL_tire_camber = FL_tire_params["static_camber"]["Value"]

                FL_mf52 = MF52(tire_name="FL_tire", file_path=FL_tir_file_path)
                self.FL_tire = Tire(tire=FL_mf52, contact_patch=FL_contact_patch, outer_diameter=FL_tire_od, width=FL_tire_width,
                                    inner_diameter=FL_tire_id, static_toe=FL_tire_toe, static_gamma=FL_tire_camber)

                self.FL_nodes["contact_patch"] = FL_contact_patch

            else:
                raise Exception('Vehicle definition yaml must contain "tire" under "FL QuarterCar"')
            
            # Lower wishbone
            if "lower_wishbone" in raw_params["FL QuarterCar"].keys():
                FL_lower_params = raw_params["FL QuarterCar"]["lower_wishbone"]
                FL_lower_fore_inboard = Node(position=FL_lower_params["fore_inboard"]["Value"])
                FL_lower_aft_inboard = Node(position=FL_lower_params["aft_inboard"]["Value"])
                FL_lower_outboard = Node(position=FL_lower_params["outboard"]["Value"])

                FL_lower_fore = Link(inboard_node=FL_lower_fore_inboard, outboard_node=FL_lower_outboard)
                FL_lower_aft = Link(inboard_node=FL_lower_aft_inboard, outboard_node=FL_lower_outboard)

                FL_lower_wb = Wishbone(fore_link=FL_lower_fore, aft_link=FL_lower_aft)

                self.FL_nodes["lower_fore_inboard"] = FL_lower_fore_inboard
                self.FL_nodes["lower_aft_inboard"] = FL_lower_aft_inboard
                self.FL_nodes["lower_outboard"] = FL_lower_outboard

                self.FL_links["lower_fore"] = FL_lower_fore
                self.FL_links["lower_aft"] = FL_lower_aft

            else:
                raise Exception('Vehicle definition yaml must contain "lower_wishbone" under "FL QuarterCar"')

            # Upper wishbone
            if "upper_wishbone" in raw_params["FL QuarterCar"].keys():
                FL_upper_params = raw_params["FL QuarterCar"]["upper_wishbone"]
                FL_upper_fore_inboard = Node(position=FL_upper_params["fore_inboard"]["Value"])
                FL_upper_aft_inboard = Node(position=FL_upper_params["aft_inboard"]["Value"])
                FL_upper_outboard = Node(position=FL_upper_params["outboard"]["Value"])

                FL_upper_fore = Link(inboard_node=FL_upper_fore_inboard, outboard_node=FL_upper_outboard)
                FL_upper_aft = Link(inboard_node=FL_upper_aft_inboard, outboard_node=FL_upper_outboard)

                FL_upper_wb = Wishbone(fore_link=FL_upper_fore, aft_link=FL_upper_aft)

                self.FL_nodes["upper_fore_inboard"] = FL_upper_fore_inboard
                self.FL_nodes["upper_aft_inboard"] = FL_upper_aft_inboard
                self.FL_nodes["upper_outboard"] = FL_upper_outboard

                self.FL_links["upper_fore"] = FL_upper_fore
                self.FL_links["upper_aft"] = FL_upper_aft

            else:
                raise Exception('Vehicle definition yaml must contain "upper_wishbone" under "FL QuarterCar"')
            
            # Tie rod
            if "tie_rod" in raw_params["FL QuarterCar"].keys():
                FL_tie_params = raw_params["FL QuarterCar"]["tie_rod"]
                FL_tie_inboard = Node(position=FL_tie_params["inboard"]["Value"])
                FL_tie_outboard = Node(position=FL_tie_params["outboard"]["Value"])

                FL_tie = Link(inboard_node=FL_tie_inboard, outboard_node=FL_tie_outboard)

                self.FL_nodes["tie_rod_inboard"] = FL_tie_inboard
                self.FL_nodes["tie_rod_outboard"] = FL_tie_outboard

                self.FL_links["tie"] = FL_tie

            else:
                raise Exception('Vehicle definition yaml must contain "tie_rod" under "FL QuarterCar"')
            
            if raw_params["FL QuarterCar"]["steering_ratio"]["Value"]:
                self.steering_ratio = raw_params["FL QuarterCar"]["steering_ratio"]["Value"]
            
            else:
                raise Exception("You must set a steering ratio")

            # Push/pull rod
            if "push_pull_rod" in raw_params["FL QuarterCar"].keys():
                FL_ppr_params = raw_params["FL QuarterCar"]["push_pull_rod"]
                FL_ppr_outboard_inboard = Node(position=FL_ppr_params["outboard_rod"]["inboard_node"]["Value"])
                FL_ppr_outboard_outboard = Node(position=FL_ppr_params["outboard_rod"]["outboard_node"]["Value"])
                
                FL_ppr_spring_inboard = Node(position=FL_ppr_params["spring"]["inboard_node"]["Value"])
                FL_ppr_spring_outboard = FL_ppr_outboard_inboard

                if FL_ppr_params["connection"]["Value"] not in list(self.FL_nodes.keys()):
                        raise Exception(f"Push/pull rod connection must be one of the following: {list(self.FL_nodes.keys())}")
                else:
                    self.FL_nodes[FL_ppr_params["connection"]["Value"]].add_child(node=FL_ppr_outboard_outboard)

                if FL_ppr_params["inboard_rod"]["inboard_node"]["Value"]:
                    FL_ppr_inboard_inboard = Node(position=FL_ppr_params["inboard_rod"]["inboard_node"]["Value"])
                    FL_ppr_inboard_outboard = Node(position=FL_ppr_params["inboard_rod"]["outboard_node"]["Value"])

                    FL_bc_pivot = Node(position=FL_ppr_params["bellcrank"]["pivot"]["Value"])
                    FL_bc_direction = FL_ppr_params["bellcrank"]["pivot_direction"]["Value"]
                    FL_bc_nodes = [Node(position=pos) for pos in FL_ppr_params["bellcrank"]["nodes"]]

                    FL_ppr_outboard_rod = Link(inboard_node=FL_ppr_outboard_inboard, outboard_node=FL_ppr_outboard_outboard)
                    FL_bc = Bellcrank(*FL_bc_nodes, pivot=FL_bc_pivot, pivot_direction=FL_bc_direction)
                    FL_inboard_rod = Link(inboard_node=FL_ppr_inboard_inboard, outboard_node=FL_ppr_inboard_outboard)
                    FL_spring = Spring(inboard_node=FL_ppr_spring_inboard, outboard_node=FL_ppr_spring_outboard,
                                       free_length=FL_ppr_params["spring"]["free_length"]["Value"], rate=FL_ppr_params["spring"]["rate"]["Value"])
                    FL_ppr = PushPullRod(outboard_rod=FL_ppr_outboard_rod, spring=FL_spring, inboard_rod=FL_inboard_rod, bellcrank=FL_bc)

                    self.FL_nodes["push_pull_rod_inboard_inboard"] = FL_ppr_inboard_inboard
                    self.FL_nodes["push_pull_rod_inboard_outboard"] = FL_ppr_inboard_outboard
                    self.FL_nodes["push_pull_rod_spring_outboard"] = FL_ppr_spring_outboard
                    self.FL_nodes["bellcrank_pivot"] = FL_bc_pivot

                    self.FL_links["push_pull_inboard"] = FL_inboard_rod
                    self.FL_bellcranks["FL_push_pull_bellcrank"] = FL_bc
                    
                    for i, node in enumerate(FL_bc_nodes):
                        self.FL_nodes[f"bellcrank_node_{i}"] = node

                else:
                    FL_ppr_outboard_rod = Link(inboard_node=FL_ppr_outboard_inboard, outboard_node=FL_ppr_outboard_outboard)
                    FL_spring = Spring(inboard_node=FL_ppr_spring_inboard, outboard_node=FL_ppr_outboard_inboard,
                                       free_length=FL_ppr_params["spring"]["free_length"]["Value"], rate=FL_ppr_params["spring"]["rate"]["Value"])
                    FL_ppr = PushPullRod(outboard_rod=FL_ppr_outboard_rod, spring=FL_spring)

                    self.FL_nodes["push_pull_rod_spring_outboard"] = FL_ppr_outboard_inboard
                
                self.FL_links["push_pull_outboard"] = FL_ppr_outboard_rod
                self.Fr_springs["FL_spring"] = FL_spring

                self.FL_nodes["push_pull_rod_outboard_inboard"] = FL_ppr_outboard_inboard
                self.FL_nodes["push_pull_rod_outboard_outboard"] = FL_ppr_outboard_outboard
                self.FL_nodes["push_pull_rod_spring_inboard"] = FL_ppr_spring_inboard

                self.FL_quarter_car = QuarterCar(tire=self.FL_tire,
                                                 lower_wishbone=FL_lower_wb,
                                                 upper_wishbone=FL_upper_wb,
                                                 tie_rod=FL_tie,
                                                 push_pull_rod=FL_ppr)

            else:
                raise Exception('Vehicle definition yaml must contain "push_pull_rod" under "FL QuarterCar"')

            #################
            ### FR Corner ###
            #################

            # Tire
            FR_tire_params = raw_params["FL QuarterCar"]["tire"]
            FR_tir_file_path = FR_tire_params["tir_path"]["Value"]
            FR_contact_patch = Node(position=FR_tire_params["contact_patch"]["Value"]).mirrored_xz
            FR_tire_od = FR_tire_params["outer_diameter"]["Value"]
            FR_tire_id = FR_tire_params["inner_diameter"]["Value"]
            FR_tire_width = FR_tire_params["width"]["Value"]
            FR_tire_toe = -FR_tire_params["static_toe"]["Value"]
            FR_tire_camber = -FR_tire_params["static_camber"]["Value"]

            FR_mf52 = MF52(tire_name="FR_tire", file_path=FR_tir_file_path)
            self.FR_tire = Tire(tire=FR_mf52, contact_patch=FR_contact_patch, outer_diameter=FR_tire_od, width=FR_tire_width,
                                inner_diameter=FR_tire_id, static_toe=FR_tire_toe, static_gamma=FR_tire_camber)

            self.FR_nodes["contact_patch"] = FR_contact_patch
            
            # Lower wishbone
            FR_lower_params = raw_params["FL QuarterCar"]["lower_wishbone"]
            FR_lower_fore_inboard = Node(position=FR_lower_params["fore_inboard"]["Value"]).mirrored_xz
            FR_lower_aft_inboard = Node(position=FR_lower_params["aft_inboard"]["Value"]).mirrored_xz
            FR_lower_outboard = Node(position=FR_lower_params["outboard"]["Value"]).mirrored_xz

            FR_lower_fore = Link(inboard_node=FR_lower_fore_inboard, outboard_node=FR_lower_outboard)
            FR_lower_aft = Link(inboard_node=FR_lower_aft_inboard, outboard_node=FR_lower_outboard)

            FR_lower_wb = Wishbone(fore_link=FR_lower_fore, aft_link=FR_lower_aft)

            self.FR_nodes["lower_fore_inboard"] = FR_lower_fore_inboard
            self.FR_nodes["lower_aft_inboard"] = FR_lower_aft_inboard
            self.FR_nodes["lower_outboard"] = FR_lower_outboard

            self.FR_links["lower_fore"] = FR_lower_fore
            self.FR_links["lower_aft"] = FR_lower_aft

            # Upper wishbone
            FR_upper_params = raw_params["FL QuarterCar"]["upper_wishbone"]
            FR_upper_fore_inboard = Node(position=FR_upper_params["fore_inboard"]["Value"]).mirrored_xz
            FR_upper_aft_inboard = Node(position=FR_upper_params["aft_inboard"]["Value"]).mirrored_xz
            FR_upper_outboard = Node(position=FR_upper_params["outboard"]["Value"]).mirrored_xz

            FR_upper_fore = Link(inboard_node=FR_upper_fore_inboard, outboard_node=FR_upper_outboard)
            FR_upper_aft = Link(inboard_node=FR_upper_aft_inboard, outboard_node=FR_upper_outboard)

            FR_upper_wb = Wishbone(fore_link=FR_upper_fore, aft_link=FR_upper_aft)

            self.FR_nodes["upper_fore_inboard"] = FR_upper_fore_inboard
            self.FR_nodes["upper_aft_inboard"] = FR_upper_aft_inboard
            self.FR_nodes["upper_outboard"] = FR_upper_outboard

            self.FR_links["upper_fore"] = FR_upper_fore
            self.FR_links["upper_aft"] = FR_upper_aft

            # Tie rod
            FR_tie_params = raw_params["FL QuarterCar"]["tie_rod"]
            FR_tie_inboard = Node(position=FR_tie_params["inboard"]["Value"]).mirrored_xz
            FR_tie_outboard = Node(position=FR_tie_params["outboard"]["Value"]).mirrored_xz

            FR_tie = Link(inboard_node=FR_tie_inboard, outboard_node=FR_tie_outboard)

            self.FR_nodes["tie_rod_inboard"] = FR_tie_inboard
            self.FR_nodes["tie_rod_outboard"] = FR_tie_outboard

            self.FR_links["tie"] = FR_tie

            # Push/pull rod
            FR_ppr_params = raw_params["FL QuarterCar"]["push_pull_rod"]
            FR_ppr_outboard_inboard = Node(position=FR_ppr_params["outboard_rod"]["inboard_node"]["Value"]).mirrored_xz
            FR_ppr_outboard_outboard = Node(position=FR_ppr_params["outboard_rod"]["outboard_node"]["Value"]).mirrored_xz
            
            FR_ppr_spring_inboard = Node(position=FR_ppr_params["spring"]["inboard_node"]["Value"]).mirrored_xz
            FR_ppr_spring_outboard = Node(position=FR_ppr_params["spring"]["outboard_node"]["Value"]).mirrored_xz

            if FR_ppr_params["connection"]["Value"] not in list(self.FR_nodes.keys()):
                    raise Exception(f"Push/pull rod connection must be one of the following: {list(self.FR_nodes.keys())}")
            else:
                self.FR_nodes[FR_ppr_params["connection"]["Value"]].add_child(node=FR_ppr_outboard_outboard)

            if FR_ppr_params["inboard_rod"]["inboard_node"]["Value"]:
                FR_ppr_inboard_inboard = Node(position=FR_ppr_params["inboard_rod"]["inboard_node"]["Value"]).mirrored_xz
                FR_ppr_inboard_outboard = Node(position=FR_ppr_params["inboard_rod"]["outboard_node"]["Value"]).mirrored_xz

                FR_bc_pivot = Node(position=FR_ppr_params["bellcrank"]["pivot"]["Value"]).mirrored_xz
                FR_bc_direction = Node(position=FR_ppr_params["bellcrank"]["pivot_direction"]["Value"]).mirrored_xz
                FR_bc_nodes = [Node(position=pos).mirrored_xz for pos in FR_ppr_params["bellcrank"]["nodes"]]

                FR_ppr_outboard_rod = Link(inboard_node=FR_ppr_outboard_inboard, outboard_node=FR_ppr_outboard_outboard)
                FR_bc = Bellcrank(*FR_bc_nodes, pivot=FR_bc_pivot, pivot_direction=FR_bc_direction)
                FR_inboard_rod = Link(inboard_node=FR_ppr_inboard_inboard, outboard_node=FR_ppr_inboard_outboard)
                FR_spring = Spring(inboard_node=FR_ppr_spring_inboard, outboard_node=FR_ppr_spring_outboard,
                                    free_length=FR_ppr_params["spring"]["free_length"]["Value"], rate=FR_ppr_params["spring"]["rate"]["Value"])
                FR_ppr = PushPullRod(outboard_rod=FR_ppr_outboard_rod, spring=FR_spring, inboard_rod=FR_inboard_rod, bellcrank=FR_bc)

                self.FR_nodes["push_pull_rod_inboard_inboard"] = FR_ppr_inboard_inboard
                self.FR_nodes["push_pull_rod_inboard_outboard"] = FR_ppr_inboard_outboard
                self.FR_nodes["push_pull_rod_spring_outboard"] = FR_ppr_spring_outboard
                self.FR_nodes["bellcrank_pivot"] = FR_bc_pivot
                
                self.FR_links["push_pull_inboard"] = FR_inboard_rod
                self.FR_bellcranks["FR_push_pull_bellcrank"] = FR_bc

                for i, node in enumerate(FR_bc_nodes):
                    self.FR_nodes[f"bellcrank_node_{i}"] = node

            else:
                FR_ppr_outboard_rod = Link(inboard_node=FR_ppr_outboard_inboard, outboard_node=FR_ppr_outboard_outboard)
                FR_spring = Spring(inboard_node=FR_ppr_spring_inboard, outboard_node=FR_ppr_outboard_inboard,
                                    free_length=FR_ppr_params["spring"]["free_length"]["Value"], rate=FR_ppr_params["spring"]["rate"]["Value"])
                FR_ppr = PushPullRod(outboard_rod=FR_ppr_outboard_rod, spring=FR_spring)

                self.FR_nodes["push_pull_rod_spring_outboard"] = FR_ppr_outboard_inboard

            self.FR_links["push_pull_outboard"] = FR_ppr_outboard_rod
            self.Fr_springs["FR_spring"] = FR_spring
            
            self.FR_nodes["push_pull_rod_outboard_inboard"] = FR_ppr_outboard_inboard
            self.FR_nodes["push_pull_rod_outboard_outboard"] = FR_ppr_outboard_outboard
            self.FR_nodes["push_pull_rod_spring_inboard"] = FR_ppr_spring_inboard

            self.FR_quarter_car = QuarterCar(tire=self.FR_tire,
                                             lower_wishbone=FR_lower_wb,
                                             upper_wishbone=FR_upper_wb,
                                             tie_rod=FR_tie,
                                             push_pull_rod=FR_ppr)
            
            if raw_params["FL QuarterCar"]["stabar"]["connection"]["Value"]:
                Fr_stabar_params = raw_params["FL QuarterCar"]["stabar"]
                Fr_left_arm_end = Node(position=Fr_stabar_params["arm_end"]["Value"])
                Fr_right_arm_end = Node(position=Fr_stabar_params["arm_end"]["Value"]).mirrored_xz
                Fr_left_droplink_end = Node(position=Fr_stabar_params["droplink_end"]["Value"])
                Fr_right_droplink_end = Node(position=Fr_stabar_params["droplink_end"]["Value"]).mirrored_xz
                Fr_bar_left_end = Node(position=Fr_stabar_params["bar_end"]["Value"])
                Fr_bar_right_end = Node(position=Fr_stabar_params["bar_end"]["Value"]).mirrored_xz
                Fr_torsional_stiffness = Fr_stabar_params["torsional_stiffness"]["Value"]

                self.FL_nodes["stabar_arm_end"] = Fr_left_arm_end
                self.FL_nodes["stabar_droplink_end"] = Fr_left_droplink_end
                self.FL_nodes["stabar_bar_end"] = Fr_bar_left_end

                self.FR_nodes["stabar_arm_end"] = Fr_right_arm_end
                self.FR_nodes["stabar_droplink_end"] = Fr_right_droplink_end
                self.FR_nodes["stabar_bar_end"] = Fr_bar_right_end

                self.Fr_stabar = Stabar(left_arm_end=Fr_left_arm_end, 
                                        right_arm_end=Fr_right_arm_end, 
                                        left_droplink_end=Fr_left_droplink_end, 
                                        right_droplink_end=Fr_right_droplink_end, 
                                        bar_left_end=Fr_bar_left_end, 
                                        bar_right_end=Fr_bar_right_end, 
                                        torsional_stiffness=Fr_torsional_stiffness)

                self.Fr_stabar_links["torsion_bar"] = self.Fr_stabar.bar
                self.Fr_stabar_links["arm_left"] = self.Fr_stabar.left_arm
                self.Fr_stabar_links["arm_right"] = self.Fr_stabar.right_arm
                self.Fr_stabar_links["droplink_left"] = self.Fr_stabar.left_droplink
                self.Fr_stabar_links["droplink_right"] = self.Fr_stabar.right_droplink

                Fr_left_droplink_end.add_listener(self.Fr_stabar)
                Fr_right_droplink_end.add_listener(self.Fr_stabar)

                self.FL_nodes[FL_ppr_params["connection"]["Value"]].add_child(node=Fr_left_droplink_end)
                self.FR_nodes[FL_ppr_params["connection"]["Value"]].add_child(node=Fr_right_droplink_end)
            
            else:
                self.Fr_stabar = None
            
        if "RL QuarterCar" in raw_params.keys():
            
            #################
            ### RL Corner ###
            #################

            # Tire
            if "tire" in raw_params["RL QuarterCar"].keys():
                RL_tire_params = raw_params["RL QuarterCar"]["tire"]
                RL_tir_file_path = RL_tire_params["tir_path"]["Value"]
                RL_contact_patch = Node(position=RL_tire_params["contact_patch"]["Value"])
                RL_tire_od = RL_tire_params["outer_diameter"]["Value"]
                RL_tire_id = RL_tire_params["inner_diameter"]["Value"]
                RL_tire_width = RL_tire_params["width"]["Value"]
                RL_tire_toe = RL_tire_params["static_toe"]["Value"]
                RL_tire_camber = RL_tire_params["static_camber"]["Value"]

                RL_mf52 = MF52(tire_name="RL_tire", file_path=RL_tir_file_path)
                self.RL_tire = Tire(tire=RL_mf52, contact_patch=RL_contact_patch, outer_diameter=RL_tire_od, width=RL_tire_width,
                                    inner_diameter=RL_tire_id, static_toe=RL_tire_toe, static_gamma=RL_tire_camber)

                self.RL_nodes["contact_patch"] = RL_contact_patch

            else:
                raise Exception('Vehicle definition yaml must contain "tire" under "RL QuarterCar"')
            
            # Lower wishbone
            if "lower_wishbone" in raw_params["RL QuarterCar"].keys():
                RL_lower_params = raw_params["RL QuarterCar"]["lower_wishbone"]
                RL_lower_fore_inboard = Node(position=RL_lower_params["fore_inboard"]["Value"])
                RL_lower_aft_inboard = Node(position=RL_lower_params["aft_inboard"]["Value"])
                RL_lower_outboard = Node(position=RL_lower_params["outboard"]["Value"])

                RL_lower_fore = Link(inboard_node=RL_lower_fore_inboard, outboard_node=RL_lower_outboard)
                RL_lower_aft = Link(inboard_node=RL_lower_aft_inboard, outboard_node=RL_lower_outboard)

                RL_lower_wb = Wishbone(fore_link=RL_lower_fore, aft_link=RL_lower_aft)

                self.RL_nodes["lower_fore_inboard"] = RL_lower_fore_inboard
                self.RL_nodes["lower_aft_inboard"] = RL_lower_aft_inboard
                self.RL_nodes["lower_outboard"] = RL_lower_outboard

                self.RL_links["lower_fore"] = RL_lower_fore
                self.RL_links["lower_aft"] = RL_lower_aft

            else:
                raise Exception('Vehicle definition yaml must contain "lower_wishbone" under "RL QuarterCar"')

            # Upper wishbone
            if "upper_wishbone" in raw_params["RL QuarterCar"].keys():
                RL_upper_params = raw_params["RL QuarterCar"]["upper_wishbone"]
                RL_upper_fore_inboard = Node(position=RL_upper_params["fore_inboard"]["Value"])
                RL_upper_aft_inboard = Node(position=RL_upper_params["aft_inboard"]["Value"])
                RL_upper_outboard = Node(position=RL_upper_params["outboard"]["Value"])

                RL_upper_fore = Link(inboard_node=RL_upper_fore_inboard, outboard_node=RL_upper_outboard)
                RL_upper_aft = Link(inboard_node=RL_upper_aft_inboard, outboard_node=RL_upper_outboard)

                RL_upper_wb = Wishbone(fore_link=RL_upper_fore, aft_link=RL_upper_aft)

                self.RL_nodes["upper_fore_inboard"] = RL_upper_fore_inboard
                self.RL_nodes["upper_aft_inboard"] = RL_upper_aft_inboard
                self.RL_nodes["upper_outboard"] = RL_upper_outboard

                self.RL_links["upper_fore"] = RL_upper_fore
                self.RL_links["upper_aft"] = RL_upper_aft

            else:
                raise Exception('Vehicle definition yaml must contain "upper_wishbone" under "RL QuarterCar"')
            
            # Tie rod
            if "tie_rod" in raw_params["RL QuarterCar"].keys():
                RL_tie_params = raw_params["RL QuarterCar"]["tie_rod"]
                RL_tie_inboard = Node(position=RL_tie_params["inboard"]["Value"])
                RL_tie_outboard = Node(position=RL_tie_params["outboard"]["Value"])

                RL_tie = Link(inboard_node=RL_tie_inboard, outboard_node=RL_tie_outboard)

                self.RL_nodes["tie_rod_inboard"] = RL_tie_inboard
                self.RL_nodes["tie_rod_outboard"] = RL_tie_outboard

                self.RL_links["tie"] = RL_tie

            else:
                raise Exception('Vehicle definition yaml must contain "tie_rod" under "RL QuarterCar"')

            # Push/pull rod
            if "push_pull_rod" in raw_params["RL QuarterCar"].keys():
                RL_ppr_params = raw_params["RL QuarterCar"]["push_pull_rod"]
                RL_ppr_outboard_inboard = Node(position=RL_ppr_params["outboard_rod"]["inboard_node"]["Value"])
                RL_ppr_outboard_outboard = Node(position=RL_ppr_params["outboard_rod"]["outboard_node"]["Value"])
                
                RL_ppr_spring_inboard = Node(position=RL_ppr_params["spring"]["inboard_node"]["Value"])
                RL_ppr_spring_outboard = Node(position=RL_ppr_params["spring"]["outboard_node"]["Value"])

                if RL_ppr_params["connection"]["Value"] not in list(self.RL_nodes.keys()):
                        raise Exception(f"Push/pull rod connection must be one of the following: {list(self.RL_nodes.keys())}")
                else:
                    self.RL_nodes[RL_ppr_params["connection"]["Value"]].add_child(node=RL_ppr_outboard_outboard)

                if RL_ppr_params["inboard_rod"]["inboard_node"]["Value"]:
                    RL_ppr_inboard_inboard = Node(position=RL_ppr_params["inboard_rod"]["inboard_node"]["Value"])
                    RL_ppr_inboard_outboard = Node(position=RL_ppr_params["inboard_rod"]["outboard_node"]["Value"])

                    RL_bc_pivot = Node(position=RL_ppr_params["bellcrank"]["pivot"]["Value"])
                    RL_bc_direction = RL_ppr_params["bellcrank"]["pivot_direction"]["Value"]
                    RL_bc_nodes = [Node(position=pos) for pos in RL_ppr_params["bellcrank"]["nodes"]]

                    RL_ppr_outboard_rod = Link(inboard_node=RL_ppr_outboard_inboard, outboard_node=RL_ppr_outboard_outboard)
                    RL_bc = Bellcrank(*RL_bc_nodes, pivot=RL_bc_pivot, pivot_direction=RL_bc_direction)
                    RL_inboard_rod = Link(inboard_node=RL_ppr_inboard_inboard, outboard_node=RL_ppr_inboard_outboard)
                    RL_spring = Spring(inboard_node=RL_ppr_spring_inboard, outboard_node=RL_ppr_spring_outboard,
                                       free_length=RL_ppr_params["spring"]["free_length"]["Value"], rate=RL_ppr_params["spring"]["rate"]["Value"])
                    RL_ppr = PushPullRod(outboard_rod=RL_ppr_outboard_rod, spring=RL_spring, inboard_rod=RL_inboard_rod, bellcrank=RL_bc)

                    self.RL_nodes["push_pull_rod_inboard_inboard"] = RL_ppr_inboard_inboard
                    self.RL_nodes["push_pull_rod_inboard_outboard"] = RL_ppr_inboard_outboard
                    self.RL_nodes["push_pull_rod_spring_outboard"] = RL_ppr_spring_outboard
                    self.RL_nodes["bellcrank_pivot"] = RL_bc_pivot

                    self.RL_links["push_pull_inboard"] = RL_inboard_rod
                    self.RL_bellcranks["RL_push_pull_bellcrank"] = FL_bc
                    
                    for i, node in enumerate(RL_bc_nodes):
                        self.RL_nodes[f"bellcrank_node_{i}"] = node

                else:
                    RL_ppr_outboard_rod = Link(inboard_node=RL_ppr_outboard_inboard, outboard_node=RL_ppr_outboard_outboard)
                    RL_spring = Spring(inboard_node=RL_ppr_spring_inboard, outboard_node=RL_ppr_outboard_inboard,
                                       free_length=RL_ppr_params["spring"]["free_length"]["Value"], rate=RL_ppr_params["spring"]["rate"]["Value"])
                    RL_ppr = PushPullRod(outboard_rod=RL_ppr_outboard_rod, spring=RL_spring)

                    self.RL_nodes["push_pull_rod_spring_outboard"] = RL_ppr_outboard_inboard

                self.RL_links["push_pull_outboard"] = RL_ppr_outboard_rod
                self.Rr_springs["RL_spring"] = RL_spring

                self.RL_nodes["push_pull_rod_outboard_inboard"] = RL_ppr_outboard_inboard
                self.RL_nodes["push_pull_rod_outboard_outboard"] = RL_ppr_outboard_outboard
                self.RL_nodes["push_pull_rod_spring_inboard"] = RL_ppr_spring_inboard

                self.RL_quarter_car = QuarterCar(tire=self.RL_tire,
                                                 lower_wishbone=RL_lower_wb,
                                                 upper_wishbone=RL_upper_wb,
                                                 tie_rod=RL_tie,
                                                 push_pull_rod=RL_ppr)

            else:
                raise Exception('Vehicle definition yaml must contain "push_pull_rod" under "RL QuarterCar"')

            #################
            ### RR Corner ###
            #################

            # Tire
            RR_tire_params = raw_params["RL QuarterCar"]["tire"]
            RR_tir_file_path = RR_tire_params["tir_path"]["Value"]
            RR_contact_patch = Node(position=RR_tire_params["contact_patch"]["Value"]).mirrored_xz
            RR_tire_od = RR_tire_params["outer_diameter"]["Value"]
            RR_tire_id = RR_tire_params["inner_diameter"]["Value"]
            RR_tire_width = RR_tire_params["width"]["Value"]
            RR_tire_toe = -RR_tire_params["static_toe"]["Value"]
            RR_tire_camber = -RR_tire_params["static_camber"]["Value"]

            RR_mf52 = MF52(tire_name="RR_tire", file_path=RR_tir_file_path)
            self.RR_tire = Tire(tire=RR_mf52, contact_patch=RR_contact_patch, outer_diameter=RR_tire_od, width=RR_tire_width,
                                inner_diameter=RR_tire_id, static_toe=RR_tire_toe, static_gamma=RR_tire_camber)

            self.RR_nodes["contact_patch"] = RR_contact_patch
            
            # Lower wishbone
            RR_lower_params = raw_params["RL QuarterCar"]["lower_wishbone"]
            RR_lower_fore_inboard = Node(position=RR_lower_params["fore_inboard"]["Value"]).mirrored_xz
            RR_lower_aft_inboard = Node(position=RR_lower_params["aft_inboard"]["Value"]).mirrored_xz
            RR_lower_outboard = Node(position=RR_lower_params["outboard"]["Value"]).mirrored_xz

            RR_lower_fore = Link(inboard_node=RR_lower_fore_inboard, outboard_node=RR_lower_outboard)
            RR_lower_aft = Link(inboard_node=RR_lower_aft_inboard, outboard_node=RR_lower_outboard)

            RR_lower_wb = Wishbone(fore_link=RR_lower_fore, aft_link=RR_lower_aft)

            self.RR_nodes["lower_fore_inboard"] = RR_lower_fore_inboard
            self.RR_nodes["lower_aft_inboard"] = RR_lower_aft_inboard
            self.RR_nodes["lower_outboard"] = RR_lower_outboard

            self.RR_links["lower_fore"] = RR_lower_fore
            self.RR_links["lower_aft"] = RR_lower_aft

            # Upper wishbone
            RR_upper_params = raw_params["RL QuarterCar"]["upper_wishbone"]
            RR_upper_fore_inboard = Node(position=RR_upper_params["fore_inboard"]["Value"]).mirrored_xz
            RR_upper_aft_inboard = Node(position=RR_upper_params["aft_inboard"]["Value"]).mirrored_xz
            RR_upper_outboard = Node(position=RR_upper_params["outboard"]["Value"]).mirrored_xz

            RR_upper_fore = Link(inboard_node=RR_upper_fore_inboard, outboard_node=RR_upper_outboard)
            RR_upper_aft = Link(inboard_node=RR_upper_aft_inboard, outboard_node=RR_upper_outboard)

            RR_upper_wb = Wishbone(fore_link=RR_upper_fore, aft_link=RR_upper_aft)

            self.RR_nodes["upper_fore_inboard"] = RR_upper_fore_inboard
            self.RR_nodes["upper_aft_inboard"] = RR_upper_aft_inboard
            self.RR_nodes["upper_outboard"] = RR_upper_outboard

            self.RR_links["upper_fore"] = RR_upper_fore
            self.RR_links["upper_aft"] = RR_upper_aft

            # Tie rod
            RR_tie_params = raw_params["RL QuarterCar"]["tie_rod"]
            RR_tie_inboard = Node(position=RR_tie_params["inboard"]["Value"]).mirrored_xz
            RR_tie_outboard = Node(position=RR_tie_params["outboard"]["Value"]).mirrored_xz

            RR_tie = Link(inboard_node=RR_tie_inboard, outboard_node=RR_tie_outboard)

            self.RR_nodes["tie_rod_inboard"] = RR_tie_inboard
            self.RR_nodes["tie_rod_outboard"] = RR_tie_outboard

            self.RR_links["tie"] = RR_tie

            # Push/pull rod
            RR_ppr_params = raw_params["RL QuarterCar"]["push_pull_rod"]
            RR_ppr_outboard_inboard = Node(position=RR_ppr_params["outboard_rod"]["inboard_node"]["Value"]).mirrored_xz
            RR_ppr_outboard_outboard = Node(position=RR_ppr_params["outboard_rod"]["outboard_node"]["Value"]).mirrored_xz
            
            RR_ppr_spring_inboard = Node(position=RR_ppr_params["spring"]["inboard_node"]["Value"]).mirrored_xz
            RR_ppr_spring_outboard = Node(position=RR_ppr_params["spring"]["outboard_node"]["Value"]).mirrored_xz

            if RR_ppr_params["connection"]["Value"] not in list(self.RR_nodes.keys()):
                    raise Exception(f"Push/pull rod connection must be one of the following: {list(self.RR_nodes.keys())}")
            else:
                self.RR_nodes[RR_ppr_params["connection"]["Value"]].add_child(node=RR_ppr_outboard_outboard)

            if RR_ppr_params["inboard_rod"]["inboard_node"]["Value"]:
                RR_ppr_inboard_inboard = Node(position=RR_ppr_params["inboard_rod"]["inboard_node"]["Value"]).mirrored_xz
                RR_ppr_inboard_outboard = Node(position=RR_ppr_params["inboard_rod"]["outboard_node"]["Value"]).mirrored_xz

                RR_bc_pivot = Node(position=RR_ppr_params["bellcrank"]["pivot"]["Value"]).mirrored_xz
                RR_bc_direction = Node(position=RR_ppr_params["bellcrank"]["pivot_direction"]["Value"]).mirrored_xz
                RR_bc_nodes = [Node(position=pos).mirrored_xz for pos in RR_ppr_params["bellcrank"]["nodes"]]

                RR_ppr_outboard_rod = Link(inboard_node=RR_ppr_outboard_inboard, outboard_node=RR_ppr_outboard_outboard)
                RR_bc = Bellcrank(*RR_bc_nodes, pivot=RR_bc_pivot, pivot_direction=RR_bc_direction)
                RR_inboard_rod = Link(inboard_node=RR_ppr_inboard_inboard, outboard_node=RR_ppr_inboard_outboard)
                RR_spring = Spring(inboard_node=RR_ppr_spring_inboard, outboard_node=RR_ppr_spring_outboard,
                                    free_length=RR_ppr_params["spring"]["free_length"]["Value"], rate=RR_ppr_params["spring"]["rate"]["Value"])
                RR_ppr = PushPullRod(outboard_rod=RR_ppr_outboard_rod, spring=RR_spring, inboard_rod=RR_inboard_rod, bellcrank=RR_bc)

                self.RR_nodes["push_pull_rod_inboard_inboard"] = RR_ppr_inboard_inboard
                self.RR_nodes["push_pull_rod_inboard_outboard"] = RR_ppr_inboard_outboard
                self.RR_nodes["push_pull_rod_spring_outboard"] = RR_ppr_spring_outboard
                self.RR_nodes["bellcrank_pivot"] = RR_bc_pivot
                
                self.RR_links["push_pull_inboard"] = RR_inboard_rod
                self.RR_bellcranks["RR_push_pull_bellcrank"] = RR_bc

                for i, node in enumerate(RR_bc_nodes):
                    self.RR_nodes[f"bellcrank_node_{i}"] = node

            else:
                RR_ppr_outboard_rod = Link(inboard_node=RR_ppr_outboard_inboard, outboard_node=RR_ppr_outboard_outboard)
                RR_spring = Spring(inboard_node=RR_ppr_spring_inboard, outboard_node=RR_ppr_outboard_inboard,
                                    free_length=RR_ppr_params["spring"]["free_length"]["Value"], rate=RR_ppr_params["spring"]["rate"]["Value"])
                RR_ppr = PushPullRod(outboard_rod=RR_ppr_outboard_rod, spring=RR_spring)

                self.RR_nodes["push_pull_rod_spring_outboard"] = RR_ppr_outboard_inboard

            self.RR_links["push_pull_outboard"] = RR_ppr_outboard_rod
            self.Rr_springs["RR_spring"] = RR_spring

            self.RR_nodes["push_pull_rod_outboard_inboard"] = RR_ppr_outboard_inboard
            self.RR_nodes["push_pull_rod_outboard_outboard"] = RR_ppr_outboard_outboard
            self.RR_nodes["push_pull_rod_spring_inboard"] = RR_ppr_spring_inboard

            self.RR_quarter_car = QuarterCar(tire=self.RR_tire,
                                             lower_wishbone=RR_lower_wb,
                                             upper_wishbone=RR_upper_wb,
                                             tie_rod=RR_tie,
                                             push_pull_rod=RR_ppr)
            
            if raw_params["RL QuarterCar"]["stabar"]["connection"]["Value"]:
                Rr_stabar_params = raw_params["RL QuarterCar"]["stabar"]
                Rr_left_arm_end = Node(position=Rr_stabar_params["arm_end"]["Value"])
                Rr_right_arm_end = Node(position=Rr_stabar_params["arm_end"]["Value"]).mirrored_xz
                Rr_left_droplink_end = Node(position=Rr_stabar_params["droplink_end"]["Value"])
                Rr_right_droplink_end = Node(position=Rr_stabar_params["droplink_end"]["Value"]).mirrored_xz
                Rr_bar_left_end = Node(position=Rr_stabar_params["bar_end"]["Value"])
                Rr_bar_right_end = Node(position=Rr_stabar_params["bar_end"]["Value"]).mirrored_xz
                Rr_torsional_stiffness = Rr_stabar_params["torsional_stiffness"]["Value"]

                self.RL_nodes["stabar_arm_end"] = Rr_left_arm_end
                self.RL_nodes["stabar_droplink_end"] = Rr_left_droplink_end
                self.RL_nodes["stabar_bar_end"] = Rr_bar_left_end

                self.RR_nodes["stabar_arm_end"] = Rr_right_arm_end
                self.RR_nodes["stabar_droplink_end"] = Rr_right_droplink_end
                self.RR_nodes["stabar_bar_end"] = Rr_bar_right_end

                self.Rr_stabar = Stabar(left_arm_end=Rr_left_arm_end, 
                                        right_arm_end=Rr_right_arm_end, 
                                        left_droplink_end=Rr_left_droplink_end, 
                                        right_droplink_end=Rr_right_droplink_end, 
                                        bar_left_end=Rr_bar_left_end, 
                                        bar_right_end=Rr_bar_right_end, 
                                        torsional_stiffness=Rr_torsional_stiffness)
                
                self.Rr_stabar_links["torsion_bar"] = self.Rr_stabar.bar
                self.Rr_stabar_links["arm_left"] = self.Rr_stabar.left_arm
                self.Rr_stabar_links["arm_right"] = self.Rr_stabar.right_arm
                self.Rr_stabar_links["droplink_left"] = self.Rr_stabar.left_droplink
                self.Rr_stabar_links["droplink_right"] = self.Rr_stabar.right_droplink

                Rr_left_droplink_end.add_listener(self.Rr_stabar)
                Rr_right_droplink_end.add_listener(self.Rr_stabar)

                self.RL_nodes[FL_ppr_params["connection"]["Value"]].add_child(node=Rr_left_droplink_end)
                self.RR_nodes[FL_ppr_params["connection"]["Value"]].add_child(node=Rr_right_droplink_end)
            
            else:
                self.Rr_stabar = None

        else:
            raise Exception('Vehicle definition yaml must contain "RL QuarterCar"')