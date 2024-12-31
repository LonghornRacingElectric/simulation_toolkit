from vehicle_model.suspension_model.suspension_elements.quaternary_elements.push_pull_rod import PushPullRod
from vehicle_model.suspension_model.suspension_elements.quinary_elements.quarter_car import QuarterCar
from vehicle_model.suspension_model.suspension_elements.secondary_elements.wishbone import Wishbone
from vehicle_model.suspension_model.suspension_elements.secondary_elements.spring import Spring
from vehicle_model.suspension_model.suspension_elements.secondary_elements.tire import Tire
from vehicle_model.suspension_model.suspension_elements.primary_elements.link import Link
from vehicle_model.suspension_model.suspension_elements.primary_elements.node import Node
from LHR_tire_toolkit.MF52 import MF52 # type: ignore

from unittest import main, TestCase
import numpy as np


class TestQuarterCar(TestCase):
    # Nodes
    lower_inboard_fore = Node(position=[2, 0, 0.5])
    lower_inboard_aft = Node(position=[0, 0, 0.5])
    lower_outboard = Node(position=[1, 2, 0.5])

    upper_inboard_fore = Node(position=[2, 0, 1.5])
    upper_inboard_aft = Node(position=[0, 0, 1.5])
    upper_outboard = Node(position=[1, 2, 1.5])

    tie_inboard = Node(position=[2, 0, 0.5])
    tie_outboard = Node(position=[2, 2, 1])

    pushrod_lower = Node(position=[1, 2, 1.5])
    shared_node = Node(position=[1, 1, 2])
    spring_upper = Node(position=[1, 0, 2.5])

    contact_patch = Node(position=[1, 2.5, 0])

    # Links
    lower_fore_link = Link(inboard_node=lower_inboard_fore, outboard_node=lower_outboard)
    lower_aft_link = Link(inboard_node=lower_inboard_aft, outboard_node=lower_outboard)

    upper_fore_link = Link(inboard_node=upper_inboard_fore, outboard_node=upper_outboard)
    upper_aft_link = Link(inboard_node=upper_inboard_aft, outboard_node=upper_outboard)

    tie_rod = Link(inboard_node=tie_inboard, outboard_node=tie_outboard)

    # Pushrod -> Spring -> Frame
    pushrod = Link(inboard_node=shared_node, outboard_node=pushrod_lower)
    spring = Spring(inboard_node=spring_upper, outboard_node=shared_node, free_length=1, rate=1)
    push_pull_rod = PushPullRod(outboard_rod=pushrod, spring=spring)

    # Wishbones
    lower_wishbone = Wishbone(fore_link=lower_fore_link, aft_link=lower_aft_link)
    upper_wishbone = Wishbone(fore_link=upper_fore_link, aft_link=upper_aft_link)

    # Tire
    tire_mf52 = MF52(tire_name="test_tire", file_path="./1_model_inputs/Modified_Round_8_Hoosier_R25B_16x7p5_10_on_7in_12psi_PAC02_UM2.tir")
    tire = Tire(tire=tire_mf52, contact_patch=contact_patch, outer_diameter=16*0.0254, width=7*0.0254, inner_diameter=10*0.0254)

    # Set relation between upper wishbone and pushrod
    upper_outboard.add_child(node=pushrod_lower)

    quarter_car = QuarterCar(tire=tire,
                                lower_wishbone=lower_wishbone,
                                upper_wishbone=upper_wishbone,
                                tie_rod=tie_rod,
                                push_pull_rod=pushrod)
    
    def test_jounce_resid(self):
        # Known outputs for jounce of 0.25 units
        wishbone_angles = 7.18076 * np.pi / 180
        wheel_angle = -3.67105 * np.pi / 180

        # Manually set jounce
        self.quarter_car.wheel_jounce = 0.25

        # Calculate residuals and confirm residuals go to zero
        residuals = self.quarter_car._geometry_resid_func(x=[wishbone_angles, wishbone_angles, wheel_angle])

        rounded_residuals = [round(x, 6) for x in residuals]

        self.assertListEqual(rounded_residuals, [0, 0, 0])

    def test_update_geometry_one(self):
        self.quarter_car.jounce(jounce=0.25)
        self.quarter_car.steer(rack_displacement=0)

        known_results = [-3.67, 7.18, 7.18, 1.0320141, 2.4832875, 0.25, 1.9979481, 1.9202853, 1.25, 1.0032864]
        test_results = [round(self.tire.steered_angle * 180 / np.pi, 2),
                        round(self.lower_wishbone.angle * 180 / np.pi, 2),
                        round(self.upper_wishbone.angle * 180 / np.pi, 2),
                        round(self.tire.contact_patch[0], 7),
                        round(self.tire.contact_patch[1], 7),
                        round(self.tire.contact_patch[2], 7),
                        round(self.tie_rod.outboard_node[0], 7),
                        round(self.tie_rod.outboard_node[1], 7),
                        round(self.tie_rod.outboard_node[2], 7),
                        round(self.spring.length, 7)]

        corresponding_values = ["wheel_angle",
                                "lower_wishbone_angle",
                                "upper_wishbone_angle",
                                "contact_patch_x",
                                "contact_patch_y",
                                "contact_patch_z",
                                "tie_rod_x",
                                "tie_rod_y",
                                "tie_rod_z",
                                "spring_length"]

        tolerances = [0.01 * np.pi / 180,
                      0.01 * np.pi / 180,
                      0.01 * np.pi / 180,
                      0.001 * 0.0254,
                      0.001 * 0.0254,
                      0.001 * 0.0254,
                      0.001 * 0.0254,
                      0.001 * 0.0254,
                      0.001 * 0.0254,
                      0.001 * 0.0254]

        for i in range(len(known_results)):
            with self.subTest(i=i):
                self.assertLess(abs(test_results[i] - known_results[i]), tolerances[i], msg=corresponding_values[i])
        
    def test_update_geometry_two(self):
        self.quarter_car.jounce(jounce=-0.5)
        self.quarter_car.steer(rack_displacement=-0.25)

        known_results = [-7.18, -14.48, -14.48, 1.0624769, 2.4325730, -0.5, 1.9921626, 1.8115379, 0.5, 1.3314558]
        test_results = [round(self.tire.steered_angle * 180 / np.pi, 2),
                        round(self.lower_wishbone.angle * 180 / np.pi, 2),
                        round(self.upper_wishbone.angle * 180 / np.pi, 2),
                        round(self.tire.contact_patch[0], 7),
                        round(self.tire.contact_patch[1], 7),
                        round(self.tire.contact_patch[2], 7),
                        round(self.tie_rod.outboard_node[0], 7),
                        round(self.tie_rod.outboard_node[1], 7),
                        round(self.tie_rod.outboard_node[2], 7),
                        round(self.spring.length, 7)]

        corresponding_values = ["wheel_angle",
                                "lower_wishbone_angle",
                                "upper_wishbone_angle",
                                "contact_patch_x",
                                "contact_patch_y",
                                "contact_patch_z",
                                "tie_rod_x",
                                "tie_rod_y",
                                "tie_rod_z",
                                "spring_length"]

        tolerances = [0.01 * np.pi / 180,
                      0.01 * np.pi / 180,
                      0.01 * np.pi / 180,
                      0.001 * 0.0254,
                      0.001 * 0.0254,
                      0.001 * 0.0254,
                      0.001 * 0.0254,
                      0.001 * 0.0254,
                      0.001 * 0.0254,
                      0.001 * 0.0254]

        for i in range(len(known_results)):
            with self.subTest(i=i):
                self.assertLess(abs(test_results[i] - known_results[i]), tolerances[i], msg=corresponding_values[i])
    
    def test_update_geometry_three(self):
        self.quarter_car.jounce(jounce=0.5)
        self.quarter_car.steer(rack_displacement=-0.15)

        known_results = [-16.51, 14.48, 14.48, 1.1420938, 2.4158760, 0.5, 1.9587687, 1.6523041, 1.5, 0.8819660]
        test_results = [round(self.tire.steered_angle * 180 / np.pi, 2),
                        round(self.lower_wishbone.angle * 180 / np.pi, 2),
                        round(self.upper_wishbone.angle * 180 / np.pi, 2),
                        round(self.tire.contact_patch[0], 7),
                        round(self.tire.contact_patch[1], 7),
                        round(self.tire.contact_patch[2], 7),
                        round(self.tie_rod.outboard_node[0], 7),
                        round(self.tie_rod.outboard_node[1], 7),
                        round(self.tie_rod.outboard_node[2], 7),
                        round(self.spring.length, 7)]

        corresponding_values = ["wheel_angle",
                                "lower_wishbone_angle",
                                "upper_wishbone_angle",
                                "contact_patch_x",
                                "contact_patch_y",
                                "contact_patch_z",
                                "tie_rod_x",
                                "tie_rod_y",
                                "tie_rod_z",
                                "spring_length"]

        tolerances = [0.01 * np.pi / 180,
                      0.01 * np.pi / 180,
                      0.01 * np.pi / 180,
                      0.001 * 0.0254,
                      0.001 * 0.0254,
                      0.001 * 0.0254,
                      0.001 * 0.0254,
                      0.001 * 0.0254,
                      0.001 * 0.0254,
                      0.001 * 0.0254]

        for i in range(len(known_results)):
            with self.subTest(i=i):
                self.assertLess(abs(test_results[i] - known_results[i]), tolerances[i], msg=corresponding_values[i])
    
    def test_update_geometry_four(self):
        self.quarter_car.jounce(jounce=-0.3)
        self.quarter_car.steer(rack_displacement=-0.05)

        known_results = [1.40, -8.63, -8.63, 0.9877718, 2.4772224, -0.3, 1.9997009, 2.0018284, 0.7, 1.2483979]
        test_results = [round(self.tire.steered_angle * 180 / np.pi, 2),
                        round(self.lower_wishbone.angle * 180 / np.pi, 2),
                        round(self.upper_wishbone.angle * 180 / np.pi, 2),
                        round(self.tire.contact_patch[0], 7),
                        round(self.tire.contact_patch[1], 7),
                        round(self.tire.contact_patch[2], 7),
                        round(self.tie_rod.outboard_node[0], 7),
                        round(self.tie_rod.outboard_node[1], 7),
                        round(self.tie_rod.outboard_node[2], 7),
                        round(self.spring.length, 7)]

        corresponding_values = ["wheel_angle",
                                "lower_wishbone_angle",
                                "upper_wishbone_angle",
                                "contact_patch_x",
                                "contact_patch_y",
                                "contact_patch_z",
                                "tie_rod_x",
                                "tie_rod_y",
                                "tie_rod_z",
                                "spring_length"]

        tolerances = [0.01 * np.pi / 180,
                      0.01 * np.pi / 180,
                      0.01 * np.pi / 180,
                      0.001 * 0.0254,
                      0.001 * 0.0254,
                      0.001 * 0.0254,
                      0.001 * 0.0254,
                      0.001 * 0.0254,
                      0.001 * 0.0254,
                      0.001 * 0.0254]

        for i in range(len(known_results)):
            with self.subTest(i=i):
                self.assertLess(abs(test_results[i] - known_results[i]), tolerances[i], msg=corresponding_values[i])

    def test_update_geometry_five(self):
        self.quarter_car.jounce(jounce=-0.3)
        self.quarter_car.steer(rack_displacement=0.15)

        known_results = [12.96, -8.63, -8.63, 0.887851, 2.464632, -0.3, 1.974521, 2.201670, 0.7, 1.248398]
        test_results = [round(self.tire.steered_angle * 180 / np.pi, 2),
                        round(self.lower_wishbone.angle * 180 / np.pi, 2),
                        round(self.upper_wishbone.angle * 180 / np.pi, 2),
                        round(self.tire.contact_patch[0], 6),
                        round(self.tire.contact_patch[1], 6),
                        round(self.tire.contact_patch[2], 6),
                        round(self.tie_rod.outboard_node[0], 6),
                        round(self.tie_rod.outboard_node[1], 6),
                        round(self.tie_rod.outboard_node[2], 6),
                        round(self.spring.length, 6)]

        corresponding_values = ["wheel_angle",
                                "lower_wishbone_angle",
                                "upper_wishbone_angle",
                                "contact_patch_x",
                                "contact_patch_y",
                                "contact_patch_z",
                                "tie_rod_x",
                                "tie_rod_y",
                                "tie_rod_z",
                                "spring_length"]

        tolerances = [0.01 * np.pi / 180,
                      0.01 * np.pi / 180,
                      0.01 * np.pi / 180,
                      0.001 * 0.0254,
                      0.001 * 0.0254,
                      0.001 * 0.0254,
                      0.001 * 0.0254,
                      0.001 * 0.0254,
                      0.001 * 0.0254,
                      0.001 * 0.0254]

        for i in range(len(known_results)):
            with self.subTest(i=i):
                self.assertLess(abs(test_results[i] - known_results[i]), tolerances[i], msg=corresponding_values[i])
        
    def test_update_geometry_six(self):
        self.quarter_car.jounce(jounce=-0.1)
        self.quarter_car.steer(rack_displacement=0.25)

        known_results = [15.93, -2.87, -2.87, 0.862744, 2.478290, -0.1, 1.961584, 2.272010, 0.9, 1.162317]
        test_results = [round(self.tire.steered_angle * 180 / np.pi, 2),
                        round(self.lower_wishbone.angle * 180 / np.pi, 2),
                        round(self.upper_wishbone.angle * 180 / np.pi, 2),
                        round(self.tire.contact_patch[0], 6),
                        round(self.tire.contact_patch[1], 6),
                        round(self.tire.contact_patch[2], 6),
                        round(self.tie_rod.outboard_node[0], 6),
                        round(self.tie_rod.outboard_node[1], 6),
                        round(self.tie_rod.outboard_node[2], 6),
                        round(self.spring.length, 6)]

        corresponding_values = ["wheel_angle",
                                "lower_wishbone_angle",
                                "upper_wishbone_angle",
                                "contact_patch_x",
                                "contact_patch_y",
                                "contact_patch_z",
                                "tie_rod_x",
                                "tie_rod_y",
                                "tie_rod_z",
                                "spring_length"]

        tolerances = [0.01 * np.pi / 180,
                      0.01 * np.pi / 180,
                      0.01 * np.pi / 180,
                      0.001 * 0.0254,
                      0.001 * 0.0254,
                      0.001 * 0.0254,
                      0.001 * 0.0254,
                      0.001 * 0.0254,
                      0.001 * 0.0254,
                      0.001 * 0.0254]

        for i in range(len(known_results)):
            with self.subTest(i=i):
                self.assertLess(abs(test_results[i] - known_results[i]), tolerances[i], msg=corresponding_values[i])