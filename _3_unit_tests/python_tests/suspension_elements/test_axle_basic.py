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

from unittest import TestCase
import numpy as np

# Basic axle test uses control arms configured as a parallelogram

class TestAxleBasic(TestCase):
    ### Left corner
    # Nodes
    lower_inboard_fore_L = Node(position=[2, 2, 0])
    lower_inboard_aft_L = Node(position=[0, 2, 0])
    lower_outboard_L = Node(position=[1, 4, 0])

    upper_inboard_fore_L = Node(position=[2, 2, 1])
    upper_inboard_aft_L = Node(position=[0, 2, 1])
    upper_outboard_L = Node(position=[1, 4, 1])

    tie_inboard_L = Node(position=[2, 2, 0])
    tie_outboard_L = Node(position=[2, 4, 0.5])

    bellcrank_pivot_L = Node(position=[1, 2, 2])
    bellcrank_direction_L = [1, 0, 0]

    pushrod_lower_L = Node(position=[1, 4, 1])
    pushrod_upper_L = Node(position=[1, 2.5, 2])
    inboard_rod_outboard_L = Node(position=[1, 2, 2.5])
    rod_to_spring_L = Node(position=[1, 1, 2.5])
    spring_inboard_L = Node(position=[1, 0, 2.5])

    contact_patch_L = Node(position=[1, 4.5, -0.5])

    # Links
    lower_fore_link_L = Link(inboard_node=lower_inboard_fore_L, outboard_node=lower_outboard_L)
    lower_aft_link_L = Link(inboard_node=lower_inboard_aft_L, outboard_node=lower_outboard_L)

    upper_fore_link_L = Link(inboard_node=upper_inboard_fore_L, outboard_node=upper_outboard_L)
    upper_aft_link_L = Link(inboard_node=upper_inboard_aft_L, outboard_node=upper_outboard_L)

    tie_rod_L = Link(inboard_node=tie_inboard_L, outboard_node=tie_outboard_L)

    # Pushrod -> Bellcrank -> Inboard Rod -> Spring -> Frame
    pushrod_L = Link(inboard_node=pushrod_upper_L, outboard_node=pushrod_lower_L)
    bellcrank_L = Bellcrank(pushrod_upper_L, inboard_rod_outboard_L, pivot=bellcrank_pivot_L, pivot_direction=bellcrank_direction_L)
    inboard_rod_L = Link(inboard_node=rod_to_spring_L, outboard_node=inboard_rod_outboard_L)
    spring_L = Spring(inboard_node=spring_inboard_L, outboard_node=rod_to_spring_L, free_length=1, rate=1)

    push_pull_rod_L = PushPullRod(outboard_rod=pushrod_L, spring=spring_L, inboard_rod=inboard_rod_L, bellcrank=bellcrank_L)

    # Wishbones
    lower_wishbone_L = Wishbone(fore_link=lower_fore_link_L, aft_link=lower_aft_link_L)
    upper_wishbone_L = Wishbone(fore_link=upper_fore_link_L, aft_link=upper_aft_link_L)
    
    # Tire
    tire_mf52_L = MF52(tire_name="test_tire", file_path="./_3_unit_tests/python_tests/_test_dependencies/unit_test_tire.tir")
    tire_L = Tire(tire=tire_mf52_L, contact_patch=contact_patch_L, outer_diameter=16*0.0254, width=7*0.0254, inner_diameter=10*0.0254)

    # Set relation between upper wishbone and pushrod
    upper_outboard_L.add_child(node=pushrod_lower_L)

    quarter_car_L = QuarterCar(tire=tire_L,
                                lower_wishbone=lower_wishbone_L,
                                upper_wishbone=upper_wishbone_L,
                                tie_rod=tie_rod_L,
                                push_pull_rod=pushrod_L)
    
    ### Right corner
    lower_inboard_fore_R = Node(position=[2, -2, 0])
    lower_inboard_aft_R = Node(position=[0, -2, 0])
    lower_outboard_R = Node(position=[1, -4, 0])

    upper_inboard_fore_R = Node(position=[2, -2, 1])
    upper_inboard_aft_R = Node(position=[0, -2, 1])
    upper_outboard_R = Node(position=[1, -4, 1])

    tie_inboard_R = Node(position=[2, -2, 0])
    tie_outboard_R = Node(position=[2, -4, 0.5])

    bellcrank_pivot_R = Node(position=[1, -2, 2])
    bellcrank_direction_R = [1, 0, 0]

    pushrod_lower_R = Node(position=[1, -4, 1])
    pushrod_upper_R = Node(position=[1, -2.5, 2])
    inboard_rod_outboard_R = Node(position=[1, -2, 2.5])
    rod_to_spring_R = Node(position=[1, -1, 2.5])
    spring_inboard_R = Node(position=[1, 0, 2.5])

    contact_patch_R = Node(position=[1, -4.5, -0.5])

    # Links
    lower_fore_link_R = Link(inboard_node=lower_inboard_fore_R, outboard_node=lower_outboard_R)
    lower_aft_link_R = Link(inboard_node=lower_inboard_aft_R, outboard_node=lower_outboard_R)

    upper_fore_link_R = Link(inboard_node=upper_inboard_fore_R, outboard_node=upper_outboard_R)
    upper_aft_link_R = Link(inboard_node=upper_inboard_aft_R, outboard_node=upper_outboard_R)

    tie_rod_R = Link(inboard_node=tie_inboard_R, outboard_node=tie_outboard_R)

    # Pushrod -> Bellcrank -> Inboard Rod -> Spring -> Frame
    pushrod_R = Link(inboard_node=pushrod_upper_R, outboard_node=pushrod_lower_R)
    bellcrank_R = Bellcrank(pushrod_upper_R, inboard_rod_outboard_R, pivot=bellcrank_pivot_R, pivot_direction=bellcrank_direction_R)
    inboard_rod_R = Link(inboard_node=rod_to_spring_R, outboard_node=inboard_rod_outboard_R)
    spring_R = Spring(inboard_node=spring_inboard_R, outboard_node=rod_to_spring_R, free_length=1, rate=1)

    push_pull_rod_R = PushPullRod(outboard_rod=pushrod_R, spring=spring_R, inboard_rod=inboard_rod_R, bellcrank=bellcrank_R)

    # Wishbones
    lower_wishbone_R = Wishbone(fore_link=lower_fore_link_R, aft_link=lower_aft_link_R)
    upper_wishbone_R = Wishbone(fore_link=upper_fore_link_R, aft_link=upper_aft_link_R)
    
    # Tire
    tire_mf52_R = MF52(tire_name="test_tire", file_path="./_3_unit_tests/python_tests/_test_dependencies/unit_test_tire.tir")
    tire_R = Tire(tire=tire_mf52_R, contact_patch=contact_patch_R, outer_diameter=16*0.0254, width=7*0.0254, inner_diameter=10*0.0254)

    # Set relation between upper wishbone and pushrod
    upper_outboard_R.add_child(node=pushrod_lower_R)

    quarter_car_R = QuarterCar(tire=tire_R,
                                lower_wishbone=lower_wishbone_R,
                                upper_wishbone=upper_wishbone_R,
                                tie_rod=tie_rod_R,
                                push_pull_rod=pushrod_R)

    ### Stabar
    left_arm_end = Node(position=[1.93301270, 2.5, 1.75])
    right_arm_end = Node(position=[1.93301270, -2.5, 1.75])
    left_droplink_end = Node(position=[1.5, 3, 1])
    right_droplink_end = Node(position=[1.5, -3, 1])
    bar_left_end = Node(position=[1.5, 2.5, 2])
    bar_right_end = Node(position=[1.5, -2.5, 2])
    torsional_stiffness = 1 # Nm/rad

    stabar = Stabar(left_arm_end=left_arm_end, 
                    right_arm_end=right_arm_end, 
                    left_droplink_end=left_droplink_end, 
                    right_droplink_end=right_droplink_end, 
                    bar_left_end=bar_left_end, 
                    bar_right_end=bar_right_end, 
                    torsional_stiffness=torsional_stiffness)
    
    left_droplink_end.add_listener(stabar)
    right_droplink_end.add_listener(stabar)

    # Couple stabar to upper wishbone
    upper_outboard_L.add_child(node=left_droplink_end)
    upper_outboard_R.add_child(node=right_droplink_end)
    
    def test_jounce_resid_left(self):
        # Known outputs for jounce of 0.25 units
        wishbone_angles = 7.18076 * np.pi / 180
        wheel_angle = -3.67105 * np.pi / 180

        # Manually set jounce
        self.quarter_car_L.wheel_jounce = self.contact_patch_L.initial_position[2] + 0.25

        # Calculate residuals and confirm residuals go to zero
        residuals = self.quarter_car_L._geometry_resid_func(x=[wishbone_angles, wishbone_angles, wheel_angle])

        rounded_residuals = [round(x, 2) for x in residuals]

        self.assertListEqual(rounded_residuals, [0, 0, 0])

    def test_update_geometry_one_left(self):
        self.quarter_car_L.jounce(jounce=0.25)
        self.quarter_car_L.steer(rack_displacement=0)

        known_results = [-3.67, 7.18, 7.18, 1.032014, 4.483288, -0.25, 1.997948, 3.920285, 0.75, 0.794828]
        test_results = [round(self.tire_L.steered_angle * 180 / np.pi, 2),
                        round(self.lower_wishbone_L.angle * 180 / np.pi, 2),
                        round(self.upper_wishbone_L.angle * 180 / np.pi, 2),
                        round(self.tire_L.contact_patch[0], 6),
                        round(self.tire_L.contact_patch[1], 6),
                        round(self.tire_L.contact_patch[2], 6),
                        round(self.tie_rod_L.outboard_node[0], 6),
                        round(self.tie_rod_L.outboard_node[1], 6),
                        round(self.tie_rod_L.outboard_node[2], 6),
                        round(self.spring_L.length, 6)]

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
            
    def test_update_geometry_one_left_repeated(self):
        self.quarter_car_L.jounce(jounce=0.125)
        self.quarter_car_L.steer(rack_displacement=0.1)
        self.quarter_car_L.jounce(jounce=0.25)
        self.quarter_car_L.steer(rack_displacement=0)

        known_results = [-3.67, 7.18, 7.18, 1.032014, 4.483288, -0.25, 1.997948, 3.920285, 0.75, 0.794828]
        test_results = [round(self.tire_L.steered_angle * 180 / np.pi, 2),
                        round(self.lower_wishbone_L.angle * 180 / np.pi, 2),
                        round(self.upper_wishbone_L.angle * 180 / np.pi, 2),
                        round(self.tire_L.contact_patch[0], 6),
                        round(self.tire_L.contact_patch[1], 6),
                        round(self.tire_L.contact_patch[2], 6),
                        round(self.tie_rod_L.outboard_node[0], 6),
                        round(self.tie_rod_L.outboard_node[1], 6),
                        round(self.tie_rod_L.outboard_node[2], 6),
                        round(self.spring_L.length, 6)]

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
    
    def test_update_geometry_two_left(self):
        self.quarter_car_L.jounce(jounce=0.5)
        self.quarter_car_L.steer(rack_displacement=-0.15)

        known_results = [-16.51, 14.48, 14.48, 1.142094, 4.415876, 0.0, 1.958769, 3.652304, 1.0, 0.650041]
        test_results = [round(self.tire_L.steered_angle * 180 / np.pi, 2),
                        round(self.lower_wishbone_L.angle * 180 / np.pi, 2),
                        round(self.upper_wishbone_L.angle * 180 / np.pi, 2),
                        round(self.tire_L.contact_patch[0], 6),
                        round(self.tire_L.contact_patch[1], 6),
                        round(self.tire_L.contact_patch[2], 6),
                        round(self.tie_rod_L.outboard_node[0], 6),
                        round(self.tie_rod_L.outboard_node[1], 6),
                        round(self.tie_rod_L.outboard_node[2], 6),
                        round(self.spring_L.length, 6)]

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
    
    def test_update_geometry_two_left_repeated(self):
        self.quarter_car_L.jounce(jounce=0.125)
        self.quarter_car_L.steer(rack_displacement=0.1)
        self.quarter_car_L.jounce(jounce=0.5)
        self.quarter_car_L.steer(rack_displacement=-0.15)

        known_results = [-16.51, 14.48, 14.48, 1.142094, 4.415876, 0.0, 1.958769, 3.652304, 1.0, 0.650041]
        test_results = [round(self.tire_L.steered_angle * 180 / np.pi, 2),
                        round(self.lower_wishbone_L.angle * 180 / np.pi, 2),
                        round(self.upper_wishbone_L.angle * 180 / np.pi, 2),
                        round(self.tire_L.contact_patch[0], 6),
                        round(self.tire_L.contact_patch[1], 6),
                        round(self.tire_L.contact_patch[2], 6),
                        round(self.tie_rod_L.outboard_node[0], 6),
                        round(self.tie_rod_L.outboard_node[1], 6),
                        round(self.tie_rod_L.outboard_node[2], 6),
                        round(self.spring_L.length, 6)]

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

    def test_update_geometry_three_left(self):
        self.quarter_car_L.jounce(jounce=-0.1)
        self.quarter_car_L.steer(rack_displacement=0.25)

        known_results = [15.93, -2.87, -2.87, 0.862744, 4.478290, -0.6, 1.961584, 4.272010, 0.4, 1.117498]
        test_results = [round(self.tire_L.steered_angle * 180 / np.pi, 2),
                        round(self.lower_wishbone_L.angle * 180 / np.pi, 2),
                        round(self.upper_wishbone_L.angle * 180 / np.pi, 2),
                        round(self.tire_L.contact_patch[0], 6),
                        round(self.tire_L.contact_patch[1], 6),
                        round(self.tire_L.contact_patch[2], 6),
                        round(self.tie_rod_L.outboard_node[0], 6),
                        round(self.tie_rod_L.outboard_node[1], 6),
                        round(self.tie_rod_L.outboard_node[2], 6),
                        round(self.spring_L.length, 6)]

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
    
    def test_update_geometry_three_left_repeated(self):
        self.quarter_car_L.jounce(jounce=0.125)
        self.quarter_car_L.steer(rack_displacement=0.1)
        self.quarter_car_L.jounce(jounce=-0.1)
        self.quarter_car_L.steer(rack_displacement=0.25)

        known_results = [15.93, -2.87, -2.87, 0.862744, 4.478290, -0.6, 1.961584, 4.272010, 0.4, 1.117498]
        test_results = [round(self.tire_L.steered_angle * 180 / np.pi, 2),
                        round(self.lower_wishbone_L.angle * 180 / np.pi, 2),
                        round(self.upper_wishbone_L.angle * 180 / np.pi, 2),
                        round(self.tire_L.contact_patch[0], 6),
                        round(self.tire_L.contact_patch[1], 6),
                        round(self.tire_L.contact_patch[2], 6),
                        round(self.tie_rod_L.outboard_node[0], 6),
                        round(self.tie_rod_L.outboard_node[1], 6),
                        round(self.tie_rod_L.outboard_node[2], 6),
                        round(self.spring_L.length, 6)]

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

    def test_jounce_resid_right(self):
        # Known outputs for jounce of 0.25 units
        wishbone_angles = -7.18076 * np.pi / 180
        wheel_angle = 3.67105 * np.pi / 180

        # Manually set jounce
        self.quarter_car_R.wheel_jounce = self.contact_patch_L.initial_position[2] + 0.25

        # Calculate residuals and confirm residuals go to zero
        residuals = self.quarter_car_R._geometry_resid_func(x=[wishbone_angles, wishbone_angles, wheel_angle])

        rounded_residuals = [round(x, 2) for x in residuals]

        self.assertListEqual(rounded_residuals, [0, 0, 0])

    def test_update_geometry_one_right(self):
        self.quarter_car_R.jounce(jounce=0.25)
        self.quarter_car_R.steer(rack_displacement=0)

        known_results = [3.67, -7.18, -7.18, 1.032014, -4.483288, -0.25, 1.997948, -3.920285, 0.75, 0.794828]
        test_results = [round(self.tire_R.steered_angle * 180 / np.pi, 2),
                        round(self.lower_wishbone_R.angle * 180 / np.pi, 2),
                        round(self.upper_wishbone_R.angle * 180 / np.pi, 2),
                        round(self.tire_R.contact_patch[0], 6),
                        round(self.tire_R.contact_patch[1], 6),
                        round(self.tire_R.contact_patch[2], 6),
                        round(self.tie_rod_R.outboard_node[0], 6),
                        round(self.tie_rod_R.outboard_node[1], 6),
                        round(self.tie_rod_R.outboard_node[2], 6),
                        round(self.spring_R.length, 6)]

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
    
    def test_update_geometry_one_right_repeated(self):
        self.quarter_car_L.jounce(jounce=0.125)
        self.quarter_car_L.steer(rack_displacement=0.1)
        self.quarter_car_R.jounce(jounce=0.25)
        self.quarter_car_R.steer(rack_displacement=0)

        known_results = [3.67, -7.18, -7.18, 1.032014, -4.483288, -0.25, 1.997948, -3.920285, 0.75, 0.794828]
        test_results = [round(self.tire_R.steered_angle * 180 / np.pi, 2),
                        round(self.lower_wishbone_R.angle * 180 / np.pi, 2),
                        round(self.upper_wishbone_R.angle * 180 / np.pi, 2),
                        round(self.tire_R.contact_patch[0], 6),
                        round(self.tire_R.contact_patch[1], 6),
                        round(self.tire_R.contact_patch[2], 6),
                        round(self.tie_rod_R.outboard_node[0], 6),
                        round(self.tie_rod_R.outboard_node[1], 6),
                        round(self.tie_rod_R.outboard_node[2], 6),
                        round(self.spring_R.length, 6)]

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
    
    def test_update_geometry_two_right(self):
        self.quarter_car_R.jounce(jounce=0.5)
        self.quarter_car_R.steer(rack_displacement=-0.15)

        known_results = [-0.93, -14.48, -14.48, 0.99185802, -4.43642538, 0.0, 1.99986741, -3.95277563, 1.0, 0.650041]
        test_results = [round(self.tire_R.steered_angle * 180 / np.pi, 2),
                        round(self.lower_wishbone_R.angle * 180 / np.pi, 2),
                        round(self.upper_wishbone_R.angle * 180 / np.pi, 2),
                        round(self.tire_R.contact_patch[0], 6),
                        round(self.tire_R.contact_patch[1], 6),
                        round(self.tire_R.contact_patch[2], 6),
                        round(self.tie_rod_R.outboard_node[0], 6),
                        round(self.tie_rod_R.outboard_node[1], 6),
                        round(self.tie_rod_R.outboard_node[2], 6),
                        round(self.spring_R.length, 6)]

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
    
    def test_update_geometry_two_right_repeated(self):
        self.quarter_car_L.jounce(jounce=0.125)
        self.quarter_car_L.steer(rack_displacement=0.1)
        self.quarter_car_R.jounce(jounce=0.5)
        self.quarter_car_R.steer(rack_displacement=-0.15)

        known_results = [-0.93, -14.48, -14.48, 0.99185802, -4.43642538, 0.0, 1.99986741, -3.95277563, 1.0, 0.650041]
        test_results = [round(self.tire_R.steered_angle * 180 / np.pi, 2),
                        round(self.lower_wishbone_R.angle * 180 / np.pi, 2),
                        round(self.upper_wishbone_R.angle * 180 / np.pi, 2),
                        round(self.tire_R.contact_patch[0], 6),
                        round(self.tire_R.contact_patch[1], 6),
                        round(self.tire_R.contact_patch[2], 6),
                        round(self.tie_rod_R.outboard_node[0], 6),
                        round(self.tie_rod_R.outboard_node[1], 6),
                        round(self.tie_rod_R.outboard_node[2], 6),
                        round(self.spring_R.length, 6)]

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

    def test_update_geometry_three_right(self):
        self.quarter_car_R.jounce(jounce=-0.1)
        self.quarter_car_R.steer(rack_displacement=0.25)

        known_results = [13.02, 2.87, 2.87, 1.11264350, -4.48464466, -0.6, 1.97429245, -3.77221144, 0.4, 1.117498]
        test_results = [round(self.tire_R.steered_angle * 180 / np.pi, 2),
                        round(self.lower_wishbone_R.angle * 180 / np.pi, 2),
                        round(self.upper_wishbone_R.angle * 180 / np.pi, 2),
                        round(self.tire_R.contact_patch[0], 6),
                        round(self.tire_R.contact_patch[1], 6),
                        round(self.tire_R.contact_patch[2], 6),
                        round(self.tie_rod_R.outboard_node[0], 6),
                        round(self.tie_rod_R.outboard_node[1], 6),
                        round(self.tie_rod_R.outboard_node[2], 6),
                        round(self.spring_R.length, 6)]

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
    
    def test_update_geometry_three_right_repeated(self):
        self.quarter_car_L.jounce(jounce=0.125)
        self.quarter_car_L.steer(rack_displacement=0.1)
        self.quarter_car_R.jounce(jounce=-0.1)
        self.quarter_car_R.steer(rack_displacement=0.25)

        known_results = [13.02, 2.87, 2.87, 1.11264350, -4.48464466, -0.6, 1.97429245, -3.77221144, 0.4, 1.117498]
        test_results = [round(self.tire_R.steered_angle * 180 / np.pi, 2),
                        round(self.lower_wishbone_R.angle * 180 / np.pi, 2),
                        round(self.upper_wishbone_R.angle * 180 / np.pi, 2),
                        round(self.tire_R.contact_patch[0], 6),
                        round(self.tire_R.contact_patch[1], 6),
                        round(self.tire_R.contact_patch[2], 6),
                        round(self.tie_rod_R.outboard_node[0], 6),
                        round(self.tie_rod_R.outboard_node[1], 6),
                        round(self.tie_rod_R.outboard_node[2], 6),
                        round(self.spring_R.length, 6)]

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
    
    def test_stabar_one(self):
        self.quarter_car_L.jounce(jounce=0.5)
        self.quarter_car_R.jounce(jounce=0.5)

        known_result = 0

        self.assertEqual(self.stabar.rotation, known_result)
    
    def test_stabar_one_repeated(self):
        self.quarter_car_L.jounce(jounce=-0.125)
        self.quarter_car_R.jounce(jounce=0.25)
        self.quarter_car_L.jounce(jounce=0.5)
        self.quarter_car_R.jounce(jounce=0.5)

        known_result = 0

        self.assertEqual(self.stabar.rotation, known_result)

    def test_stabar_two(self):
        self.quarter_car_L.jounce(jounce=-0.125)
        self.quarter_car_R.jounce(jounce=-0.125)

        known_result = 0

        self.assertEqual(self.stabar.rotation, known_result)
    
    def test_stabar_two_repeated(self):
        self.quarter_car_L.jounce(jounce=-0.125)
        self.quarter_car_R.jounce(jounce=0.25)
        self.quarter_car_L.jounce(jounce=-0.125)
        self.quarter_car_R.jounce(jounce=-0.125)

        known_result = 0

        self.assertEqual(self.stabar.rotation, known_result)

    def test_stabar_three(self):
        self.quarter_car_L.jounce(jounce=0.5)
        self.quarter_car_R.jounce(jounce=0)

        known_result = 27.57

        self.assertEqual(round(self.stabar.rotation * 180 / np.pi, 2), known_result)
    
    def test_stabar_three_repeated(self):
        self.quarter_car_L.jounce(jounce=-0.125)
        self.quarter_car_R.jounce(jounce=0.25)
        self.quarter_car_L.jounce(jounce=0.5)
        self.quarter_car_R.jounce(jounce=0)

        known_result = 27.57

        self.assertEqual(round(self.stabar.rotation * 180 / np.pi, 2), known_result)
    
    def test_stabar_four(self):
        self.quarter_car_L.jounce(jounce=-0.125)
        self.quarter_car_R.jounce(jounce=0)

        known_result = -6.16

        self.assertEqual(round(self.stabar.rotation * 180 / np.pi, 2), known_result)
    
    def test_stabar_four_repeated(self):
        self.quarter_car_L.jounce(jounce=-0.125)
        self.quarter_car_R.jounce(jounce=0.25)
        self.quarter_car_L.jounce(jounce=-0.125)
        self.quarter_car_R.jounce(jounce=0)

        known_result = -6.16

        self.assertEqual(round(self.stabar.rotation * 180 / np.pi, 2), known_result)
    
    def test_stabar_five(self):
        self.quarter_car_L.jounce(jounce=0)
        self.quarter_car_R.jounce(jounce=0.5)

        known_result = -27.57

        self.assertEqual(round(self.stabar.rotation * 180 / np.pi, 2), known_result)
    
    def test_stabar_five_repeated(self):
        self.quarter_car_L.jounce(jounce=-0.125)
        self.quarter_car_R.jounce(jounce=0.25)
        self.quarter_car_L.jounce(jounce=0)
        self.quarter_car_R.jounce(jounce=0.5)

        known_result = -27.57

        self.assertEqual(round(self.stabar.rotation * 180 / np.pi, 2), known_result)
    
    def test_stabar_six(self):
        self.quarter_car_L.jounce(jounce=0)
        self.quarter_car_R.jounce(jounce=-0.125)

        known_result = 6.16

        self.assertEqual(round(self.stabar.rotation * 180 / np.pi, 2), known_result)
    
    def test_stabar_six_repeated(self):
        self.quarter_car_L.jounce(jounce=-0.125)
        self.quarter_car_R.jounce(jounce=0.25)
        self.quarter_car_L.jounce(jounce=0)
        self.quarter_car_R.jounce(jounce=-0.125)

        known_result = 6.16

        self.assertEqual(round(self.stabar.rotation * 180 / np.pi, 2), known_result)
    
    def test_stabar_seven(self):
        self.quarter_car_L.jounce(jounce=0.25)
        self.quarter_car_R.jounce(jounce=-0.125)

        known_result = 19.02

        self.assertEqual(round(self.stabar.rotation * 180 / np.pi, 2), known_result)
    
    def test_stabar_seven_repeated(self):
        self.quarter_car_L.jounce(jounce=-0.125)
        self.quarter_car_R.jounce(jounce=0.25)
        self.quarter_car_L.jounce(jounce=0.25)
        self.quarter_car_R.jounce(jounce=-0.125)

        known_result = 19.02

        self.assertEqual(round(self.stabar.rotation * 180 / np.pi, 2), known_result)
    
    def test_stabar_eight(self):
        self.quarter_car_L.jounce(jounce=-0.125)
        self.quarter_car_R.jounce(jounce=0.25)

        known_result = -19.02

        self.assertEqual(round(self.stabar.rotation * 180 / np.pi, 2), known_result)
    
    def test_stabar_eight_repeated(self):
        self.quarter_car_L.jounce(jounce=-0.125)
        self.quarter_car_R.jounce(jounce=0.25)
        self.quarter_car_L.jounce(jounce=-0.125)
        self.quarter_car_R.jounce(jounce=0.25)

        known_result = -19.02

        self.assertEqual(round(self.stabar.rotation * 180 / np.pi, 2), known_result)
    
    def test_stabar_nine(self):
        self.quarter_car_L.jounce(jounce=0.5)
        self.quarter_car_R.jounce(jounce=-0.125)

        known_result = 33.74

        self.assertEqual(round(self.stabar.rotation * 180 / np.pi, 2), known_result)
    
    def test_stabar_nine_repeated(self):
        self.quarter_car_L.jounce(jounce=-0.125)
        self.quarter_car_R.jounce(jounce=0.25)
        self.quarter_car_L.jounce(jounce=0.5)
        self.quarter_car_R.jounce(jounce=-0.125)

        known_result = 33.74

        self.assertEqual(round(self.stabar.rotation * 180 / np.pi, 2), known_result)
    
    def test_stabar_ten(self):
        self.quarter_car_L.jounce(jounce=-0.125)
        self.quarter_car_R.jounce(jounce=0.5)

        known_result = -33.74

        self.assertEqual(round(self.stabar.rotation * 180 / np.pi, 2), known_result)
    
    def test_stabar_ten_repeated(self):
        self.quarter_car_L.jounce(jounce=-0.125)
        self.quarter_car_R.jounce(jounce=0.25)
        self.quarter_car_L.jounce(jounce=-0.125)
        self.quarter_car_R.jounce(jounce=0.5)

        known_result = -33.74

        self.assertEqual(round(self.stabar.rotation * 180 / np.pi, 2), known_result)