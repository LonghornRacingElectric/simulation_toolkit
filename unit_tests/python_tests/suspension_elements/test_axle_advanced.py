from src.vehicle_model.suspension_model.suspension_elements._4_elements.push_pull_rod import PushPullRod
from src.vehicle_model.suspension_model.suspension_elements._5_elements.quarter_car import QuarterCar
from src.vehicle_model.suspension_model.suspension_elements._2_elements.bellcrank import Bellcrank
from src.vehicle_model.suspension_model.suspension_elements._2_elements.wishbone import Wishbone
from src.vehicle_model.suspension_model.suspension_elements._2_elements.stabar import Stabar
from src.vehicle_model.suspension_model.suspension_elements._2_elements.spring import Spring
from src.vehicle_model.suspension_model.suspension_elements._2_elements.tire import Tire
from src.vehicle_model.suspension_model.suspension_elements._1_elements.link import Link
from src.vehicle_model.suspension_model.suspension_elements._1_elements.node import Node
from LHR_tire_toolkit.MF52 import MF52 # type: ignore

from unittest import TestCase
import numpy as np

# Advanced axle test uses control arms configured with arbitrary points in 3D space
# This particular configuration has anti geometry, a kingpin angled toward the car, and negative caster

class TestAxleAdvanced(TestCase):
    ### Left corner
    # Nodes
    lower_inboard_fore_L = Node(position=[2, 2, 0])
    lower_inboard_aft_L = Node(position=[0, 2, 0])
    lower_outboard_L = Node(position=[1, 4, 0.00006249])

    upper_inboard_fore_L = Node(position=[2.125, 2.125, 1.25])
    upper_inboard_aft_L = Node(position=[-0.1, 2.25, 1.0])
    upper_outboard_L = Node(position=[1.12418341, 3.90000559, 0.98727048])

    tie_inboard_L = Node(position=[1.9, 2.1, 0.23750083])
    tie_outboard_L = Node(position=[2.00003349, 3.99909923, 0.38065338])

    bellcrank_pivot_L = Node(position=[1, 2, 2])
    bellcrank_direction_L = [1, 0, 0]

    pushrod_lower_L = Node(position=[1.12418341, 3.90000559, 0.98727048])
    pushrod_upper_L = Node(position=[1, 2.5, 2.00005703])
    inboard_rod_outboard_L = Node(position=[1, 1.99994297, 2.5])
    rod_to_spring_L = Node(position=[1, 0.99994297, 2.5])
    spring_inboard_L = Node(position=[1, 0, 2.5])

    contact_patch_L = Node(position=[1.00042957, 4.49993731, -0.5])

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
    tire_mf52_L = MF52(tire_name="test_tire", file_path="./unit_tests/python_tests/_test_dependencies/unit_test_tire.tir")
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
    lower_outboard_R = Node(position=[1, -4, 0.00006249])

    upper_inboard_fore_R = Node(position=[2.125, -2.125, 1.25])
    upper_inboard_aft_R = Node(position=[-0.1, -2.25, 1.0])
    upper_outboard_R = Node(position=[1.12418341, -3.90000559, 0.98727048])

    tie_inboard_R = Node(position=[1.9, -2.1, 0.23750083])
    tie_outboard_R = Node(position=[2.00003349, -3.99909923, 0.38065338])

    bellcrank_pivot_R = Node(position=[1, -2, 2])
    bellcrank_direction_R = [1, 0, 0]

    pushrod_lower_R = Node(position=[1.12418341, -3.90000559, 0.98727048])
    pushrod_upper_R = Node(position=[1, -2.5, 2.00005703])
    inboard_rod_outboard_R = Node(position=[1, -1.99994297, 2.5])
    rod_to_spring_R = Node(position=[1, -0.99994297, 2.5])
    spring_inboard_R = Node(position=[1, 0, 2.5])

    contact_patch_R = Node(position=[1.00042957, -4.49993731, -0.5])

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
    tire_mf52_R = MF52(tire_name="test_tire", file_path="./unit_tests/python_tests/_test_dependencies/unit_test_tire.tir")
    tire_R = Tire(tire=tire_mf52_R, contact_patch=contact_patch_R, outer_diameter=16*0.0254, width=7*0.0254, inner_diameter=10*0.0254)

    # Set relation between upper wishbone and pushrod
    upper_outboard_R.add_child(node=pushrod_lower_R)

    quarter_car_R = QuarterCar(tire=tire_R,
                                lower_wishbone=lower_wishbone_R,
                                upper_wishbone=upper_wishbone_R,
                                tie_rod=tie_rod_R,
                                push_pull_rod=pushrod_R)

    ### Stabar
    left_arm_end = Node(position=[1.98908984, 2.5, 1.89611964])
    right_arm_end = Node(position=[1.98908984, -2.5, 1.89611964])
    left_droplink_end = Node(position=[1.62459170, 3.01250280, 1.11863524])
    right_droplink_end = Node(position=[1.62459170, -3.01250280, 1.11863524])
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
        lower_wishbone_angle = 7.5064069 * np.pi / 180
        upper_wishbone_angle = 8.92843131 * np.pi / 180
        wheel_angle = -1.92297609 * np.pi / 180

        # Manually set jounce
        self.quarter_car_L.wheel_jounce = self.contact_patch_L.initial_position[2] + 0.25

        # Calculate residuals and confirm residuals go to zero
        residuals = self.quarter_car_L._geometry_resid_func(x=[lower_wishbone_angle, upper_wishbone_angle, wheel_angle])

        rounded_residuals = [round(x, 2) for x in residuals]

        self.assertListEqual(rounded_residuals, [0, 0, 0])

    def test_update_geometry_one_left(self):
        self.quarter_car_L.jounce(jounce=0.25)
        self.quarter_car_L.steer(rack_displacement=0)

        known_results = [-1.92297609, 7.5064069, 8.92843131, 1.03174301, 4.4702206, -0.25, 1.9889554, 3.95553957, 0.6689369, 0.7970262]
        test_results = [self.tire_L.steered_angle * 180 / np.pi,
                        self.lower_wishbone_L.angle * 180 / np.pi,
                        self.upper_wishbone_L.angle * 180 / np.pi,
                        self.tire_L.contact_patch[0],
                        self.tire_L.contact_patch[1],
                        self.tire_L.contact_patch[2],
                        self.tie_rod_L.outboard_node[0],
                        self.tie_rod_L.outboard_node[1],
                        self.tie_rod_L.outboard_node[2],
                        self.spring_L.length]

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
        
        tolerances = [0.001,
                      0.001,
                      0.001,
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

        known_results = [-13.4382, 15.173, 17.9412, 1.13627065, 4.38561849, 0.0, 1.9574342, 3.71755794, 0.95130577, 0.6483891]
        test_results = [self.tire_L.steered_angle * 180 / np.pi,
                        self.lower_wishbone_L.angle * 180 / np.pi,
                        self.upper_wishbone_L.angle * 180 / np.pi,
                        self.tire_L.contact_patch[0],
                        self.tire_L.contact_patch[1],
                        self.tire_L.contact_patch[2],
                        self.tie_rod_L.outboard_node[0],
                        self.tie_rod_L.outboard_node[1],
                        self.tie_rod_L.outboard_node[2],
                        self.spring_L.length]

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
        
        tolerances = [0.001,
                      0.001,
                      0.001,
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

        known_results = [16.4573, -3.55849, -4.29109, 0.86481624, 4.50133922, -0.6, 1.9561914, 4.25581283, 0.27989442, 1.13104708]
        test_results = [self.tire_L.steered_angle * 180 / np.pi,
                        self.lower_wishbone_L.angle * 180 / np.pi,
                        self.upper_wishbone_L.angle * 180 / np.pi,
                        self.tire_L.contact_patch[0],
                        self.tire_L.contact_patch[1],
                        self.tire_L.contact_patch[2],
                        self.tie_rod_L.outboard_node[0],
                        self.tie_rod_L.outboard_node[1],
                        self.tie_rod_L.outboard_node[2],
                        self.spring_L.length]

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
        
        tolerances = [0.001,
                      0.001,
                      0.001,
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
        lower_wishbone_angle = -7.5064069 * np.pi / 180
        upper_wishbone_angle = -8.92843131 * np.pi / 180
        wheel_angle = 1.92297609 * np.pi / 180

        # Manually set jounce
        self.quarter_car_R.wheel_jounce = self.contact_patch_L.initial_position[2] + 0.25

        # Calculate residuals and confirm residuals go to zero
        residuals = self.quarter_car_R._geometry_resid_func(x=[lower_wishbone_angle, upper_wishbone_angle, wheel_angle])

        rounded_residuals = [round(x, 2) for x in residuals]

        self.assertListEqual(rounded_residuals, [0, 0, 0])

    def test_update_geometry_one_right(self):
        self.quarter_car_R.jounce(jounce=0.25)
        self.quarter_car_R.steer(rack_displacement=0)

        known_results = [1.92297609, -7.5064069, -8.92843131, 1.03174301, -4.4702206, -0.25, 1.9889554, -3.95553957, 0.6689369, 0.7970262]
        test_results = [self.tire_R.steered_angle * 180 / np.pi,
                        self.lower_wishbone_R.angle * 180 / np.pi,
                        self.upper_wishbone_R.angle * 180 / np.pi,
                        self.tire_R.contact_patch[0],
                        self.tire_R.contact_patch[1],
                        self.tire_R.contact_patch[2],
                        self.tie_rod_R.outboard_node[0],
                        self.tie_rod_R.outboard_node[1],
                        self.tie_rod_R.outboard_node[2],
                        self.spring_R.length]

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
        
        tolerances = [0.001,
                      0.001,
                      0.001,
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

        known_results = [-4.48566, -14.8494, -17.5621, 0.99860835, -4.42024104, 0.0, 1.96937047, -4.01427356, 0.9583363, 0.65340869]
        test_results = [self.tire_R.steered_angle * 180 / np.pi,
                        self.lower_wishbone_R.angle * 180 / np.pi,
                        self.upper_wishbone_R.angle * 180 / np.pi,
                        self.tire_R.contact_patch[0],
                        self.tire_R.contact_patch[1],
                        self.tire_R.contact_patch[2],
                        self.tie_rod_R.outboard_node[0],
                        self.tie_rod_R.outboard_node[1],
                        self.tie_rod_R.outboard_node[2],
                        self.spring_R.length]

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
        
        tolerances = [0.001,
                      0.001,
                      0.001,
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

        known_results = [14.696, 2.46439, 2.9667, 1.105437, -4.47209349, -0.6, 1.98232467, -3.75517963, 0.26180026, 1.08558541]
        test_results = [self.tire_R.steered_angle * 180 / np.pi,
                        self.lower_wishbone_R.angle * 180 / np.pi,
                        self.upper_wishbone_R.angle * 180 / np.pi,
                        self.tire_R.contact_patch[0],
                        self.tire_R.contact_patch[1],
                        self.tire_R.contact_patch[2],
                        self.tie_rod_R.outboard_node[0],
                        self.tie_rod_R.outboard_node[1],
                        self.tie_rod_R.outboard_node[2],
                        self.spring_R.length]

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
        
        tolerances = [0.001,
                      0.001,
                      0.001,
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

    def test_stabar_two(self):
        self.quarter_car_L.jounce(jounce=-0.125)
        self.quarter_car_R.jounce(jounce=-0.125)

        known_result = 0

        self.assertEqual(self.stabar.rotation, known_result)

    def test_stabar_three(self):
        self.quarter_car_L.jounce(jounce=0.5)
        self.quarter_car_R.jounce(jounce=0)

        known_result = 31.1811

        tolerance = 0.001

        self.assertLess(abs(self.stabar.rotation * 180 / np.pi - known_result), tolerance)
    
    def test_stabar_four(self):
        self.quarter_car_L.jounce(jounce=-0.125)
        self.quarter_car_R.jounce(jounce=0)

        known_result = -6.26456

        tolerance = 0.001

        self.assertLess(abs(self.stabar.rotation * 180 / np.pi - known_result), tolerance)
    
    def test_stabar_five(self):
        self.quarter_car_L.jounce(jounce=0)
        self.quarter_car_R.jounce(jounce=0.5)

        known_result = -31.1811

        tolerance = 0.001

        self.assertLess(abs(self.stabar.rotation * 180 / np.pi - known_result), tolerance)
    
    def test_stabar_six(self):
        self.quarter_car_L.jounce(jounce=0)
        self.quarter_car_R.jounce(jounce=-0.125)

        known_result = 6.26456

        tolerance = 0.001

        self.assertLess(abs(self.stabar.rotation * 180 / np.pi - known_result), tolerance)
    
    def test_stabar_seven(self):
        self.quarter_car_L.jounce(jounce=0.25)
        self.quarter_car_R.jounce(jounce=-0.125)

        known_result = 20.0884

        tolerance = 0.001

        self.assertLess(abs(self.stabar.rotation * 180 / np.pi - known_result), tolerance)
    
    def test_stabar_eight(self):
        self.quarter_car_L.jounce(jounce=-0.125)
        self.quarter_car_R.jounce(jounce=0.25)

        known_result = -20.0884

        tolerance = 0.001

        self.assertLess(abs(self.stabar.rotation * 180 / np.pi - known_result), tolerance)
    
    def test_stabar_nine(self):
        self.quarter_car_L.jounce(jounce=0.5)
        self.quarter_car_R.jounce(jounce=-0.125)

        known_result = 37.4457

        tolerance = 0.001

        self.assertLess(abs(self.stabar.rotation * 180 / np.pi - known_result), tolerance)
    
    def test_stabar_ten(self):
        self.quarter_car_L.jounce(jounce=-0.125)
        self.quarter_car_R.jounce(jounce=0.5)

        known_result = -37.4457

        tolerance = 0.001

        self.assertLess(abs(self.stabar.rotation * 180 / np.pi - known_result), tolerance)