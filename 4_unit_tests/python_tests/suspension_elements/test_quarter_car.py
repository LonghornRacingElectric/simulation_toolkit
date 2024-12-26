from vehicle_model.suspension_model.suspension_elements.quaternary_elements.quarter_car import QuarterCar
from vehicle_model.suspension_model.suspension_elements.secondary_elements.wishbone import Wishbone
from vehicle_model.suspension_model.suspension_elements.primary_elements.link import Link
from vehicle_model.suspension_model.suspension_elements.primary_elements.node import Node
import numpy as np

from unittest import main, TestCase


class TestQuarterCar(TestCase):
    def test_jounce_lower(self):
        # Nodes
        lower_inboard_fore = Node(position=[2, 0, 0])
        lower_inboard_aft = Node(position=[0, 0, 0])
        lower_outboard = Node(position=[1, 2, 0])

        upper_inboard_fore = Node(position=[2, 0, 1])
        upper_inboard_aft = Node(position=[0, 0, 1])
        upper_outboard = Node(position=[1, 2, 1])

        tie_inboard = Node(position=[2, 0, 0.5])
        tie_outboard = Node(position=[2, 2, 0.5])

        pushrod_lower = Node(position=[1, 2, 1])
        pushrod_upper = Node(position=[1, 0, 2])

        # Links
        lower_fore_link = Link(inboard_node=lower_inboard_fore, outboard_node=lower_outboard)
        lower_aft_link = Link(inboard_node=lower_inboard_aft, outboard_node=lower_outboard)

        upper_fore_link = Link(inboard_node=upper_inboard_fore, outboard_node=upper_outboard)
        upper_aft_link = Link(inboard_node=upper_inboard_aft, outboard_node=upper_outboard)

        tie_rod = Link(inboard_node=tie_inboard, outboard_node=tie_outboard)

        pushrod = Link(inboard_node=pushrod_upper, outboard_node=pushrod_lower)

        # Wishbones
        lower_wishbone = Wishbone(fore_link=lower_fore_link, aft_link=lower_aft_link)
        upper_wishbone = Wishbone(fore_link=upper_fore_link, aft_link=upper_aft_link)

        double_wishbone = QuarterCar(lower_wishbone=lower_wishbone,
                                         upper_wishbone=upper_wishbone,
                                         tie_rod=tie_rod,
                                         push_pull_rod=pushrod)

        double_wishbone.jounce(jounce=0.5)
        self.assertEqual(round(lower_wishbone.angle * 180 / np.pi, 3), round(np.asin(0.5/2) * 180 / np.pi, 3))
    
    def test_jounce_lower_repeated(self):
        # Nodes
        lower_inboard_fore = Node(position=[2, 0, 0])
        lower_inboard_aft = Node(position=[0, 0, 0])
        lower_outboard = Node(position=[1, 2, 0])

        upper_inboard_fore = Node(position=[2, 0, 1])
        upper_inboard_aft = Node(position=[0, 0, 1])
        upper_outboard = Node(position=[1, 2, 1])

        tie_inboard = Node(position=[2, 0, 0.5])
        tie_outboard = Node(position=[2, 2, 0.5])

        pushrod_lower = Node(position=[1, 2, 1])
        pushrod_upper = Node(position=[1, 0, 2])

        # Links
        lower_fore_link = Link(inboard_node=lower_inboard_fore, outboard_node=lower_outboard)
        lower_aft_link = Link(inboard_node=lower_inboard_aft, outboard_node=lower_outboard)

        upper_fore_link = Link(inboard_node=upper_inboard_fore, outboard_node=upper_outboard)
        upper_aft_link = Link(inboard_node=upper_inboard_aft, outboard_node=upper_outboard)

        tie_rod = Link(inboard_node=tie_inboard, outboard_node=tie_outboard)

        pushrod = Link(inboard_node=pushrod_upper, outboard_node=pushrod_lower)

        # Wishbones
        lower_wishbone = Wishbone(fore_link=lower_fore_link, aft_link=lower_aft_link)
        upper_wishbone = Wishbone(fore_link=upper_fore_link, aft_link=upper_aft_link)

        double_wishbone = QuarterCar(lower_wishbone=lower_wishbone,
                                         upper_wishbone=upper_wishbone,
                                         tie_rod=tie_rod,
                                         push_pull_rod=pushrod)
        
        double_wishbone.jounce(jounce=0.5)
        double_wishbone.jounce(jounce=-0.5)

        self.assertEqual(round(lower_wishbone.angle * 180 / np.pi, 3), round(-np.asin(0.5/2) * 180 / np.pi, 3))

    def test_jounce_upper(self):
        # Nodes
        lower_inboard_fore = Node(position=[2, 0, 0])
        lower_inboard_aft = Node(position=[0, 0, 0])
        lower_outboard = Node(position=[1, 2, 0])

        upper_inboard_fore = Node(position=[2, 0, 1])
        upper_inboard_aft = Node(position=[0, 0, 1])
        upper_outboard = Node(position=[1, 2, 1])

        tie_inboard = Node(position=[2, 0, 0.5])
        tie_outboard = Node(position=[2, 2, 0.5])

        pushrod_lower = Node(position=[1, 2, 1])
        pushrod_upper = Node(position=[1, 0, 2])

        # Links
        lower_fore_link = Link(inboard_node=lower_inboard_fore, outboard_node=lower_outboard)
        lower_aft_link = Link(inboard_node=lower_inboard_aft, outboard_node=lower_outboard)

        upper_fore_link = Link(inboard_node=upper_inboard_fore, outboard_node=upper_outboard)
        upper_aft_link = Link(inboard_node=upper_inboard_aft, outboard_node=upper_outboard)

        tie_rod = Link(inboard_node=tie_inboard, outboard_node=tie_outboard)

        pushrod = Link(inboard_node=pushrod_upper, outboard_node=pushrod_lower)

        # Wishbones
        lower_wishbone = Wishbone(fore_link=lower_fore_link, aft_link=lower_aft_link)
        upper_wishbone = Wishbone(fore_link=upper_fore_link, aft_link=upper_aft_link)

        double_wishbone = QuarterCar(lower_wishbone=lower_wishbone,
                                         upper_wishbone=upper_wishbone,
                                         tie_rod=tie_rod,
                                         push_pull_rod=pushrod)

        double_wishbone.jounce(jounce=0.5)
        self.assertEqual(round(upper_wishbone.angle * 180 / np.pi, 3), round(np.asin(0.5/2) * 180 / np.pi, 3))
    
    def test_jounce_upper_repeated(self):
        # Nodes
        lower_inboard_fore = Node(position=[2, 0, 0])
        lower_inboard_aft = Node(position=[0, 0, 0])
        lower_outboard = Node(position=[1, 2, 0])

        upper_inboard_fore = Node(position=[2, 0, 1])
        upper_inboard_aft = Node(position=[0, 0, 1])
        upper_outboard = Node(position=[1, 2, 1])

        tie_inboard = Node(position=[2, 0, 0.5])
        tie_outboard = Node(position=[2, 2, 0.5])

        pushrod_lower = Node(position=[1, 2, 1])
        pushrod_upper = Node(position=[1, 0, 2])

        # Links
        lower_fore_link = Link(inboard_node=lower_inboard_fore, outboard_node=lower_outboard)
        lower_aft_link = Link(inboard_node=lower_inboard_aft, outboard_node=lower_outboard)

        upper_fore_link = Link(inboard_node=upper_inboard_fore, outboard_node=upper_outboard)
        upper_aft_link = Link(inboard_node=upper_inboard_aft, outboard_node=upper_outboard)

        tie_rod = Link(inboard_node=tie_inboard, outboard_node=tie_outboard)

        pushrod = Link(inboard_node=pushrod_upper, outboard_node=pushrod_lower)

        # Wishbones
        lower_wishbone = Wishbone(fore_link=lower_fore_link, aft_link=lower_aft_link)
        upper_wishbone = Wishbone(fore_link=upper_fore_link, aft_link=upper_aft_link)

        double_wishbone = QuarterCar(lower_wishbone=lower_wishbone,
                                         upper_wishbone=upper_wishbone,
                                         tie_rod=tie_rod,
                                         push_pull_rod=pushrod)

        double_wishbone.jounce(jounce=0.5)
        double_wishbone.jounce(jounce=-0.5)

        self.assertEqual(round(upper_wishbone.angle * 180 / np.pi, 3), round(-np.asin(0.5/2) * 180 / np.pi, 3))
    
    def test_rack_displacement_one(self):
        # Nodes
        lower_inboard_fore = Node(position=[2, 0, 0])
        lower_inboard_aft = Node(position=[0, 0, 0])
        lower_outboard = Node(position=[1, 2, 0])

        upper_inboard_fore = Node(position=[2, 0, 1])
        upper_inboard_aft = Node(position=[0, 0, 1])
        upper_outboard = Node(position=[1, 2, 1])

        tie_inboard = Node(position=[2, 0, 0])
        tie_outboard = Node(position=[2, 2, 0.5])

        pushrod_lower = Node(position=[1, 2, 1])
        pushrod_upper = Node(position=[1, 0, 2])

        # Links
        lower_fore_link = Link(inboard_node=lower_inboard_fore, outboard_node=lower_outboard)
        lower_aft_link = Link(inboard_node=lower_inboard_aft, outboard_node=lower_outboard)

        upper_fore_link = Link(inboard_node=upper_inboard_fore, outboard_node=upper_outboard)
        upper_aft_link = Link(inboard_node=upper_inboard_aft, outboard_node=upper_outboard)

        tie_rod = Link(inboard_node=tie_inboard, outboard_node=tie_outboard)

        pushrod = Link(inboard_node=pushrod_upper, outboard_node=pushrod_lower)

        # Wishbones
        lower_wishbone = Wishbone(fore_link=lower_fore_link, aft_link=lower_aft_link)
        upper_wishbone = Wishbone(fore_link=upper_fore_link, aft_link=upper_aft_link)

        double_wishbone = QuarterCar(lower_wishbone=lower_wishbone,
                                         upper_wishbone=upper_wishbone,
                                         tie_rod=tie_rod,
                                         push_pull_rod=pushrod)

        double_wishbone.steer(rack_translation=0.25)

        self.assertEqual(double_wishbone.tie_rod.inboard_node[1], 0.25)
    
    def test_rack_displacement_two(self):
        # Nodes
        lower_inboard_fore = Node(position=[2, 0, 0])
        lower_inboard_aft = Node(position=[0, 0, 0])
        lower_outboard = Node(position=[1, 2, 0])

        upper_inboard_fore = Node(position=[2, 0, 1])
        upper_inboard_aft = Node(position=[0, 0, 1])
        upper_outboard = Node(position=[1, 2, 1])

        tie_inboard = Node(position=[2, 0, 0.5])
        tie_outboard = Node(position=[2, 2, 0.5])

        pushrod_lower = Node(position=[1, 2, 1])
        pushrod_upper = Node(position=[1, 0, 2])

        # Links
        lower_fore_link = Link(inboard_node=lower_inboard_fore, outboard_node=lower_outboard)
        lower_aft_link = Link(inboard_node=lower_inboard_aft, outboard_node=lower_outboard)

        upper_fore_link = Link(inboard_node=upper_inboard_fore, outboard_node=upper_outboard)
        upper_aft_link = Link(inboard_node=upper_inboard_aft, outboard_node=upper_outboard)

        tie_rod = Link(inboard_node=tie_inboard, outboard_node=tie_outboard)

        pushrod = Link(inboard_node=pushrod_upper, outboard_node=pushrod_lower)

        # Wishbones
        lower_wishbone = Wishbone(fore_link=lower_fore_link, aft_link=lower_aft_link)
        upper_wishbone = Wishbone(fore_link=upper_fore_link, aft_link=upper_aft_link)

        double_wishbone = QuarterCar(lower_wishbone=lower_wishbone,
                                         upper_wishbone=upper_wishbone,
                                         tie_rod=tie_rod,
                                         push_pull_rod=pushrod)

        double_wishbone.steer(rack_translation=0.25)

        self.assertEqual(double_wishbone.tie_rod.inboard_node[1], 0.25)
    
    def test_rack_displacement_three(self):
        # Nodes
        lower_inboard_fore = Node(position=[2, 0, 0])
        lower_inboard_aft = Node(position=[0, 0, 0])
        lower_outboard = Node(position=[1, 2, 0])

        upper_inboard_fore = Node(position=[2, 0, 1])
        upper_inboard_aft = Node(position=[0, 0, 1])
        upper_outboard = Node(position=[1, 2, 1])

        tie_inboard = Node(position=[2, 0, 1])
        tie_outboard = Node(position=[2, 2, 0.5])

        pushrod_lower = Node(position=[1, 2, 1])
        pushrod_upper = Node(position=[1, 0, 2])

        # Links
        lower_fore_link = Link(inboard_node=lower_inboard_fore, outboard_node=lower_outboard)
        lower_aft_link = Link(inboard_node=lower_inboard_aft, outboard_node=lower_outboard)

        upper_fore_link = Link(inboard_node=upper_inboard_fore, outboard_node=upper_outboard)
        upper_aft_link = Link(inboard_node=upper_inboard_aft, outboard_node=upper_outboard)

        tie_rod = Link(inboard_node=tie_inboard, outboard_node=tie_outboard)

        pushrod = Link(inboard_node=pushrod_upper, outboard_node=pushrod_lower)

        # Wishbones
        lower_wishbone = Wishbone(fore_link=lower_fore_link, aft_link=lower_aft_link)
        upper_wishbone = Wishbone(fore_link=upper_fore_link, aft_link=upper_aft_link)

        double_wishbone = QuarterCar(lower_wishbone=lower_wishbone,
                                         upper_wishbone=upper_wishbone,
                                         tie_rod=tie_rod,
                                         push_pull_rod=pushrod)

        double_wishbone.steer(rack_translation=0.25)

        self.assertEqual(double_wishbone.tie_rod.inboard_node[1], 0.25)
    
    def test_combined_steer_one(self):
        # Nodes
        lower_inboard_fore = Node(position=[2, 0, 0])
        lower_inboard_aft = Node(position=[0, 0, 0])
        lower_outboard = Node(position=[1, 2, 0])

        upper_inboard_fore = Node(position=[2, 0, 1])
        upper_inboard_aft = Node(position=[0, 0, 1])
        upper_outboard = Node(position=[1, 2, 1])

        tie_inboard = Node(position=[2, 0, 0])
        tie_outboard = Node(position=[2, 2, 0.5])

        pushrod_lower = Node(position=[1, 2, 1])
        pushrod_upper = Node(position=[1, 0, 2])

        # Links
        lower_fore_link = Link(inboard_node=lower_inboard_fore, outboard_node=lower_outboard)
        lower_aft_link = Link(inboard_node=lower_inboard_aft, outboard_node=lower_outboard)

        upper_fore_link = Link(inboard_node=upper_inboard_fore, outboard_node=upper_outboard)
        upper_aft_link = Link(inboard_node=upper_inboard_aft, outboard_node=upper_outboard)

        tie_rod = Link(inboard_node=tie_inboard, outboard_node=tie_outboard)

        pushrod = Link(inboard_node=pushrod_upper, outboard_node=pushrod_lower)

        # Wishbones
        lower_wishbone = Wishbone(fore_link=lower_fore_link, aft_link=lower_aft_link)
        upper_wishbone = Wishbone(fore_link=upper_fore_link, aft_link=upper_aft_link)

        double_wishbone = QuarterCar(lower_wishbone=lower_wishbone,
                                         upper_wishbone=upper_wishbone,
                                         tie_rod=tie_rod,
                                         push_pull_rod=pushrod)

        double_wishbone.jounce(jounce=0)
        double_wishbone.steer(rack_translation=0.25)

        known_angle = 14.46 # Found using Solidworks
        self.assertEqual(round(double_wishbone.wheel_angle * 180 / np.pi, 2), known_angle)
    
    def test_combined_steer_two(self):
        # Nodes
        lower_inboard_fore = Node(position=[2, 0, 0])
        lower_inboard_aft = Node(position=[0, 0, 0])
        lower_outboard = Node(position=[1, 2, 0])

        upper_inboard_fore = Node(position=[2, 0, 1])
        upper_inboard_aft = Node(position=[0, 0, 1])
        upper_outboard = Node(position=[1, 2, 1])

        tie_inboard = Node(position=[2, 0, 0])
        tie_outboard = Node(position=[2, 2, 0.5])

        pushrod_lower = Node(position=[1, 2, 1])
        pushrod_upper = Node(position=[1, 0, 2])

        # Links
        lower_fore_link = Link(inboard_node=lower_inboard_fore, outboard_node=lower_outboard)
        lower_aft_link = Link(inboard_node=lower_inboard_aft, outboard_node=lower_outboard)

        upper_fore_link = Link(inboard_node=upper_inboard_fore, outboard_node=upper_outboard)
        upper_aft_link = Link(inboard_node=upper_inboard_aft, outboard_node=upper_outboard)

        tie_rod = Link(inboard_node=tie_inboard, outboard_node=tie_outboard)

        pushrod = Link(inboard_node=pushrod_upper, outboard_node=pushrod_lower)

        # Wishbones
        lower_wishbone = Wishbone(fore_link=lower_fore_link, aft_link=lower_aft_link)
        upper_wishbone = Wishbone(fore_link=upper_fore_link, aft_link=upper_aft_link)

        double_wishbone = QuarterCar(lower_wishbone=lower_wishbone,
                                         upper_wishbone=upper_wishbone,
                                         tie_rod=tie_rod,
                                         push_pull_rod=pushrod)

        double_wishbone.jounce(jounce=-1)
        double_wishbone.steer(rack_translation=0.25)

        known_angle = 30.86 # Found using Solidworks
        self.assertEqual(round(double_wishbone.wheel_angle * 180 / np.pi, 2), known_angle)
    
    def test_combined_steer_three(self):
        # Nodes
        lower_inboard_fore = Node(position=[2, 0, 0])
        lower_inboard_aft = Node(position=[0, 0, 0])
        lower_outboard = Node(position=[1, 2, 0])

        upper_inboard_fore = Node(position=[2, 0, 1])
        upper_inboard_aft = Node(position=[0, 0, 1])
        upper_outboard = Node(position=[1, 2, 1])

        tie_inboard = Node(position=[2, 0, 0])
        tie_outboard = Node(position=[2, 2, 0.5])

        pushrod_lower = Node(position=[1, 2, 1])
        pushrod_upper = Node(position=[1, 0, 2])

        # Links
        lower_fore_link = Link(inboard_node=lower_inboard_fore, outboard_node=lower_outboard)
        lower_aft_link = Link(inboard_node=lower_inboard_aft, outboard_node=lower_outboard)

        upper_fore_link = Link(inboard_node=upper_inboard_fore, outboard_node=upper_outboard)
        upper_aft_link = Link(inboard_node=upper_inboard_aft, outboard_node=upper_outboard)

        tie_rod = Link(inboard_node=tie_inboard, outboard_node=tie_outboard)

        pushrod = Link(inboard_node=pushrod_upper, outboard_node=pushrod_lower)

        # Wishbones
        lower_wishbone = Wishbone(fore_link=lower_fore_link, aft_link=lower_aft_link)
        upper_wishbone = Wishbone(fore_link=upper_fore_link, aft_link=upper_aft_link)

        double_wishbone = QuarterCar(lower_wishbone=lower_wishbone,
                                         upper_wishbone=upper_wishbone,
                                         tie_rod=tie_rod,
                                         push_pull_rod=pushrod)

        double_wishbone.jounce(jounce=1)
        double_wishbone.steer(rack_translation=0.25)

        known_angle = -3.89 # Found using Solidworks
        self.assertEqual(round(double_wishbone.wheel_angle * 180 / np.pi, 2), known_angle)
    
    def test_combined_steer_four(self):
        # Nodes
        lower_inboard_fore = Node(position=[2, 0, 0])
        lower_inboard_aft = Node(position=[0, 0, 0])
        lower_outboard = Node(position=[1, 2, 0])

        upper_inboard_fore = Node(position=[2, 0, 1])
        upper_inboard_aft = Node(position=[0, 0, 1])
        upper_outboard = Node(position=[1, 2, 1])

        tie_inboard = Node(position=[2, 0, 0])
        tie_outboard = Node(position=[2, 2, 0.5])

        pushrod_lower = Node(position=[1, 2, 1])
        pushrod_upper = Node(position=[1, 0, 2])

        # Links
        lower_fore_link = Link(inboard_node=lower_inboard_fore, outboard_node=lower_outboard)
        lower_aft_link = Link(inboard_node=lower_inboard_aft, outboard_node=lower_outboard)

        upper_fore_link = Link(inboard_node=upper_inboard_fore, outboard_node=upper_outboard)
        upper_aft_link = Link(inboard_node=upper_inboard_aft, outboard_node=upper_outboard)

        tie_rod = Link(inboard_node=tie_inboard, outboard_node=tie_outboard)

        pushrod = Link(inboard_node=pushrod_upper, outboard_node=pushrod_lower)

        # Wishbones
        lower_wishbone = Wishbone(fore_link=lower_fore_link, aft_link=lower_aft_link)
        upper_wishbone = Wishbone(fore_link=upper_fore_link, aft_link=upper_aft_link)

        double_wishbone = QuarterCar(lower_wishbone=lower_wishbone,
                                         upper_wishbone=upper_wishbone,
                                         tie_rod=tie_rod,
                                         push_pull_rod=pushrod)

        double_wishbone.jounce(jounce=1)
        double_wishbone.steer(rack_translation=0)

        known_angle = -18.59 # Found using Solidworks
        self.assertEqual(round(double_wishbone.wheel_angle * 180 / np.pi, 2), known_angle)
    
    def test_combined_steer_five(self):
        # Nodes
        lower_inboard_fore = Node(position=[2, 0, 0])
        lower_inboard_aft = Node(position=[0, 0, 0])
        lower_outboard = Node(position=[1, 2, 0])

        upper_inboard_fore = Node(position=[2, 0, 1])
        upper_inboard_aft = Node(position=[0, 0, 1])
        upper_outboard = Node(position=[1, 2, 1])

        tie_inboard = Node(position=[2, 0, 0])
        tie_outboard = Node(position=[2, 2, 0.5])

        pushrod_lower = Node(position=[1, 2, 1])
        pushrod_upper = Node(position=[1, 0, 2])

        # Links
        lower_fore_link = Link(inboard_node=lower_inboard_fore, outboard_node=lower_outboard)
        lower_aft_link = Link(inboard_node=lower_inboard_aft, outboard_node=lower_outboard)

        upper_fore_link = Link(inboard_node=upper_inboard_fore, outboard_node=upper_outboard)
        upper_aft_link = Link(inboard_node=upper_inboard_aft, outboard_node=upper_outboard)

        tie_rod = Link(inboard_node=tie_inboard, outboard_node=tie_outboard)

        pushrod = Link(inboard_node=pushrod_upper, outboard_node=pushrod_lower)

        # Wishbones
        lower_wishbone = Wishbone(fore_link=lower_fore_link, aft_link=lower_aft_link)
        upper_wishbone = Wishbone(fore_link=upper_fore_link, aft_link=upper_aft_link)

        double_wishbone = QuarterCar(lower_wishbone=lower_wishbone,
                                         upper_wishbone=upper_wishbone,
                                         tie_rod=tie_rod,
                                         push_pull_rod=pushrod)

        double_wishbone.jounce(jounce=1)
        double_wishbone.steer(rack_translation=-0.25)

        known_angle = -35.45 # Found using Solidworks
        self.assertEqual(round(double_wishbone.wheel_angle * 180 / np.pi, 2), known_angle)
    
    def test_combined_steer_six(self):
        # Nodes
        lower_inboard_fore = Node(position=[2, 0, 0])
        lower_inboard_aft = Node(position=[0, 0, 0])
        lower_outboard = Node(position=[1, 2, 0])

        upper_inboard_fore = Node(position=[2, 0, 1])
        upper_inboard_aft = Node(position=[0, 0, 1])
        upper_outboard = Node(position=[1, 2, 1])

        tie_inboard = Node(position=[2, 0, 0])
        tie_outboard = Node(position=[2, 2, 0.5])

        pushrod_lower = Node(position=[1, 2, 1])
        pushrod_upper = Node(position=[1, 0, 2])

        # Links
        lower_fore_link = Link(inboard_node=lower_inboard_fore, outboard_node=lower_outboard)
        lower_aft_link = Link(inboard_node=lower_inboard_aft, outboard_node=lower_outboard)

        upper_fore_link = Link(inboard_node=upper_inboard_fore, outboard_node=upper_outboard)
        upper_aft_link = Link(inboard_node=upper_inboard_aft, outboard_node=upper_outboard)

        tie_rod = Link(inboard_node=tie_inboard, outboard_node=tie_outboard)

        pushrod = Link(inboard_node=pushrod_upper, outboard_node=pushrod_lower)

        # Wishbones
        lower_wishbone = Wishbone(fore_link=lower_fore_link, aft_link=lower_aft_link)
        upper_wishbone = Wishbone(fore_link=upper_fore_link, aft_link=upper_aft_link)

        double_wishbone = QuarterCar(lower_wishbone=lower_wishbone,
                                         upper_wishbone=upper_wishbone,
                                         tie_rod=tie_rod,
                                         push_pull_rod=pushrod)

        double_wishbone.jounce(jounce=-1)
        double_wishbone.steer(rack_translation=-0.25)

        known_angle = 1.03 # Found using Solidworks
        self.assertEqual(round(double_wishbone.wheel_angle * 180 / np.pi, 2), known_angle)
    
    def test_combined_steer_seven(self):
        # Nodes
        lower_inboard_fore = Node(position=[2, 0, 0])
        lower_inboard_aft = Node(position=[0, 0, 0])
        lower_outboard = Node(position=[1, 2, 0])

        upper_inboard_fore = Node(position=[2, 0, 1])
        upper_inboard_aft = Node(position=[0, 0, 1])
        upper_outboard = Node(position=[1, 2, 1])

        tie_inboard = Node(position=[2, 0, 0])
        tie_outboard = Node(position=[2, 2, 0.5])

        pushrod_lower = Node(position=[1, 2, 1])
        pushrod_upper = Node(position=[1, 0, 2])

        # Links
        lower_fore_link = Link(inboard_node=lower_inboard_fore, outboard_node=lower_outboard)
        lower_aft_link = Link(inboard_node=lower_inboard_aft, outboard_node=lower_outboard)

        upper_fore_link = Link(inboard_node=upper_inboard_fore, outboard_node=upper_outboard)
        upper_aft_link = Link(inboard_node=upper_inboard_aft, outboard_node=upper_outboard)

        tie_rod = Link(inboard_node=tie_inboard, outboard_node=tie_outboard)

        pushrod = Link(inboard_node=pushrod_upper, outboard_node=pushrod_lower)

        # Wishbones
        lower_wishbone = Wishbone(fore_link=lower_fore_link, aft_link=lower_aft_link)
        upper_wishbone = Wishbone(fore_link=upper_fore_link, aft_link=upper_aft_link)

        double_wishbone = QuarterCar(lower_wishbone=lower_wishbone,
                                         upper_wishbone=upper_wishbone,
                                         tie_rod=tie_rod,
                                         push_pull_rod=pushrod)

        double_wishbone.jounce(jounce=-1)
        double_wishbone.steer(rack_translation=0)

        known_angle = 15.52 # Found using Solidworks
        self.assertEqual(round(double_wishbone.wheel_angle * 180 / np.pi, 2), known_angle)

        ####################################################################################

    def test_combined_steer_eight(self):
        # Nodes
        lower_inboard_fore = Node(position=[2, 0, 0])
        lower_inboard_aft = Node(position=[0, 0, 0])
        lower_outboard = Node(position=[1, 2, 0])

        upper_inboard_fore = Node(position=[2, 0, 1])
        upper_inboard_aft = Node(position=[0, 0, 1])
        upper_outboard = Node(position=[1, 2, 1])

        tie_inboard = Node(position=[2, 0, 0])
        tie_outboard = Node(position=[2, 2, 0.5])

        pushrod_lower = Node(position=[1, 2, 1])
        pushrod_upper = Node(position=[1, 0, 2])

        # Links
        lower_fore_link = Link(inboard_node=lower_inboard_fore, outboard_node=lower_outboard)
        lower_aft_link = Link(inboard_node=lower_inboard_aft, outboard_node=lower_outboard)

        upper_fore_link = Link(inboard_node=upper_inboard_fore, outboard_node=upper_outboard)
        upper_aft_link = Link(inboard_node=upper_inboard_aft, outboard_node=upper_outboard)

        tie_rod = Link(inboard_node=tie_inboard, outboard_node=tie_outboard)

        pushrod = Link(inboard_node=pushrod_upper, outboard_node=pushrod_lower)

        # Wishbones
        lower_wishbone = Wishbone(fore_link=lower_fore_link, aft_link=lower_aft_link)
        upper_wishbone = Wishbone(fore_link=upper_fore_link, aft_link=upper_aft_link)

        double_wishbone = QuarterCar(lower_wishbone=lower_wishbone,
                                         upper_wishbone=upper_wishbone,
                                         tie_rod=tie_rod,
                                         push_pull_rod=pushrod)

        double_wishbone.steer(rack_translation=0.25)
        double_wishbone.jounce(jounce=0)

        known_angle = 14.46 # Found using Solidworks
        self.assertEqual(round(double_wishbone.wheel_angle * 180 / np.pi, 2), known_angle)
    
    def test_combined_steer_nine(self):
        # Nodes
        lower_inboard_fore = Node(position=[2, 0, 0])
        lower_inboard_aft = Node(position=[0, 0, 0])
        lower_outboard = Node(position=[1, 2, 0])

        upper_inboard_fore = Node(position=[2, 0, 1])
        upper_inboard_aft = Node(position=[0, 0, 1])
        upper_outboard = Node(position=[1, 2, 1])

        tie_inboard = Node(position=[2, 0, 0])
        tie_outboard = Node(position=[2, 2, 0.5])

        pushrod_lower = Node(position=[1, 2, 1])
        pushrod_upper = Node(position=[1, 0, 2])

        # Links
        lower_fore_link = Link(inboard_node=lower_inboard_fore, outboard_node=lower_outboard)
        lower_aft_link = Link(inboard_node=lower_inboard_aft, outboard_node=lower_outboard)

        upper_fore_link = Link(inboard_node=upper_inboard_fore, outboard_node=upper_outboard)
        upper_aft_link = Link(inboard_node=upper_inboard_aft, outboard_node=upper_outboard)

        tie_rod = Link(inboard_node=tie_inboard, outboard_node=tie_outboard)

        pushrod = Link(inboard_node=pushrod_upper, outboard_node=pushrod_lower)

        # Wishbones
        lower_wishbone = Wishbone(fore_link=lower_fore_link, aft_link=lower_aft_link)
        upper_wishbone = Wishbone(fore_link=upper_fore_link, aft_link=upper_aft_link)

        double_wishbone = QuarterCar(lower_wishbone=lower_wishbone,
                                         upper_wishbone=upper_wishbone,
                                         tie_rod=tie_rod,
                                         push_pull_rod=pushrod)

        double_wishbone.steer(rack_translation=0.25)
        double_wishbone.jounce(jounce=-1)

        known_angle = 30.86 # Found using Solidworks
        self.assertEqual(round(double_wishbone.wheel_angle * 180 / np.pi, 2), known_angle)
    
    def test_combined_steer_ten(self):
        # Nodes
        lower_inboard_fore = Node(position=[2, 0, 0])
        lower_inboard_aft = Node(position=[0, 0, 0])
        lower_outboard = Node(position=[1, 2, 0])

        upper_inboard_fore = Node(position=[2, 0, 1])
        upper_inboard_aft = Node(position=[0, 0, 1])
        upper_outboard = Node(position=[1, 2, 1])

        tie_inboard = Node(position=[2, 0, 0])
        tie_outboard = Node(position=[2, 2, 0.5])

        pushrod_lower = Node(position=[1, 2, 1])
        pushrod_upper = Node(position=[1, 0, 2])

        # Links
        lower_fore_link = Link(inboard_node=lower_inboard_fore, outboard_node=lower_outboard)
        lower_aft_link = Link(inboard_node=lower_inboard_aft, outboard_node=lower_outboard)

        upper_fore_link = Link(inboard_node=upper_inboard_fore, outboard_node=upper_outboard)
        upper_aft_link = Link(inboard_node=upper_inboard_aft, outboard_node=upper_outboard)

        tie_rod = Link(inboard_node=tie_inboard, outboard_node=tie_outboard)

        pushrod = Link(inboard_node=pushrod_upper, outboard_node=pushrod_lower)

        # Wishbones
        lower_wishbone = Wishbone(fore_link=lower_fore_link, aft_link=lower_aft_link)
        upper_wishbone = Wishbone(fore_link=upper_fore_link, aft_link=upper_aft_link)

        double_wishbone = QuarterCar(lower_wishbone=lower_wishbone,
                                         upper_wishbone=upper_wishbone,
                                         tie_rod=tie_rod,
                                         push_pull_rod=pushrod)

        double_wishbone.steer(rack_translation=0.25)
        double_wishbone.jounce(jounce=1)

        known_angle = -3.89 # Found using Solidworks
        self.assertEqual(round(double_wishbone.wheel_angle * 180 / np.pi, 2), known_angle)
    
    def test_combined_steer_eleven(self):
        # Nodes
        lower_inboard_fore = Node(position=[2, 0, 0])
        lower_inboard_aft = Node(position=[0, 0, 0])
        lower_outboard = Node(position=[1, 2, 0])

        upper_inboard_fore = Node(position=[2, 0, 1])
        upper_inboard_aft = Node(position=[0, 0, 1])
        upper_outboard = Node(position=[1, 2, 1])

        tie_inboard = Node(position=[2, 0, 0])
        tie_outboard = Node(position=[2, 2, 0.5])

        pushrod_lower = Node(position=[1, 2, 1])
        pushrod_upper = Node(position=[1, 0, 2])

        # Links
        lower_fore_link = Link(inboard_node=lower_inboard_fore, outboard_node=lower_outboard)
        lower_aft_link = Link(inboard_node=lower_inboard_aft, outboard_node=lower_outboard)

        upper_fore_link = Link(inboard_node=upper_inboard_fore, outboard_node=upper_outboard)
        upper_aft_link = Link(inboard_node=upper_inboard_aft, outboard_node=upper_outboard)

        tie_rod = Link(inboard_node=tie_inboard, outboard_node=tie_outboard)

        pushrod = Link(inboard_node=pushrod_upper, outboard_node=pushrod_lower)

        # Wishbones
        lower_wishbone = Wishbone(fore_link=lower_fore_link, aft_link=lower_aft_link)
        upper_wishbone = Wishbone(fore_link=upper_fore_link, aft_link=upper_aft_link)

        double_wishbone = QuarterCar(lower_wishbone=lower_wishbone,
                                         upper_wishbone=upper_wishbone,
                                         tie_rod=tie_rod,
                                         push_pull_rod=pushrod)

        double_wishbone.steer(rack_translation=0)
        double_wishbone.jounce(jounce=1)

        known_angle = -18.59 # Found using Solidworks
        self.assertEqual(round(double_wishbone.wheel_angle * 180 / np.pi, 2), known_angle)
    
    def test_combined_steer_twelve(self):
        # Nodes
        lower_inboard_fore = Node(position=[2, 0, 0])
        lower_inboard_aft = Node(position=[0, 0, 0])
        lower_outboard = Node(position=[1, 2, 0])

        upper_inboard_fore = Node(position=[2, 0, 1])
        upper_inboard_aft = Node(position=[0, 0, 1])
        upper_outboard = Node(position=[1, 2, 1])

        tie_inboard = Node(position=[2, 0, 0])
        tie_outboard = Node(position=[2, 2, 0.5])

        pushrod_lower = Node(position=[1, 2, 1])
        pushrod_upper = Node(position=[1, 0, 2])

        # Links
        lower_fore_link = Link(inboard_node=lower_inboard_fore, outboard_node=lower_outboard)
        lower_aft_link = Link(inboard_node=lower_inboard_aft, outboard_node=lower_outboard)

        upper_fore_link = Link(inboard_node=upper_inboard_fore, outboard_node=upper_outboard)
        upper_aft_link = Link(inboard_node=upper_inboard_aft, outboard_node=upper_outboard)

        tie_rod = Link(inboard_node=tie_inboard, outboard_node=tie_outboard)

        pushrod = Link(inboard_node=pushrod_upper, outboard_node=pushrod_lower)

        # Wishbones
        lower_wishbone = Wishbone(fore_link=lower_fore_link, aft_link=lower_aft_link)
        upper_wishbone = Wishbone(fore_link=upper_fore_link, aft_link=upper_aft_link)

        double_wishbone = QuarterCar(lower_wishbone=lower_wishbone,
                                         upper_wishbone=upper_wishbone,
                                         tie_rod=tie_rod,
                                         push_pull_rod=pushrod)

        double_wishbone.steer(rack_translation=-0.25)
        double_wishbone.jounce(jounce=1)

        known_angle = -35.45 # Found using Solidworks
        self.assertEqual(round(double_wishbone.wheel_angle * 180 / np.pi, 2), known_angle)
    
    def test_combined_steer_thirteen(self):
        # Nodes
        lower_inboard_fore = Node(position=[2, 0, 0])
        lower_inboard_aft = Node(position=[0, 0, 0])
        lower_outboard = Node(position=[1, 2, 0])

        upper_inboard_fore = Node(position=[2, 0, 1])
        upper_inboard_aft = Node(position=[0, 0, 1])
        upper_outboard = Node(position=[1, 2, 1])

        tie_inboard = Node(position=[2, 0, 0])
        tie_outboard = Node(position=[2, 2, 0.5])

        pushrod_lower = Node(position=[1, 2, 1])
        pushrod_upper = Node(position=[1, 0, 2])

        # Links
        lower_fore_link = Link(inboard_node=lower_inboard_fore, outboard_node=lower_outboard)
        lower_aft_link = Link(inboard_node=lower_inboard_aft, outboard_node=lower_outboard)

        upper_fore_link = Link(inboard_node=upper_inboard_fore, outboard_node=upper_outboard)
        upper_aft_link = Link(inboard_node=upper_inboard_aft, outboard_node=upper_outboard)

        tie_rod = Link(inboard_node=tie_inboard, outboard_node=tie_outboard)

        pushrod = Link(inboard_node=pushrod_upper, outboard_node=pushrod_lower)

        # Wishbones
        lower_wishbone = Wishbone(fore_link=lower_fore_link, aft_link=lower_aft_link)
        upper_wishbone = Wishbone(fore_link=upper_fore_link, aft_link=upper_aft_link)

        double_wishbone = QuarterCar(lower_wishbone=lower_wishbone,
                                         upper_wishbone=upper_wishbone,
                                         tie_rod=tie_rod,
                                         push_pull_rod=pushrod)

        double_wishbone.steer(rack_translation=-0.25)
        double_wishbone.jounce(jounce=-1)

        known_angle = 1.03 # Found using Solidworks
        self.assertEqual(round(double_wishbone.wheel_angle * 180 / np.pi, 2), known_angle)
    
    def test_combined_steer_fourteen(self):
        # Nodes
        lower_inboard_fore = Node(position=[2, 0, 0])
        lower_inboard_aft = Node(position=[0, 0, 0])
        lower_outboard = Node(position=[1, 2, 0])

        upper_inboard_fore = Node(position=[2, 0, 1])
        upper_inboard_aft = Node(position=[0, 0, 1])
        upper_outboard = Node(position=[1, 2, 1])

        tie_inboard = Node(position=[2, 0, 0])
        tie_outboard = Node(position=[2, 2, 0.5])

        pushrod_lower = Node(position=[1, 2, 1])
        pushrod_upper = Node(position=[1, 0, 2])

        # Links
        lower_fore_link = Link(inboard_node=lower_inboard_fore, outboard_node=lower_outboard)
        lower_aft_link = Link(inboard_node=lower_inboard_aft, outboard_node=lower_outboard)

        upper_fore_link = Link(inboard_node=upper_inboard_fore, outboard_node=upper_outboard)
        upper_aft_link = Link(inboard_node=upper_inboard_aft, outboard_node=upper_outboard)

        tie_rod = Link(inboard_node=tie_inboard, outboard_node=tie_outboard)

        pushrod = Link(inboard_node=pushrod_upper, outboard_node=pushrod_lower)

        # Wishbones
        lower_wishbone = Wishbone(fore_link=lower_fore_link, aft_link=lower_aft_link)
        upper_wishbone = Wishbone(fore_link=upper_fore_link, aft_link=upper_aft_link)

        double_wishbone = QuarterCar(lower_wishbone=lower_wishbone,
                                         upper_wishbone=upper_wishbone,
                                         tie_rod=tie_rod,
                                         push_pull_rod=pushrod)

        double_wishbone.steer(rack_translation=0)
        double_wishbone.jounce(jounce=-1)

        known_angle = 15.52 # Found using Solidworks
        self.assertEqual(round(double_wishbone.wheel_angle * 180 / np.pi, 2), known_angle)