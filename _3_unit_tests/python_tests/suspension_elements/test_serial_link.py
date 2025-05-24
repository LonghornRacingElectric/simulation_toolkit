from vehicle_model.suspension_model.suspension_elements._3_elements.serial_link import SerialLink
from vehicle_model.suspension_model.suspension_elements._2_elements.spring import Spring
from vehicle_model.suspension_model.suspension_elements._1_elements.link import Link
from vehicle_model.suspension_model.suspension_elements._1_elements.node import Node

from unittest import TestCase
import numpy as np


class TestSerialLink(TestCase):
    def test_serial_link_coords_one(self):
        rigid_outboard = Node(position=[0, 2, 0])
        shared_node = Node(position=[0, 1, 0])
        compliant_inboard = Node(position=[0, 0, 0])

        rigid_link = Link(inboard_node=shared_node, outboard_node=rigid_outboard)
        compliant_link = Spring(inboard_node=compliant_inboard, outboard_node=shared_node, free_length=1, rate=1)

        serial_link = SerialLink(rigid_link=rigid_link, compliant_link=compliant_link)

        rigid_outboard.translate(translation=[0, -0.5, 0])

        self.assertEqual(shared_node[1], 0.5)

    def test_serial_link_coords_two(self):
        rigid_outboard = Node(position=[0, 2, 0])
        shared_node = Node(position=[0, 1, 0])
        compliant_inboard = Node(position=[0, 0, 0])

        rigid_link = Link(inboard_node=shared_node, outboard_node=rigid_outboard)
        compliant_link = Spring(inboard_node=compliant_inboard, outboard_node=shared_node, free_length=1, rate=1)

        serial_link = SerialLink(rigid_link=rigid_link, compliant_link=compliant_link)

        rigid_outboard.translate(translation=[0, 0, 1])

        angle = np.atan(1/2)
        compliant_length = np.sqrt(5) - 1

        self.assertListEqual(shared_node.position, [0, compliant_length * np.cos(angle), compliant_length * np.sin(angle)])

    def test_serial_link_rigid(self):
        rigid_outboard = Node(position=[0, 2, 0])
        shared_node = Node(position=[0, 1, 0])
        compliant_inboard = Node(position=[0, 0, 0])

        rigid_link = Link(inboard_node=shared_node, outboard_node=rigid_outboard)
        compliant_link = Spring(inboard_node=compliant_inboard, outboard_node=shared_node, free_length=1, rate=1)

        serial_link = SerialLink(rigid_link=rigid_link, compliant_link=compliant_link)

        rigid_outboard.translate(translation=[0, -0.5, 0])

        self.assertEqual(rigid_link.length, rigid_link.initial_length)
    
    def test_serial_link_compliant(self):
        rigid_outboard = Node(position=[0, 2, 0])
        shared_node = Node(position=[0, 1, 0])
        compliant_inboard = Node(position=[0, 0, 0])

        rigid_link = Link(inboard_node=shared_node, outboard_node=rigid_outboard)
        compliant_link = Spring(inboard_node=compliant_inboard, outboard_node=shared_node, free_length=1, rate=1)

        serial_link = SerialLink(rigid_link=rigid_link, compliant_link=compliant_link)

        rigid_outboard.translate(translation=[0, -0.5, 0])

        self.assertNotEqual(compliant_link.length, compliant_link.initial_length)
    
    def test_serial_link_angled_rigid(self):
        rigid_outboard = Node(position=[0, 2, 0])
        shared_node = Node(position=[0, 1, 0])
        compliant_inboard = Node(position=[0, 0, 0])

        rigid_link = Link(inboard_node=shared_node, outboard_node=rigid_outboard)
        compliant_link = Spring(inboard_node=compliant_inboard, outboard_node=shared_node, free_length=1, rate=1)

        serial_link = SerialLink(rigid_link=rigid_link, compliant_link=compliant_link)

        rigid_outboard.translate(translation=[0, 0, 1])

        self.assertEqual(round(rigid_link.length, 7), round(rigid_link.initial_length, 7))
    
    def test_serial_link_angled_compliant(self):
        rigid_outboard = Node(position=[0, 2, 0])
        shared_node = Node(position=[0, 1, 0])
        compliant_inboard = Node(position=[0, 0, 0])

        rigid_link = Link(inboard_node=shared_node, outboard_node=rigid_outboard)
        compliant_link = Spring(inboard_node=compliant_inboard, outboard_node=shared_node, free_length=1, rate=1)

        serial_link = SerialLink(rigid_link=rigid_link, compliant_link=compliant_link)

        rigid_outboard.translate(translation=[0, 0, 1])

        self.assertEqual(round(compliant_link.length, 7), round(np.sqrt(5) - 1, 7))