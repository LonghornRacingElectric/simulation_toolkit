from vehicle_model.assets.misc_linalg import unit_vec, rotation_matrix
import numpy as np

from unittest import main, TestCase


class NodeTest(TestCase):
    def test_unit_vec(self):
        p1=[0, 0, 0]
        p2=[1, 1, 1]
        
        in_vec_known = [(p2[i] - val) / np.sqrt(sum([(p2[i] - val)**2 for i, val in enumerate(p1)])) for i, val in enumerate(p1)]
        in_vec_check = list(unit_vec(p1=p1, p2=p2))
        self.assertListEqual(in_vec_known, in_vec_check)