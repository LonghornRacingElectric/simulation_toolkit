from _4_custom_libraries.misc_math import unit_vec, nearest_root, directional_root
import numpy as np

from unittest import TestCase


class TestMiscMath(TestCase):
    def test_unit_vec(self):
        p1=[0, 0, 0]
        p2=[1, 1, 1]
        
        in_vec_known = [(p2[i] - val) / np.sqrt(sum([(p2[i] - val)**2 for i, val in enumerate(p1)])) for i, val in enumerate(p1)]
        in_vec_check = list(unit_vec(p1=p1, p2=p2))
        self.assertListEqual(in_vec_known, in_vec_check)
    
    def test_nearest_root_one(self):
        f = lambda x, args: x - 0.2

        soln = nearest_root(func=f, x0=1, bounds=(-5, 5), tol=1e-8)

        self.assertEqual(round(soln, 7), 0.2)

    def test_nearest_root_two(self):
        f = lambda x, args: x - 0.2

        soln = nearest_root(func=f, x0=-1, bounds=(-5, 5), tol=1e-8)

        self.assertEqual(round(soln, 7), 0.2)
    
    def test_nearest_root_three(self):
        f = lambda x, args: x + 0.2

        soln = nearest_root(func=f, x0=-1, bounds=(-5, 5), tol=1e-8)

        self.assertEqual(round(soln, 7), -0.2)
    
    def test_nearest_root_four(self):
        f = lambda x, args: x + 0.2

        soln = nearest_root(func=f, x0=1, bounds=(-5, 5), tol=1e-8)

        self.assertEqual(round(soln, 7), -0.2)
    
    def test_nearest_root_five(self):
        f = lambda x, args: x - 0.2

        soln = nearest_root(func=f, x0=0, bounds=(-5, 5), tol=1e-8)

        self.assertEqual(round(soln, 7), 0.2)
    
    def test_nearest_root_six(self):
        f = lambda x, args: x + 0.2

        soln = nearest_root(func=f, x0=0, bounds=(-5, 5), tol=1e-8)

        self.assertEqual(round(soln, 7), -0.2)
    
    def test_directional_root_positive(self):
        f = lambda x, args: (x - 5)**2 - 25**2

        soln = directional_root(func=f, x0=0, bounds=(0, 100), tol=1e-10)

        self.assertEqual(round(soln, 7), 30)
    
    def test_directional_root_negative(self):
        f = lambda x, args: (x - 5)**2 - 25**2

        soln = directional_root(func=f, x0=0, bounds=(-100, 0), tol=1e-10)

        self.assertEqual(round(soln, 7), -20)