# from src.vehicle_model.suspension_model.suspension_data import SuspensionData
# from src.vehicle_model.suspension_model.suspension import Suspension

# import numpy as np
# import warnings

# from unittest import TestCase


# class TestSuspension(TestCase):
#     def test_init(self):
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)
    
#     def test_static_delta(self):
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         known_results = [0, 0, 0, 0]
#         test_results = [sus.FL_delta,
#                         sus.FR_delta,
#                         sus.RL_delta,
#                         sus.RR_delta]

#         corresponding_values = ["FL_delta",
#                                 "FR_delta",
#                                 "RL_delta",
#                                 "RR_delta"]
        
#         tolerances = [0.01,
#                       0.01,
#                       0.01,
#                       0.01]

#         for i in range(len(known_results)):
#             with self.subTest(i=i):
#                 self.assertLess(abs(test_results[i] - known_results[i]), tolerances[i], msg=corresponding_values[i])
    
#     def test_static_gamma(self):
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         known_results = [1, -1, 0, 0]
#         test_results = [sus.FL_gamma,
#                         sus.FR_gamma,
#                         sus.RL_gamma,
#                         sus.RR_gamma]

#         corresponding_values = ["FL_gamma",
#                                 "FR_gamma",
#                                 "RL_gamma",
#                                 "RR_gamma"]
        
#         tolerances = [0.01,
#                       0.01,
#                       0.01,
#                       0.01]

#         for i in range(len(known_results)):
#             with self.subTest(i=i):
#                 self.assertLess(abs(test_results[i] - known_results[i]), tolerances[i], msg=corresponding_values[i])
    
#     def test_static_caster(self):
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         known_results = [2.2435, 2.2435, 6.4043, 6.4043]
#         test_results = [sus.FL_caster,
#                         sus.FR_caster,
#                         sus.RL_caster,
#                         sus.RR_caster]

#         corresponding_values = ["FL_caster",
#                                 "FR_caster",
#                                 "RL_caster",
#                                 "RR_caster"]
        
#         tolerances = [0.01,
#                       0.01,
#                       0.01,
#                       0.01]

#         for i in range(len(known_results)):
#             with self.subTest(i=i):
#                 self.assertLess(abs(test_results[i] - known_results[i]), tolerances[i], msg=corresponding_values[i])
    
#     def test_static_kpi(self):
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         known_results = [11.6002, 11.6002, 8.0610, 8.0610]
#         test_results = [sus.FL_kpi,
#                         sus.FR_kpi,
#                         sus.RL_kpi,
#                         sus.RR_kpi]

#         corresponding_values = ["FL_kpi",
#                                 "FR_kpi",
#                                 "RL_kpi",
#                                 "RR_kpi"]
        
#         tolerances = [0.01,
#                       0.01,
#                       0.01,
#                       0.01]

#         for i in range(len(known_results)):
#             with self.subTest(i=i):
#                 self.assertLess(abs(test_results[i] - known_results[i]), tolerances[i], msg=corresponding_values[i])
    
#     def test_static_scrub(self):
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         known_results = [float(x) * 0.0254 for x in [1.0804, 1.0804, 0.5352, 0.5352]]
#         test_results = [sus.FL_scrub,
#                         sus.FR_scrub,
#                         sus.RL_scrub,
#                         sus.RR_scrub]

#         corresponding_values = ["FL_scrub",
#                                 "FR_scrub",
#                                 "RL_scrub",
#                                 "RR_scrub"]
        
#         tolerances = [0.001,
#                       0.001,
#                       0.001,
#                       0.001]

#         for i in range(len(known_results)):
#             with self.subTest(i=i):
#                 self.assertLess(abs(test_results[i] - known_results[i]), tolerances[i], msg=corresponding_values[i])
    
#     def test_static_mech_trail(self):
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         known_results = [float(x) * 0.0254 for x in [0.1928, 0.1928, 0.2797, 0.2797]]
#         test_results = [sus.FL_mech_trail,
#                         sus.FR_mech_trail,
#                         sus.RL_mech_trail,
#                         sus.RR_mech_trail]

#         corresponding_values = ["FL_mech_trail",
#                                 "FR_mech_trail",
#                                 "RL_mech_trail",
#                                 "RR_mech_trail"]
        
#         tolerances = [0.001,
#                       0.001,
#                       0.001,
#                       0.001]

#         for i in range(len(known_results)):
#             with self.subTest(i=i):
#                 self.assertLess(abs(test_results[i] - known_results[i]), tolerances[i], msg=corresponding_values[i])
    
#     def test_static_FVIC(self):
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         known_results = [[float(x) for x in np.array(y) * 0.0254] for y in [[0, -15.5082, 1.0763], [0, 15.5082, 1.0763], [-61, -15.6465, 1.4661], [-61, 15.6465, 1.4661]]]
#         test_results = [sus.FL_FVIC,
#                         sus.FR_FVIC,
#                         sus.RL_FVIC,
#                         sus.RR_FVIC]

#         corresponding_values = ["FL_FVIC",
#                                 "FR_FVIC",
#                                 "RL_FVIC",
#                                 "RR_FVIC"]
        
#         tolerances = [0.001,
#                       0.001,
#                       0.001,
#                       0.001]

#         for i in range(len(known_results)):
#             with self.subTest(i=i):
#                 self.assertLess(np.linalg.norm(np.array(test_results[i]) - np.array(known_results[i])), tolerances[i], msg=corresponding_values[i])
    
#     def test_bump_steer_positive(self):
#         warnings.warn("Bump steer tests aren't valid since the unit test vehicle has no bump steer")
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         jounce_vals = [float(x) for x in np.array([0.5, 1]) * 0.0254]

#         known_results = [[0, 0, 0, 0],
#                          [0, 0, 0, 0]]
        
#         for i in range(len(jounce_vals)):
#             with self.subTest(i=i):
#                 sus.heave(heave=None)
#                 sus.heave(heave=jounce_vals[i])
#                 test_results = [sus.FL_delta,
#                                 sus.FR_delta,
#                                 sus.RL_delta,
#                                 sus.RR_delta]

#                 corresponding_values = ["FL_delta",
#                                         "FR_delta",
#                                         "RL_delta",
#                                         "RR_delta"]
        
#                 tolerances = [0.1,
#                               0.1,
#                               0.1,
#                               0.1]

#                 for j in range(len(test_results)):
#                     with self.subTest(i=j):
#                         self.assertLess(abs(test_results[j] - known_results[i][j]), tolerances[j], msg=corresponding_values[j])
    
#     def test_bump_steer_negative(self):
#         warnings.warn("Bump steer tests aren't valid since the unit test vehicle has no bump steer")
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         jounce_vals = [float(x) for x in np.array([-1, -0.5]) * 0.0254]

#         known_results = [[0, 0, 0, 0],
#                          [0, 0, 0, 0]]
        
#         for i in range(len(jounce_vals)):
#             with self.subTest(i=i):
#                 sus.heave(heave=None)
#                 sus.heave(heave=jounce_vals[i])
#                 test_results = [sus.FL_delta,
#                                 sus.FR_delta,
#                                 sus.RL_delta,
#                                 sus.RR_delta]

#                 corresponding_values = ["FL_delta",
#                                         "FR_delta",
#                                         "RL_delta",
#                                         "RR_delta"]
        
#                 tolerances = [0.1,
#                               0.1,
#                               0.1,
#                               0.1]

#                 for j in range(len(test_results)):
#                     with self.subTest(i=j):
#                         self.assertLess(abs(test_results[j] - known_results[i][j]), tolerances[j], msg=corresponding_values[j])
    
#     def test_bump_gamma_positive(self):
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         jounce_vals = [float(x) for x in np.array([0.5, 1]) * 0.0254]

#         known_results = [[1.7341, -1.7341, 0.7327, -0.7327],
#                          [2.4826, -2.4826, 1.4827, -1.4827]]
        
#         for i in range(len(jounce_vals)):
#             with self.subTest(i=i):
#                 sus.heave(heave=None)
#                 sus.heave(heave=jounce_vals[i])
#                 test_results = [sus.FL_gamma,
#                                 sus.FR_gamma,
#                                 sus.RL_gamma,
#                                 sus.RR_gamma]

#                 corresponding_values = ["FL_gamma",
#                                         "FR_gamma",
#                                         "RL_gamma",
#                                         "RR_gamma"]
        
#                 tolerances = [0.01,
#                               0.01,
#                               0.01,
#                               0.01]

#                 for j in range(len(test_results)):
#                     with self.subTest(i=j):
#                         self.assertLess(abs(test_results[j] - known_results[i][j]), tolerances[j], msg=corresponding_values[j])

#     def test_bump_gamma_negative(self):
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         jounce_vals = [float(x) for x in np.array([-1, -0.5]) * 0.0254]

#         known_results = [[-0.4331, 0.4331, -1.4257, 1.4257],
#                          [0.2782, -0.2782, -0.7185, 0.7185]]
        
#         for i in range(len(jounce_vals)):
#             with self.subTest(i=i):
#                 sus.heave(heave=None)
#                 sus.heave(heave=jounce_vals[i])
#                 test_results = [sus.FL_gamma,
#                                 sus.FR_gamma,
#                                 sus.RL_gamma,
#                                 sus.RR_gamma]

#                 corresponding_values = ["FL_gamma",
#                                         "FR_gamma",
#                                         "RL_gamma",
#                                         "RR_gamma"]
        
#                 tolerances = [0.01,
#                               0.01,
#                               0.01,
#                               0.01]

#                 for j in range(len(test_results)):
#                     with self.subTest(i=j):
#                         self.assertLess(abs(test_results[j] - known_results[i][j]), tolerances[j], msg=corresponding_values[j])
    
#     def test_bump_caster_positive(self):
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         jounce_vals = [float(x) for x in np.array([0.5, 1]) * 0.0254]

#         known_results = [[2.2496, 2.2496, 6.4164, 6.4164],
#                          [2.2562, 2.2562, 6.4298, 6.4298]]
        
#         for i in range(len(jounce_vals)):
#             with self.subTest(i=i):
#                 sus.heave(heave=None)
#                 sus.heave(heave=jounce_vals[i])
#                 test_results = [sus.FL_caster,
#                                 sus.FR_caster,
#                                 sus.RL_caster,
#                                 sus.RR_caster]

#                 corresponding_values = ["FL_caster",
#                                         "FR_caster",
#                                         "RL_caster",
#                                         "RR_caster"]
        
#                 tolerances = [0.01,
#                               0.01,
#                               0.01,
#                               0.01]

#                 for j in range(len(test_results)):
#                     with self.subTest(i=j):
#                         self.assertLess(abs(test_results[j] - known_results[i][j]), tolerances[j], msg=corresponding_values[j])
    
#     def test_bump_caster_negative(self):
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         jounce_vals = [float(x) for x in np.array([-1, -0.5]) * 0.0254]

#         known_results = [[2.2328, 2.2328, 6.3840, 6.3840],
#                          [2.2379, 2.2379, 6.3936, 6.3936]]
        
#         for i in range(len(jounce_vals)):
#             with self.subTest(i=i):
#                 sus.heave(heave=None)
#                 sus.heave(heave=jounce_vals[i])
#                 test_results = [sus.FL_caster,
#                                 sus.FR_caster,
#                                 sus.RL_caster,
#                                 sus.RR_caster]

#                 corresponding_values = ["FL_caster",
#                                         "FR_caster",
#                                         "RL_caster",
#                                         "RR_caster"]
        
#                 tolerances = [0.01,
#                               0.01,
#                               0.01,
#                               0.01]

#                 for j in range(len(test_results)):
#                     with self.subTest(i=j):
#                         self.assertLess(abs(test_results[j] - known_results[i][j]), tolerances[j], msg=corresponding_values[j])
    
#     def test_bump_kpi_positive(self):
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         jounce_vals = [float(x) for x in np.array([0.5, 1]) * 0.0254]

#         known_results = [[12.3343, 12.3343, 8.7936, 8.7936],
#                          [13.0828, 13.0828, 9.5436, 9.5436]]
        
#         for i in range(len(jounce_vals)):
#             with self.subTest(i=i):
#                 sus.heave(heave=None)
#                 sus.heave(heave=jounce_vals[i])
#                 test_results = [sus.FL_kpi,
#                                 sus.FR_kpi,
#                                 sus.RL_kpi,
#                                 sus.RR_kpi]

#                 corresponding_values = ["FL_kpi",
#                                         "FR_kpi",
#                                         "RL_kpi",
#                                         "RR_kpi"]
        
#                 tolerances = [0.01,
#                               0.01,
#                               0.01,
#                               0.01]

#                 for j in range(len(test_results)):
#                     with self.subTest(i=j):
#                         self.assertLess(abs(test_results[j] - known_results[i][j]), tolerances[j], msg=corresponding_values[j])

#     def test_bump_kpi_negative(self):
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         jounce_vals = [float(x) for x in np.array([-1, -0.5]) * 0.0254]

#         known_results = [[10.1670, 10.1670, 6.6353, 6.6353],
#                          [10.8784, 10.8784, 7.3425, 7.3425]]
        
#         for i in range(len(jounce_vals)):
#             with self.subTest(i=i):
#                 sus.heave(heave=None)
#                 sus.heave(heave=jounce_vals[i])
#                 test_results = [sus.FL_kpi,
#                                 sus.FR_kpi,
#                                 sus.RL_kpi,
#                                 sus.RR_kpi]

#                 corresponding_values = ["FL_kpi",
#                                         "FR_kpi",
#                                         "RL_kpi",
#                                         "RR_kpi"]
        
#                 tolerances = [0.01,
#                               0.01,
#                               0.01,
#                               0.01]

#                 for j in range(len(test_results)):
#                     with self.subTest(i=j):
#                         self.assertLess(abs(test_results[j] - known_results[i][j]), tolerances[j], msg=corresponding_values[j])
    
#     def test_bump_scrub_positive(self):
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         jounce_vals = [float(x) for x in np.array([0.5, 1]) * 0.0254]

#         known_results = [[float(x) * 0.0254 for x in [1.07736, 1.07736, 0.53593, 0.53593]],
#                          [float(x) * 0.0254 for x in [1.09062, 1.09062, 0.53674, 0.53674]]]
        
#         for i in range(len(jounce_vals)):
#             with self.subTest(i=i):
#                 sus.heave(heave=None)
#                 sus.heave(heave=jounce_vals[i])
#                 test_results = [sus.FL_scrub,
#                                 sus.FR_scrub,
#                                 sus.RL_scrub,
#                                 sus.RR_scrub]

#                 corresponding_values = ["FL_scrub",
#                                         "FR_scrub",
#                                         "RL_scrub",
#                                         "RR_scrub"]
        
#                 tolerances = [0.001,
#                               0.001,
#                               0.001,
#                               0.001]

#                 for j in range(len(test_results)):
#                     with self.subTest(i=j):
#                         self.assertLess(abs(test_results[j] - known_results[i][j]), tolerances[j], msg=corresponding_values[j])

#     def test_bump_scrub_negative(self):
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         jounce_vals = [float(x) for x in np.array([-1, -0.5]) * 0.0254]

#         known_results = [[float(x) * 0.0254 for x in [1.075915, 1.075915, 0.533730, 0.533730]],
#                          [float(x) * 0.0254 for x in [1.078042, 1.078042, 0.534433, 0.534433]]]
        
#         for i in range(len(jounce_vals)):
#             with self.subTest(i=i):
#                 sus.heave(heave=None)
#                 sus.heave(heave=jounce_vals[i])
#                 test_results = [sus.FL_scrub,
#                                 sus.FR_scrub,
#                                 sus.RL_scrub,
#                                 sus.RR_scrub]

#                 corresponding_values = ["FL_scrub",
#                                         "FR_scrub",
#                                         "RL_scrub",
#                                         "RR_scrub"]
        
#                 tolerances = [0.001,
#                               0.001,
#                               0.001,
#                               0.001]

#                 for j in range(len(test_results)):
#                     with self.subTest(i=j):
#                         self.assertLess(abs(test_results[j] - known_results[i][j]), tolerances[j], msg=corresponding_values[j])
    
#     def test_bump_mech_trail_positive(self):
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         jounce_vals = [float(x) for x in np.array([0.5, 1]) * 0.0254]

#         known_results = [[float(x) * 0.0254 for x in [0.192321, 0.192321, 0.279269, 0.279269]],
#                          [float(x) * 0.0254 for x in [0.191834, 0.191834, 0.279195, 0.279195]]]
        
#         for i in range(len(jounce_vals)):
#             with self.subTest(i=i):
#                 sus.heave(heave=None)
#                 sus.heave(heave=jounce_vals[i])
#                 test_results = [sus.FL_mech_trail,
#                                 sus.FR_mech_trail,
#                                 sus.RL_mech_trail,
#                                 sus.RR_mech_trail]

#                 corresponding_values = ["FL_mech_trail",
#                                         "FR_mech_trail",
#                                         "RL_mech_trail",
#                                         "RR_mech_trail"]
        
#                 tolerances = [0.001,
#                               0.001,
#                               0.001,
#                               0.001]

#                 for j in range(len(test_results)):
#                     with self.subTest(i=j):
#                         self.assertLess(abs(test_results[j] - known_results[i][j]), tolerances[j], msg=corresponding_values[j])

#     def test_bump_mech_trail_negative(self):
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         jounce_vals = [float(x) for x in np.array([-1, -0.5]) * 0.0254]

#         known_results = [[float(x) * 0.0254 for x in [0.193715, 0.193715, 0.281543, 0.281543]],
#                          [float(x) * 0.0254 for x in [0.193260, 0.193260, 0.280447, 0.280447]]]
        
#         for i in range(len(jounce_vals)):
#             with self.subTest(i=i):
#                 sus.heave(heave=None)
#                 sus.heave(heave=jounce_vals[i])
#                 test_results = [sus.FL_mech_trail,
#                                 sus.FR_mech_trail,
#                                 sus.RL_mech_trail,
#                                 sus.RR_mech_trail]

#                 corresponding_values = ["FL_mech_trail",
#                                         "FR_mech_trail",
#                                         "RL_mech_trail",
#                                         "RR_mech_trail"]
        
#                 tolerances = [0.001,
#                               0.001,
#                               0.001,
#                               0.001]

#                 for j in range(len(test_results)):
#                     with self.subTest(i=j):
#                         self.assertLess(abs(test_results[j] - known_results[i][j]), tolerances[j], msg=corresponding_values[j])
    
#     def test_bump_FVIC_positive(self):
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         jounce_vals = [float(x) for x in np.array([0.5, 1]) * 0.0254]

#         for i in range(len(jounce_vals)):
#             with self.subTest(i=i):
#                 sus.heave(heave=None)
#                 sus.heave(heave=jounce_vals[i])
#                 known_results = [np.array(x) * 0.0254 for x in [[[0, -14.793533, 0.311568 - 0.5], [0, 14.793533, 0.311568 - 0.5], [-61, -14.801520, 0.439463 - 0.5], [-61, 14.801520, 0.439463 - 0.5]],
#                                                                 [[0, -14.016735, -0.400640 - 1], [0, 14.016735, -0.400640 - 1], [-61, -13.851012, -0.512532 - 1], [-61, 13.851012, -0.512532 - 1]]]]
#                 test_results = [sus.FL_FVIC,
#                                 sus.FR_FVIC,
#                                 sus.RL_FVIC,
#                                 sus.RR_FVIC]

#                 corresponding_values = ["FL_FVIC",
#                                         "FR_FVIC",
#                                         "RL_FVIC",
#                                         "RR_FVIC"]
        
#                 tolerances = [0.001,
#                               0.001,
#                               0.001,
#                               0.001]

#                 for j in range(len(test_results)):
#                     with self.subTest(i=j):
#                         # 2D projection doesn't account for CP_x migration, so don't check x-values (the math here is known to be more accurate)
#                         self.assertLess(np.linalg.norm(np.array(test_results[j][1:]) - np.array(known_results[i][j][1:])), 
#                                         tolerances[j], msg=f"{corresponding_values[j]} |\nTest:\n{test_results[j][1:]}\nKnown:\n{known_results[i][j][1:]}")

#     def test_bump_FVIC_negative(self):
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         jounce_vals = [float(x) for x in np.array([-1, -0.5]) * 0.0254]

#         for i in range(len(jounce_vals)):
#             with self.subTest(i=i):
#                 sus.heave(heave=None)
#                 sus.heave(heave=jounce_vals[i])
#                 known_results = [np.array(x) * 0.0254 for x in [[[0, -16.732931, 2.746991 + 1], [0, 16.732931, 2.746991 + 1], [-61, -16.991025, 3.708966 + 1], [-61, 16.991025, 3.708966 + 1]],
#                                                                 [[0, -16.156144, 1.889552 + 0.5], [0, 16.156144, 1.889552 + 0.5], [-61, -16.378710, 2.558873 + 0.5], [-61, 16.378710, 2.558873 + 0.5]]]]
#                 test_results = [sus.FL_FVIC,
#                                 sus.FR_FVIC,
#                                 sus.RL_FVIC,
#                                 sus.RR_FVIC]

#                 corresponding_values = ["FL_FVIC",
#                                         "FR_FVIC",
#                                         "RL_FVIC",
#                                         "RR_FVIC"]
        
#                 tolerances = [0.001,
#                               0.001,
#                               0.001,
#                               0.001]

#                 for j in range(len(test_results)):
#                     with self.subTest(i=j):
#                         # 2D projection doesn't account for CP_x migration, so don't check x-values (the math here is known to be more accurate)
#                         self.assertLess(np.linalg.norm(np.array(test_results[j][1:]) - np.array(known_results[i][j][1:])), 
#                                         tolerances[j], msg=f"{corresponding_values[j]} |\nTest:\n{test_results[j][1:]}\nKnown:\n{known_results[i][j][1:]}")
    
#     # Not having anti geometry is so common that i need to find some way to standardize this
#     # def test_static_SVIC(self):

#     def test_roll_steer_positive(self):
#         warnings.warn("Roll steer tests aren't valid since the unit test vehicle has no bump steer")
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         roll_vals = [float(x) for x in np.array([0.5, 1]) * 0.0254]

#         known_results = [[0, 0, 0, 0],
#                          [0, 0, 0, 0]]
        
#         for i in range(len(roll_vals)):
#             with self.subTest(i=i):
#                 sus.heave(heave=None)
#                 sus.roll(roll=roll_vals[i])
#                 test_results = [sus.FL_delta,
#                                 sus.FR_delta,
#                                 sus.RL_delta,
#                                 sus.RR_delta]

#                 corresponding_values = ["FL_delta",
#                                         "FR_delta",
#                                         "RL_delta",
#                                         "RR_delta"]
        
#                 tolerances = [0.01,
#                               0.01,
#                               0.01,
#                               0.01]

#                 for j in range(len(test_results)):
#                     with self.subTest(i=j):
#                         self.assertLess(abs(test_results[j] - known_results[i][j]), tolerances[j], msg=corresponding_values[j])
    
#     def test_roll_steer_negative(self):
#         warnings.warn("Roll steer tests aren't valid since the unit test vehicle has no bump steer")
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         roll_vals = [float(x) for x in np.array([-1, -0.5]) * 0.0254]

#         known_results = [[0, 0, 0, 0],
#                          [0, 0, 0, 0]]
        
#         for i in range(len(roll_vals)):
#             with self.subTest(i=i):
#                 sus.heave(heave=None)
#                 sus.roll(roll=roll_vals[i])
#                 test_results = [sus.FL_delta,
#                                 sus.FR_delta,
#                                 sus.RL_delta,
#                                 sus.RR_delta]

#                 corresponding_values = ["FL_delta",
#                                         "FR_delta",
#                                         "RL_delta",
#                                         "RR_delta"]
        
#                 tolerances = [0.01,
#                               0.01,
#                               0.01,
#                               0.01]

#                 for j in range(len(test_results)):
#                     with self.subTest(i=j):
#                         self.assertLess(abs(test_results[j] - known_results[i][j]), tolerances[j], msg=corresponding_values[j])
    
#     def test_roll_gamma_positive(self):
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         roll_vals = [0.5, 1]

#         known_results = [[1.1981, -0.8055, 0.1993, 0.1953],
#                          [1.3985, -0.6134, 0.4012, 0.3877]]
        
#         for i in range(len(roll_vals)):
#             with self.subTest(i=i):
#                 sus.heave(heave=None)
#                 sus.roll(roll=roll_vals[i], n_steps=1, update_state=True)
#                 test_results = [sus.FL_gamma,
#                                 sus.FR_gamma,
#                                 sus.RL_gamma,
#                                 sus.RR_gamma]

#                 corresponding_values = ["FL_gamma",
#                                         "FR_gamma",
#                                         "RL_gamma",
#                                         "RR_gamma"]
        
#                 tolerances = [0.01,
#                               0.01,
#                               0.01,
#                               0.01]

#                 for j in range(len(test_results)):
#                     with self.subTest(i=j):
#                         self.assertLess(abs(test_results[j] - known_results[i][j]), 
#                                         tolerances[j], msg=f"{corresponding_values[j]} |\nTest:\n{test_results[j]}\nKnown:\n{known_results[i][j]}")

#     def test_roll_gamma_negative(self):
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         # Note that -0.45 is different from earlier tests (fix DYN-REF, Abishek)
#         roll_vals = [-1, -0.45]

#         known_results = [[0.6110, -1.3955, -0.3905, -0.3985],
#                          [0.8237, -1.1769, -0.1771, -0.1780]]
        
#         for i in range(len(roll_vals)):
#             with self.subTest(i=i):
#                 sus.heave(heave=None)
#                 sus.roll(roll=roll_vals[i], n_steps=1, update_state=True)
#                 test_results = [sus.FL_gamma,
#                                 sus.FR_gamma,
#                                 sus.RL_gamma,
#                                 sus.RR_gamma]

#                 corresponding_values = ["FL_gamma",
#                                         "FR_gamma",
#                                         "RL_gamma",
#                                         "RR_gamma"]
        
#                 tolerances = [0.01,
#                               0.01,
#                               0.01,
#                               0.01]

#                 for j in range(len(test_results)):
#                     with self.subTest(i=j):
#                         self.assertLess(abs(test_results[j] - known_results[i][j]), 
#                                         tolerances[j], msg=f"{corresponding_values[j]} |\nTest:\n{test_results[j]}\nKnown:\n{known_results[i][j]}")
    
#     def test_roll_caster_positive(self):
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         roll_vals = [0.5, 1]

#         known_results = [[2.241119, 2.245997, 6.399707, 6.409213],
#                          [2.238813, 2.248574, 6.395295, 6.414326]]
        
#         for i in range(len(roll_vals)):
#             with self.subTest(i=i):
#                 sus.heave(heave=None)
#                 sus.roll(roll=roll_vals[i], n_steps=1, update_state=True)
#                 test_results = [sus.FL_caster,
#                                 sus.FR_caster,
#                                 sus.RL_caster,
#                                 sus.RR_caster]

#                 corresponding_values = ["FL_caster",
#                                         "FR_caster",
#                                         "RL_caster",
#                                         "RR_caster"]
        
#                 tolerances = [0.01,
#                               0.01,
#                               0.01,
#                               0.01]

#                 for j in range(len(test_results)):
#                     with self.subTest(i=j):
#                         self.assertLess(abs(test_results[j] - known_results[i][j]), 
#                                         tolerances[j], msg=f"{corresponding_values[j]} |\nTest:\n{test_results[j]}\nKnown:\n{known_results[i][j]}")
    
#     def test_roll_caster_negative(self):
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         roll_vals = [-1, -0.45]

#         known_results = [[2.248554, 2.238790, 6.414279, 6.395256],
#                          [2.241354, 2.245744, 6.400160, 6.408715]]
        
#         for i in range(len(roll_vals)):
#             with self.subTest(i=i):
#                 sus.heave(heave=None)
#                 sus.roll(roll=roll_vals[i], n_steps=1, update_state=True)
#                 test_results = [sus.FL_caster,
#                                 sus.FR_caster,
#                                 sus.RL_caster,
#                                 sus.RR_caster]

#                 corresponding_values = ["FL_caster",
#                                         "FR_caster",
#                                         "RL_caster",
#                                         "RR_caster"]
        
#                 tolerances = [0.01,
#                               0.01,
#                               0.01,
#                               0.01]

#                 for j in range(len(test_results)):
#                     with self.subTest(i=j):
#                         self.assertLess(abs(test_results[j] - known_results[i][j]), 
#                                         tolerances[j], msg=f"{corresponding_values[j]} |\nTest:\n{test_results[j]}\nKnown:\n{known_results[i][j]}")
    
#     def test_roll_kpi_positive(self):
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         roll_vals = [0.5, 1]

#         known_results = [[11.798298, 11.405713, 8.260274, 7.865679],
#                          [11.998710, 11.213546, 8.462110, 7.673222]]
        
#         for i in range(len(roll_vals)):
#             with self.subTest(i=i):
#                 sus.heave(heave=None)
#                 sus.roll(roll=roll_vals[i], n_steps=1, update_state=True)
#                 test_results = [sus.FL_kpi,
#                                 sus.FR_kpi,
#                                 sus.RL_kpi,
#                                 sus.RR_kpi]

#                 corresponding_values = ["FL_kpi",
#                                         "FR_kpi",
#                                         "RL_kpi",
#                                         "RR_kpi"]
        
#                 tolerances = [0.01,
#                               0.01,
#                               0.01,
#                               0.01]

#                 for j in range(len(test_results)):
#                     with self.subTest(i=j):
#                         self.assertLess(abs(test_results[j] - known_results[i][j]), 
#                                         tolerances[j], msg=f"{corresponding_values[j]} |\nTest:\n{test_results[j]}\nKnown:\n{known_results[i][j]}")
    
#     def test_roll_kpi_negative(self):
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         roll_vals = [-1, -0.45]

#         known_results = [[11.211129, 11.995641, 7.670430, 8.459412],
#                          [11.423891, 11.777082, 7.883838, 8.239001]]
        
#         for i in range(len(roll_vals)):
#             with self.subTest(i=i):
#                 sus.heave(heave=None)
#                 sus.roll(roll=roll_vals[i], n_steps=1, update_state=True)
#                 test_results = [sus.FL_kpi,
#                                 sus.FR_kpi,
#                                 sus.RL_kpi,
#                                 sus.RR_kpi]

#                 corresponding_values = ["FL_kpi",
#                                         "FR_kpi",
#                                         "RL_kpi",
#                                         "RR_kpi"]
        
#                 tolerances = [0.01,
#                               0.01,
#                               0.01,
#                               0.01]

#                 for j in range(len(test_results)):
#                     with self.subTest(i=j):
#                         self.assertLess(abs(test_results[j] - known_results[i][j]), 
#                                         tolerances[j], msg=f"{corresponding_values[j]} |\nTest:\n{test_results[j]}\nKnown:\n{known_results[i][j]}")
    
#     def test_roll_scrub_positive(self):
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         roll_vals = [0.5, 1]

#         known_results = [[x * 0.0254 for x in [1.079076, 1.076285, 0.534708, 0.535512]],
#                          [x * 0.0254 for x in [1.077599, 1.073194, 0.534256, 0.535867]]]
        
#         for i in range(len(roll_vals)):
#             with self.subTest(i=i):
#                 sus.heave(heave=None)
#                 sus.roll(roll=roll_vals[i], n_steps=1, update_state=True)
#                 test_results = [sus.FL_scrub,
#                                 sus.FR_scrub,
#                                 sus.RL_scrub,
#                                 sus.RR_scrub]

#                 corresponding_values = ["FL_scrub",
#                                         "FR_scrub",
#                                         "RL_scrub",
#                                         "RR_scrub"]
        
#                 tolerances = [0.001,
#                               0.001,
#                               0.001,
#                               0.001]

#                 for j in range(len(test_results)):
#                     with self.subTest(i=j):
#                         self.assertLess(abs(test_results[j] - known_results[i][j]), 
#                                         tolerances[j], msg=f"{corresponding_values[j]} |\nTest:\n{test_results[j]}\nKnown:\n{known_results[i][j]}")

#     def test_roll_scrub_negative(self):
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         roll_vals = [-1, -0.45]

#         known_results = [[x * 0.0254 for x in [1.082662, 1.091740, 0.536099, 0.534488]],
#                          [x * 0.0254 for x in [1.081488, 1.084998, 0.535582, 0.534857]]]
        
#         for i in range(len(roll_vals)):
#             with self.subTest(i=i):
#                 sus.heave(heave=None)
#                 sus.roll(roll=roll_vals[i], n_steps=1, update_state=True)
#                 test_results = [sus.FL_scrub,
#                                 sus.FR_scrub,
#                                 sus.RL_scrub,
#                                 sus.RR_scrub]

#                 corresponding_values = ["FL_scrub",
#                                         "FR_scrub",
#                                         "RL_scrub",
#                                         "RR_scrub"]
        
#                 tolerances = [0.001,
#                               0.001,
#                               0.001,
#                               0.001]

#                 for j in range(len(test_results)):
#                     with self.subTest(i=j):
#                         self.assertLess(abs(test_results[j] - known_results[i][j]), 
#                                         tolerances[j], msg=f"{corresponding_values[j]} |\nTest:\n{test_results[j]}\nKnown:\n{known_results[i][j]}")
    
#     def test_roll_mech_trail_positive(self):
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         roll_vals = [0.5, 1]

#         known_results = [[x * 0.0254 for x in [0.193039, 0.192589, 0.280100, 0.279443]],
#                          [x * 0.0254 for x in [0.193281, 0.192381, 0.280574, 0.279257]]]
        
#         for i in range(len(roll_vals)):
#             with self.subTest(i=i):
#                 sus.heave(heave=None)
#                 sus.roll(roll=roll_vals[i], n_steps=1, update_state=True)
#                 test_results = [sus.FL_mech_trail,
#                                 sus.FR_mech_trail,
#                                 sus.RL_mech_trail,
#                                 sus.RR_mech_trail]

#                 corresponding_values = ["FL_mech_trail",
#                                         "FR_mech_trail",
#                                         "RL_mech_trail",
#                                         "RR_mech_trail"]
        
#                 tolerances = [0.001,
#                               0.001,
#                               0.001,
#                               0.001]

#                 for j in range(len(test_results)):
#                     with self.subTest(i=j):
#                         self.assertLess(abs(test_results[j] - known_results[i][j]), 
#                                         tolerances[j], msg=f"{corresponding_values[j]} |\nTest:\n{test_results[j]}\nKnown:\n{known_results[i][j]}")

#     def test_roll_mech_trail_negative(self):
#         sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
#         sus = Suspension(sus_data=sus_data)

#         roll_vals = [-1, -0.45]

#         known_results = [[x * 0.0254 for x in [0.192312, 0.193209, 0.279052, 0.280356]],
#                          [x * 0.0254 for x in [0.192578, 0.192982, 0.279372, 0.279960]]]
        
#         for i in range(len(roll_vals)):
#             with self.subTest(i=i):
#                 sus.heave(heave=None)
#                 sus.roll(roll=roll_vals[i], n_steps=1, update_state=True)
#                 test_results = [sus.FL_mech_trail,
#                                 sus.FR_mech_trail,
#                                 sus.RL_mech_trail,
#                                 sus.RR_mech_trail]

#                 corresponding_values = ["FL_mech_trail",
#                                         "FR_mech_trail",
#                                         "RL_mech_trail",
#                                         "RR_mech_trail"]
        
#                 tolerances = [0.001,
#                               0.001,
#                               0.001,
#                               0.001]

#                 for j in range(len(test_results)):
#                     with self.subTest(i=j):
#                         self.assertLess(abs(test_results[j] - known_results[i][j]), 
#                                         tolerances[j], msg=f"{corresponding_values[j]} |\nTest:\n{test_results[j]}\nKnown:\n{known_results[i][j]}")
    
#     # Need to do transformations on DYN-REF outputs, and I trust the math here more
#     # def test_roll_FVIC_positive(self):