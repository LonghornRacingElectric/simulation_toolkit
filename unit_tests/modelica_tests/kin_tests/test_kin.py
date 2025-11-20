from src.vehicle_model.suspension_model.suspension_data import SuspensionData
from src.vehicle_model.suspension_model.suspension import Suspension

from src._3_custom_libraries.fmu_simulator import FMUSimulator
from src._2_misc_studies.kin_fmu.Kinematics import Kinematics_FMU


import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import time as systime
import warnings

from unittest import TestCase
fmu_sim = FMUSimulator(fmu_path="./src/_2_misc_studies/kin_fmu/Kinematics2.fmu")
time = np.linspace(0, 2, 8000)




class TestKin(TestCase):
    
    # 3. Run the fmu (?make the kin fmu a class and import here, run)
    # 4. set up the value comparison
    # 5. html unit tests results
    @classmethod
    def setUpClass(cls):
        #loads the python model
        cls.sus_data = SuspensionData(path="./unit_tests/python_tests/_test_dependencies/unit_test_vehicle.yml")
        cls.sus = Suspension(sus_data=cls.sus_data)
        
        #loads the modelica FMU
        cls.kin = Kinematics_FMU(fmu_path="./src/_2_misc_studies/kin_fmu/Kinematics2.fmu")
        cls.kin.run_simulation_jounce()
        cls.kin.run_simulation_roll()
        
    
    #Jounce tests
    
    def test_bump_gamma(self):
        
        
        jounce_vals = [float(x) for x in np.array([-0.05079992, -0.02528294, 0.00018288, 0.02517927, 0.05079957])]
        tolerance = 0.1
        #both sims already ran
        gamma_fmu = self.kin.get_output_jounce('camber', degrees = True)
        jounce_interp = np.interp(self.kin.time, self.kin.input_data['time'], self.kin.input_data['jounce'])
        mask = (self.kin.time >= 0.5) & (self.kin.time <= 1.5)
        gamma_fmu = gamma_fmu[mask]
        jounce_interp = jounce_interp[mask]
        

        #reading the gamma values from the python suspension model
        for jounce in jounce_vals:
            with self.subTest(jounce=jounce):
                self.sus.heave(heave=None) # resets heave
                self.sus.heave(heave=jounce) #applies the current jounce value to produce heave
                gamma_py = self.sus.FL_gamma

                gamma_fmu_value = np.interp(jounce, jounce_interp, gamma_fmu)
                diff = abs(gamma_py - gamma_fmu_value)
                self.assertLess(diff, tolerance)


                
    def test_bump_toe(self):

        jounce_vals = [float(x) for x in np.array([-0.05079992, -0.02528294, 0.00018288, 0.02517927, 0.05079957])]
        tolerance = 0.1
        #both sims already ran
        toe_fmu = -1*self.kin.get_output_jounce('toe', degrees = True)
        jounce_interp = np.interp(self.kin.time, self.kin.input_data['time'], self.kin.input_data['jounce'])
        mask = (self.kin.time >= 0.5) & (self.kin.time <= 1.5)
        toe_fmu = -1*toe_fmu[mask]
        jounce_interp = jounce_interp[mask]
        

        #reading the gamma values from the python suspension model
        for jounce in jounce_vals:
            with self.subTest(jounce=jounce):
                self.sus.heave(heave=None) # resets heave
                self.sus.heave(heave=jounce) #applies the current jounce value to produce heave
                toe_py = self.sus.FL_delta

                toe_fmu_value = np.interp(jounce, jounce_interp, toe_fmu)
                diff = abs(toe_py - toe_fmu_value)
                self.assertLess(diff, tolerance)
                
    def test_bump_caster(self):
        
        
        jounce_vals = [float(x) for x in np.array([-0.05079992, -0.02528294, 0.00018288, 0.02517927, 0.05079957])]
        tolerance = 0.1

        caster_fmu = self.kin.get_output_jounce('caster', degrees = True)
        jounce_interp = np.interp(self.kin.time, self.kin.input_data['time'], self.kin.input_data['jounce'])
        mask = (self.kin.time >= 0.5) & (self.kin.time <= 1.5)
        caster_fmu = caster_fmu[mask]
        jounce_interp = jounce_interp[mask]
        
        for jounce in jounce_vals:
            with self.subTest(jounce=jounce):
                self.sus.heave(heave=None) # resets heave
                self.sus.heave(heave=jounce) #applies the current jounce value to produce heave
                caster_py = self.sus.FL_caster
                
                #now doing the same for the FMU
                caster_fmu_value = np.interp(jounce, jounce_interp, caster_fmu)
                diff = abs(caster_py - caster_fmu_value)
                self.assertLess(diff, tolerance)

    def test_bump_kpi(self):
        jounce_vals = [float(x) for x in np.array([-0.05079992, -0.02528294, 0.00018288, 0.02517927, 0.05079957])]
        tolerance = 0.1

        kpi_fmu = self.kin.get_output_jounce('kpi', degrees = True)
        jounce_interp = np.interp(self.kin.time, self.kin.input_data['time'], self.kin.input_data['jounce'])
        mask = (self.kin.time >= 0.5) & (self.kin.time <= 1.5)
        kpi_fmu = kpi_fmu[mask]
        jounce_interp = jounce_interp[mask]
        
        for jounce in jounce_vals:
            with self.subTest(jounce=jounce):
                self.sus.heave(heave=None) # resets heave
                self.sus.heave(heave=jounce) #applies the current jounce value to produce heave
                kpi_py = self.sus.FL_kpi
                
                #now doing the same for the FMU
                kpi_fmu_value = np.interp(jounce, jounce_interp, kpi_fmu)
                diff = abs(kpi_py - kpi_fmu_value)
                self.assertLess(diff, tolerance)

    def test_bump_mech_trail(self):
        jounce_vals = [float(x) for x in np.array([-0.05079992, -0.02528294, 0.00018288, 0.02517927, 0.05079957])]
        tolerance = 0.1

        mech_trail_fmu = self.kin.get_output_jounce('trail') * 1000
        jounce_interp = np.interp(self.kin.time, self.kin.input_data['time'], self.kin.input_data['jounce'])
        mask = (self.kin.time >= 0.5) & (self.kin.time <= 1.5)
        mech_trail_fmu = mech_trail_fmu[mask]
        jounce_interp = jounce_interp[mask]
        
        for jounce in jounce_vals:
            with self.subTest(jounce=jounce):
                self.sus.heave(heave=None) # resets heave
                self.sus.heave(heave=jounce) #applies the current jounce value to produce heave
                mech_trail_py = self.sus.FL_mech_trail * 1000
                
                #now doing the same for the FMU
                mech_trail_fmu_value = np.interp(jounce, jounce_interp, mech_trail_fmu)
                diff = abs(mech_trail_py - mech_trail_fmu_value)
                self.assertLess(diff, tolerance)

    def test_bump_scrub(self):
        jounce_vals = [float(x) for x in np.array([-0.05079992, -0.02528294, 0.00018288, 0.02517927, 0.05079957])]
        tolerance = 0.1

        scrub_fmu = self.kin.get_output_jounce('scrub') * 1000
        jounce_interp = np.interp(self.kin.time, self.kin.input_data['time'], self.kin.input_data['jounce'])
        mask = (self.kin.time >= 0.5) & (self.kin.time <= 1.5)
        scrub_fmu = scrub_fmu[mask]
        jounce_interp = jounce_interp[mask]
        
        for jounce in jounce_vals:
            with self.subTest(jounce=jounce):
                self.sus.heave(heave=None) # resets heave
                self.sus.heave(heave=jounce) #applies the current jounce value to produce heave
                scrub_py = self.sus.FL_scrub * 1000
                
                #now doing the same for the FMU
                scrub_fmu_value = np.interp(jounce, jounce_interp, scrub_fmu)
                diff = abs(scrub_py - scrub_fmu_value)
                self.assertLess(diff, tolerance)

    def test_bump_FVIC_y(self):
        jounce_vals = [float(x) for x in np.array([-0.05079992, -0.02528294, 0.00018288, 0.02517927, 0.05079957])]
        tolerance = 0.1

        FVIC_Y_migration_fmu = self.kin.get_output_jounce('FVIC_y_migration') * 1000
        jounce_interp = np.interp(self.kin.time, self.kin.input_data['time'], self.kin.input_data['jounce'])
        mask = (self.kin.time >= 0.5) & (self.kin.time <= 1.5)
        FVIC_Y_migration_fmu = FVIC_Y_migration_fmu[mask]
        jounce_interp = jounce_interp[mask]
        
        for jounce in jounce_vals:
            with self.subTest(jounce=jounce):
                self.sus.heave(heave=None)
                self.sus.heave(heave=jounce)
                FVIC_Y_migration_py = self.sus.FL_FVIC[1] * 1000
                

                FVIC_Y_migration_fmu_value = np.interp(jounce, jounce_interp, FVIC_Y_migration_fmu)
                diff = abs(FVIC_Y_migration_py - FVIC_Y_migration_fmu_value)
                self.assertLess(diff, tolerance)


    def test_bump_FVIC_z(self):
        jounce_vals = [float(x) for x in np.array([-0.05079992, -0.02528294, 0.00018288, 0.02517927, 0.05079957])]
        tolerance = 0.1

        FVIC_Z_migration_fmu = self.kin.get_output_jounce('FVIC_z_migration') * 1000
        jounce_interp = np.interp(self.kin.time, self.kin.input_data['time'], self.kin.input_data['jounce'])
        mask = (self.kin.time >= 0.5) & (self.kin.time <= 1.5)
        FVIC_Z_migration_fmu = FVIC_Z_migration_fmu[mask]
        jounce_interp = jounce_interp[mask]

        for jounce in jounce_vals:
            with self.subTest(jounce=jounce):
                self.sus.heave(heave=None)
                self.sus.heave(heave=jounce)
                fvic_z_py = self.sus.FL_FVIC[2] * 1000.0

                FVIC_Z_migration_fmu_value = np.interp(jounce, jounce_interp, FVIC_Z_migration_fmu)
                diff = abs(fvic_z_py - FVIC_Z_migration_fmu_value)
                self.assertLess(diff, tolerance)

    def test_bump_roll_center_y(self):
        jounce_vals = [float(x) for x in np.array([-0.05079992, -0.02528294, 0.00018288, 0.02517927, 0.05079957])]
        tolerance = 0.1

        roll_center_y_fmu = self.kin.get_output_jounce('roll_center_y') * 1000
        jounce_interp = np.interp(self.kin.time, self.kin.input_data['time'], self.kin.input_data['jounce'])
        mask = (self.kin.time >= 0.5) & (self.kin.time <= 1.5)
        roll_center_y_fmu = roll_center_y_fmu[mask]
        jounce_interp = jounce_interp[mask]

        for jounce in jounce_vals:
            with self.subTest(jounce=jounce):
                self.sus.heave(heave=None)
                self.sus.heave(heave=jounce)
                rc_y_py = self.sus.Fr_RC[1] * 1000.0

                roll_center_y_fmu_value = np.interp(jounce, jounce_interp, roll_center_y_fmu)
                diff = abs(rc_y_py - roll_center_y_fmu_value)
                self.assertLess(diff, tolerance)
    

    def test_bump_roll_center_z(self):
        jounce_vals = [float(x) for x in np.array([-0.05079992, -0.02528294, 0.00018288, 0.02517927, 0.05079957])]
        tolerance = 0.1

        roll_center_z_fmu = self.kin.get_output_jounce('roll_center_z') * 1000
        jounce_interp = np.interp(self.kin.time, self.kin.input_data['time'], self.kin.input_data['jounce'])
        mask = (self.kin.time >= 0.5) & (self.kin.time <= 1.5)
        roll_center_z_fmu = roll_center_z_fmu[mask]
        jounce_interp = jounce_interp[mask]

        for jounce in jounce_vals:
            with self.subTest(jounce=jounce):
                self.sus.heave(heave=None)
                self.sus.heave(heave=jounce)
                rc_z_py = self.sus.Fr_RC[2] * 1000.0

                roll_center_z_fmu_value = np.interp(jounce, jounce_interp, roll_center_z_fmu)
                diff = abs(rc_z_py - roll_center_z_fmu_value)
                self.assertLess(diff, tolerance)


    # Need to add all the roll tests

    # Tests for the roll cases


    def test_roll_gamma(self):
        roll_vals = [float(x) for x in np.array([-1.49228803, -7.58740401e-01, 1.97993360e-02, 7.55698496e-01, 1.49936868])]
        tolerance = 0.1
        static_FL_camber_deg = -2
        gamma_fmu = self.kin.get_output_roll('camber_deg') + static_FL_camber_deg
        roll_interp = self.kin.roll_deg
        mask = (self.kin.rt >= 0.5) & (self.kin.rt <= 1.5)
        gamma_fmu = gamma_fmu[mask]
        roll_interp = roll_interp[mask]

        for roll in roll_vals:
            with self.subTest(roll=roll):
                self.sus.heave(heave = None)
                self.sus.roll(roll = roll, n_steps=1, update_state=True)
                gamma_py = self.sus.FL_gamma
                gamma_fmu_value = np.interp(roll, roll_interp, gamma_fmu)
                diff = abs(gamma_py - gamma_fmu_value)
                self.assertLess(diff, tolerance)




    def test_roll_toe(self):
        roll_vals = [float(x) for x in np.array([-1.49228803, -7.58740401e-01, 1.97993360e-02, 7.55698496e-01, 1.49936868])]
        tolerance = 0.1
        toe_fmu = self.kin.get_output_roll('toe_deg')
        roll_interp = self.kin.roll_deg
        mask = (self.kin.rt >= 0.5) & (self.kin.rt <= 1.5)
        toe_fmu = toe_fmu[mask]
        roll_interp = roll_interp[mask]

        for roll in roll_vals:
            with self.subTest(roll=roll):
                self.sus.heave(heave=None)
                self.sus.roll(roll=roll, n_steps=1, update_state=True)

                toe_py = self.sus.FL_delta

                toe_fmu_value = np.interp(roll, roll_interp, toe_fmu)
                diff = abs(toe_py - toe_fmu_value)
                self.assertLess(diff, tolerance)


    def test_roll_caster(self):
        roll_vals = [float(x) for x in np.array([-1.49228803, -7.58740401e-01, 1.97993360e-02, 7.55698496e-01, 1.49936868])]
        tolerance = 0.1
        caster_fmu = self.kin.get_output_roll('caster_deg')
        roll_interp = self.kin.roll_deg
        mask = (self.kin.rt >= 0.5) & (self.kin.rt <= 1.5)
        caster_fmu = caster_fmu[mask]
        roll_interp = roll_interp[mask]

        for roll in roll_vals:
            with self.subTest(roll=roll):
                self.sus.heave(heave=None)
                self.sus.roll(roll=roll, n_steps=1, update_state=True)

                caster_py = self.sus.FL_caster

                caster_fmu_value = np.interp(roll, roll_interp, caster_fmu)
                diff = abs(caster_py - caster_fmu_value)
                self.assertLess(diff, tolerance)


    def test_roll_kpi(self):
        roll_vals = [float(x) for x in np.array([-1.49228803, -7.58740401e-01, 1.97993360e-02, 7.55698496e-01, 1.49936868])]
        tolerance = 0.1
        roll_interp = self.kin.roll_deg
        kpi_fmu = self.kin.get_output_roll('kpi_deg') + roll_interp
        mask = (self.kin.rt >= 0.5) & (self.kin.rt <= 1.5)
        kpi_fmu = kpi_fmu[mask]
        roll_interp = roll_interp[mask]

        for roll in roll_vals:
            with self.subTest(roll=roll):
                self.sus.heave(heave=None)
                self.sus.roll(roll=roll, n_steps=1, update_state=True)

                kpi_py = self.sus.FL_kpi

                kpi_fmu_value = np.interp(roll, roll_interp, kpi_fmu)
                diff = abs(kpi_py - kpi_fmu_value)
                self.assertLess(diff, tolerance)

    def test_roll_mech_trail(self):
        roll_vals = [float(x) for x in np.array([-1.49228803, -7.58740401e-01, 1.97993360e-02, 7.55698496e-01, 1.49936868])]
        tolerance = 0.1
        trail_fmu = self.kin.get_output_roll('trail_mm')
        roll_interp = self.kin.roll_deg
        mask = (self.kin.rt >= 0.5) & (self.kin.rt <= 1.5)
        trail_fmu = trail_fmu[mask]
        roll_interp = roll_interp[mask]

        for roll in roll_vals:
            with self.subTest(roll=roll):
                self.sus.heave(heave=None)
                self.sus.roll(roll=roll, n_steps=1, update_state=True)

                trail_py = self.sus.FL_mech_trail * 1000.0

                trail_fmu_value = np.interp(roll, roll_interp, trail_fmu)
                diff = abs(trail_py - trail_fmu_value)
                self.assertLess(diff, tolerance)


    def test_roll_scrub(self):
        roll_vals = [float(x) for x in np.array([-1.49228803, -7.58740401e-01, 1.97993360e-02, 7.55698496e-01, 1.49936868])]
        tolerance = 0.1
        scrub_fmu = self.kin.get_output_roll('scrub_mm')
        roll_interp = self.kin.roll_deg
        mask = (self.kin.rt >= 0.5) & (self.kin.rt <= 1.5)
        scrub_fmu = scrub_fmu[mask]
        roll_interp = roll_interp[mask]

        for roll in roll_vals:
            with self.subTest(roll=roll):
                self.sus.heave(heave=None)
                self.sus.roll(roll=roll, n_steps=1, update_state=True)

                scrub_py = self.sus.FL_scrub * 1000.0

                scrub_fmu_value = np.interp(roll, roll_interp, scrub_fmu)
                diff = abs(scrub_py - scrub_fmu_value)
                self.assertLess(diff, tolerance)


    def test_roll_FVIC_y(self):
        roll_vals = [float(x) for x in np.array([-1.49228803, -7.58740401e-01, 1.97993360e-02, 7.55698496e-01, 1.49936868])]
        tolerance = 0.1
        FVIC_y_fmu = self.kin.get_output_roll('FVIC_y_mm')
        roll_interp = self.kin.roll_deg
        mask = (self.kin.rt >= 0.5) & (self.kin.rt <= 1.5)
        FVIC_y_fmu = FVIC_y_fmu[mask]
        roll_interp = roll_interp[mask]

        for roll in roll_vals:
            with self.subTest(roll=roll):
                self.sus.heave(heave=None)
                self.sus.roll(roll=roll, n_steps=1, update_state=True)

                fvic_y_py = self.sus.FL_FVIC[1] * 1000.0

                FVIC_y_fmu_value = np.interp(roll, roll_interp, FVIC_y_fmu)
                diff = abs(fvic_y_py - FVIC_y_fmu_value)
                self.assertLess(diff, tolerance)


    def test_roll_FVIC_z(self):
        roll_vals = [float(x) for x in np.array([-1.49228803, -7.58740401e-01, 1.97993360e-02, 7.55698496e-01, 1.49936868])]
        tolerance = 0.1
        FVIC_z_fmu = self.kin.get_output_roll('FVIC_z_mm')
        roll_interp = self.kin.roll_deg
        mask = (self.kin.rt >= 0.5) & (self.kin.rt <= 1.5)
        FVIC_z_fmu = FVIC_z_fmu[mask]
        roll_interp = roll_interp[mask]

        for roll in roll_vals:
            with self.subTest(roll=roll):
                self.sus.heave(heave=None)
                self.sus.roll(roll=roll, n_steps=1, update_state=True)

                fvic_z_py = self.sus.FL_FVIC[2] * 1000.0

                FVIC_z_fmu_value = np.interp(roll, roll_interp, FVIC_z_fmu)
                diff = abs(fvic_z_py - FVIC_z_fmu_value)
                self.assertLess(diff, tolerance)


    def test_roll_center_y(self):
        roll_vals = [float(x) for x in np.array([-1.49228803, -7.58740401e-01, 1.97993360e-02, 7.55698496e-01, 1.49936868])]
        tolerance = 0.1
        RC_y_fmu = self.kin.get_output_roll('RC_y_mm')
        roll_interp = self.kin.roll_deg
        mask = (self.kin.rt >= 0.5) & (self.kin.rt <= 1.5)
        RC_y_fmu = RC_y_fmu[mask]
        roll_interp = roll_interp[mask]

        for roll in roll_vals:
            with self.subTest(roll=roll):
                self.sus.heave(heave=None)
                self.sus.roll(roll=roll, n_steps=1, update_state=True)

                rc_y_py = self.sus.Fr_RC[1] * 1000.0

                RC_y_fmu_value = np.interp(roll, roll_interp, RC_y_fmu)
                diff = abs(rc_y_py - RC_y_fmu_value)
                self.assertLess(diff, tolerance)


    def test_roll_center_z(self):
        roll_vals = [float(x) for x in np.array([-1.49228803, -7.58740401e-01, 1.97993360e-02, 7.55698496e-01, 1.49936868])]
        tolerance = 0.1
        RC_z_fmu = self.kin.get_output_roll('RC_z_mm')
        roll_interp = self.kin.roll_deg
        mask = (self.kin.rt >= 0.5) & (self.kin.rt <= 1.5)
        RC_z_fmu = RC_z_fmu[mask]
        roll_interp = roll_interp[mask]

        for roll in roll_vals:
            with self.subTest(roll=roll):
                self.sus.heave(heave=None)
                self.sus.roll(roll=roll, n_steps=1, update_state=True)

                rc_z_py = self.sus.Fr_RC[2] * 1000.0

                RC_z_fmu_value = np.interp(roll, roll_interp, RC_z_fmu)
                diff = abs(rc_z_py - RC_z_fmu_value)
                self.assertLess(diff, tolerance)