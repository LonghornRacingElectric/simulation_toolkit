# TODO laps, events, points
# this may either use a steady state sim or a transient sim depending on how fast/effective the transient sim turns out

from sim.simulations.QSS_sim.QSS import Simulation
from sim.simulations.QSS_sim.track import Track

class Competition:
    def __init__(self, racecar, endurance, autocross, acceleration, skidpad_times = None, accel_times = None):
        self.racecar = racecar

        accel_track = Track(acceleration, best_time = accel_times)
        skidpad_track = Track(best_time = skidpad_times)
        
        self.endurance_sim = Simulation(self.racecar, Track(endurance))
        self.autocross_sim = Simulation(self.racecar, Track(autocross))
        self.skidpad_sim = Simulation(self.racecar, skidpad_track, is_skidpad = True)
        self.acceleration_sim = Simulation(self.racecar, accel_track)

    def run(self):
        print("Running Endurance Sim")
        result_endurance, time_e = self.endurance_sim.run()
        print("Running Autocross Sim")
        results_autocross, time_ax = self.autocross_sim.run()
        print("Running Skidpad Sim")
        results_skidpad, time_s = self.skidpad_sim.run()
        print("Running Acceleration Sim")
        results_acceleration, time_a = self.acceleration_sim.run()
        
        times = [time_e, time_ax, time_s, time_a]
        points = self.points(*times)

        return [result_endurance, results_autocross, results_skidpad, results_acceleration], points, times

    def points(self, time_e, time_ax, time_s, time_a):
        a_min = self.acceleration_sim.track.best_time
        a_max = 1.5 * a_min
        if time_a < a_min:
            score_a = 100
        elif time_a <= a_max:
            score_a = 95.5 * ((a_max/time_a - 1)/(a_max/a_min - 1)) + 4.5
        else:
            score_a = 4.5
        
        s_min = self.skidpad_sim.track.best_time
        s_max = 1.25 * s_min
        if time_s < s_min:
            score_s = 75
        elif time_s <= s_max:
            score_s = 71.5 * (((s_max/time_s)**2 - 1)/((s_max/s_min)**2 - 1)) + 3.5
        else:
            score_s = 3.5
        
        ax_min = self.autocross_sim.track.best_time
        ax_max = 1.45 * ax_min
        if time_ax < ax_min:
            score_ax = 125
        elif time_ax <= ax_max:
            score_ax = 118.5 * ((ax_max/time_ax - 1)/((ax_max/ax_min) - 1)) + 6.5
        else:
            score_ax = 6.5

        e_min = self.endurance_sim.track.best_time
        e_max = 1.45 * e_min
        time_e_total = 11 * time_e
        if time_e_total < e_min:
            score_e = 275
        elif time_e_total <= e_max:
            score_e = 250 * ((e_max/time_e_total - 1)/(e_max/e_min - 1)) + 25
        else:
            score_e = 25

        return [score_e, score_ax, score_s, score_a]
    
