import numpy as np
import pandas as pd

class Simulation:
    def __init__(self, car, track, is_skidpad = False):
        self.track = track
        self.car = car
        self.is_skidpad = is_skidpad

        self.time = 0
        self.results = None
        self.reverse_sim_results = None
        self.forward_sim_results = None

    def run(self):
        if self.is_skidpad:
            return None, self.__skidpad_sim()

        # get turn radii from raceline points
        track_points = curvature(self.track.path_points())

        # find dist traveled for each point on track
        track_points["dist"], track_points["pos"], length_traveled = 0, 0, 0
        previous_row = track_points.iloc[1]
        for i, row in track_points.iloc[1:].iterrows():
            dist = ((row["x"] - previous_row["x"])**2 + (row["y"] - previous_row["y"])**2)**0.5
            track_points.loc[i,"dist"] = dist
            previous_row, length_traveled = row, length_traveled + dist
            track_points.loc[i,"pos"] = length_traveled

        # TODO: temp fix - make it work without this
        track_points = track_points.drop(track_points.shape[0]-1)
        track_points = track_points.drop(0)
        track_points = track_points.drop(1)

        for x in ["delta_t", "delta_vel", "vel", "ay", "ax", "power_into_inverter", "motor_torque", "motor_efficiency"]:
            track_points[x] = 0

        initial_forward_sim_df = self.__forward_sim(track_points.copy(deep = True), 20, False)
        if self.track.track_type == "endurance":
            starting_velocity = initial_forward_sim_df["vel"].iloc[initial_forward_sim_df.shape[1]]
            ending_velocity = starting_velocity
        else:
            starting_velocity = 0
            ending_velocity = initial_forward_sim_df["vel"].iloc[initial_forward_sim_df.shape[1]]
        print(initial_forward_sim_df["vel"])
        print(ending_velocity)
        self.reverse_sim_results = self.__forward_sim(track_points.copy(deep = True), ending_velocity, True)
        self.forward_sim_results = self.__forward_sim(track_points.copy(deep = True), starting_velocity, False)
    
        ### COMBINE RESULTS
        time, rows = 0, []
        for index, row in track_points.iterrows():
            reverse_row = self.reverse_sim_results.loc[index]
            forward_row = self.forward_sim_results.loc[index]
            if (forward_row["vel"] - reverse_row["vel"]) < 0:
                time += forward_row.loc["delta_t"]
                rows.append(np.array([*forward_row, time]))
            else:
                time += reverse_row.loc["delta_t"]
                rows.append(np.array([*reverse_row, time]))
        
        self.results = pd.DataFrame(rows, columns=[*self.reverse_sim_results.columns, "time"])
        self.time = self.results["time"].max()
        return self.results, self.time
    
    def __skidpad_sim(self):
        path_radius = 7.75 + self.car.params.front_track / 2 + .1524
        speed = self.car.max_vel_corner(path_radius) # speed to possible to take
        time = path_radius * 2 * np.pi / speed
        return time

    def __forward_sim(self, df, starting_vel, is_reverse):
        accel_func = self.car.deccel if is_reverse else self.car.accel
        vel = starting_vel

        for i, row in (df[::-1] if is_reverse else df).iterrows():
            vmax = self.car.max_vel_corner(row["R"])
            df.loc[i ,"max_vel"] = vmax
            AY = self.car.lateral(self.car.params.max_vel) # accel capabilities
            df.loc[i,"ay"] = min(vel**2/row["R"], AY) if row["R"] != 0 else 0 # actual accel
            if vel < vmax:
                df.loc[i,"ax"], df.loc[i,"power_into_inverter"], df.loc[i, "motor_torque"], df.loc[i ,"motor_efficiency"] = accel_func(vel, df.loc[i,"ay"])
                roots = np.roots([0.5*df.loc[i,"ax"], vel, -row["dist"]])
                df.loc[i,"delta_t"] = min(roots) if min(roots) > 0 else max(roots)
                df.loc[i,"delta_vel"] = min(df.loc[i,"ax"]*df.loc[i,"delta_t"], vmax-vel)
                df.loc[i,"vel"] = vel + df.loc[i,"delta_vel"]
                vel = df.loc[i,"vel"]
            else:
                # car maxed out, no more accel
                vel = vmax
                df.loc[i,"vel"] = vel
                df.loc[i,"delta_t"] = row["dist"]/vel
                df.loc[i,"ax"], df.loc[i,"delta_vel"] = 0, 0
        if is_reverse:
            df["ax"]*=-1
        return df

def curvature(track_df):
    # radius of curvature and curvature vector for 2D or 3D curve
    # X - x,y column array
    # [L, R, k] = Cumulative arc length, radius of curvature, curvature vector
    track_df["L"], track_df["R"], track_df["kx"], track_df["ky"], track_df["kz"] = 0, 0, 0, 0, 0
    previous_row = track_df.iloc[-1]
    previous_coords_row = track_df.iloc[-1][["x", "y", "z"]]
    for i, row in track_df.iloc[:-1].iterrows():
        coords_row = row[["x", "y", "z"]]
        track_df.loc[i,"R"], _, k = circumcenter(coords_row, previous_coords_row, track_df.iloc[i+1, :][["x", "y", "z"]])
        track_df.loc[i,"kx"], track_df.loc[i,"ky"], track_df.loc[i,"kz"] = k
        track_df.loc[i,"L"] = previous_row["L"] + np.linalg.norm(coords_row - previous_coords_row)
        previous_row, previous_coords_row = row, coords_row
    
    N = track_df.shape[0]
    track_df.loc[N-1,"R"], _, k = circumcenter(previous_coords_row, track_df[["x", "y", "z"]].iloc[N-1, :], track_df.iloc[0, :][["x", "y", "z"]])
    track_df.loc[N-1,"kx"], track_df.loc[N-1,"ky"], track_df.loc[N-1,"kz"] = k
    track_df.loc[N-1,"L"] = track_df["L"].iloc[N-2] + np.linalg.norm(track_df[["x", "y", "z"]].iloc[N-1, :] - previous_coords_row)
    return track_df

def circumcenter(A,B,C):
    # center and radius of the circumscribed circle for the triangle ABC
    # R - Radius
    # M - 3D coordinate vector for center
    # k - vector of length 1/r in the direction from A to M
    D = np.cross(B-A, C-A)
    b = np.linalg.norm(A-C)
    c = np.linalg.norm(A-B)
    E = np.cross(D, B-A)
    F = np.cross(D, C-A)
    G = (b**2*E-c**2*F)/np.linalg.norm(D)**2/2 if np.linalg.norm(D) != 0 else np.array([0,0,0])
    M = A + G
    R = np.linalg.norm(G)
    k = G if R == 0 else G/R**2
    return R, M, k