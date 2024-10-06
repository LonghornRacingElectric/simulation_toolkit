from TrackGen import Track as Track
from TrackData import Endurance as Endurance


if __name__ == "__main__":
    track_points_inner, track_points_outer = Endurance()
    new_Track = Track(track_points_inner, track_points_outer, trackwidth=0)
    new_Track.run_optimize()
    new_Track.plot()


   

    




