from typing import Sequence
import pyvista as pv
import numpy as np

class Plotter:
    def __init__(self) -> None:
        self.pl = pv.Plotter()

    def add_tire(self, center: Sequence[float], direction: Sequence[float], radius: float, height: float):
        self.pl.add_mesh(pv.CylinderStructured(radius=[5, radius], height=height, center=center, direction=direction), color="#504050", opacity=0.5)
    
    def add_link(self, center: Sequence[float], direction: Sequence[float], radius: float, height: float) -> None:
        self.pl.add_mesh(pv.Cylinder(center=center, direction=direction, radius=radius, height=height), color="gray")

    def add_cp(self, center: Sequence[float]):
        self.pl.add_mesh(pv.Sphere(radius=0.125, center=center), color="red")

    def start_gif(self):
        self.pl.open_gif("jounce_sweep.gif", fps=144)
    
    def write_frame(self):
        self.pl.write_frame()
        self.pl.clear_actors()

    def end_gif(self):
        self.pl.close()

    def show(self):
        self.pl.show_axes()
        self.pl.show()