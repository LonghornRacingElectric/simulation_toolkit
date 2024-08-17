from suspension_model.suspension_elements.tertiary_elements.tire import Tire
from suspension_model.suspension_elements.primary_elements.node import Node
from typing import Sequence
from typing import Callable
from typing import Tuple
import pyvista as pv
import numpy as np

class Plotter:
    def __init__(self) -> None:
        self.pl = pv.Plotter()

    def add_ground(self, FL_cp: Node, RL_cp: Node, tire: Tire):
        FL_pos = FL_cp.position
        RL_pos = RL_cp.position

        absolute_center = [((FL_pos + RL_pos) / 2)[0], 0, 0]
        length = abs(FL_pos[0] - RL_pos[0]) + 2 * tire.radius
        width = 2 * FL_pos[1] + tire.width

        self.pl.add_mesh(pv.Plane(center=absolute_center, direction=[0, 0, 1], i_size=length, j_size=width), color="lightblue", opacity=0.75)

    def add_tire(self, center: Sequence[float], direction: Sequence[float], radius: float, height: float):
        self.pl.add_mesh(pv.CylinderStructured(radius=[5, radius], height=height, center=center, direction=direction), color="#504050", opacity=0.5)
    
    def add_link(self, center: Sequence[float], direction: Sequence[float], radius: float, height: float, color: str = "gray") -> None:
        self.pl.add_mesh(pv.Cylinder(center=center, direction=direction, radius=radius, height=height), color=color)
    
    def add_node(self, center: Sequence[float], radius: float = 0.875 / 2, color: str = "red") -> pv.Actor:
        node = self.pl.add_mesh(pv.Sphere(radius=radius, center=center), color=color)
        return node

    def add_slider(self, func: Callable, title: str, bounds: Tuple[float, float], pos: Tuple[Tuple[float, float], Tuple[float, float]]):
        self.pl.add_slider_widget(
        callback=lambda value: func(value),
        rng=[bounds[0], bounds[1]],
        value=0,
        title=title,
        pointa=(pos[0][0], pos[0][1]),
        pointb=(pos[1][0], pos[1][1]),
        style='modern',
        interaction_event='always'
    )

    def start_gif(self):
        self.pl.open_gif("jounce_sweep.gif", fps=144)
    
    def write_frame(self):
        self.pl.write_frame()
        self.pl.clear_actors()
    
    def clear(self):
        self.pl.clear_actors()

    def end_gif(self):
        self.pl.close()

    def show_grid(self):
        self.pl.show_grid()

    def show(self):
        self.pl.show_axes()
        self.pl.show()