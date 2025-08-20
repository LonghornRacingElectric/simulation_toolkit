from src.vehicle_model.suspension_model.suspension_elements.tertiary_elements.tire import Tire
from src.vehicle_model.suspension_model.suspension_elements._1_elements.node import Node

from typing import Sequence, Union
from pyvista import PolyData
from typing import Callable
from typing import Tuple
import pyvista as pv


class Plotter:
    """
    ## Plotter

    Parameters
    ----------
    None

    """
    def __init__(self) -> None:
        self.pl = pv.Plotter()
        self.pl.enable_anti_aliasing()
        self.pl.set_background('white')

        self.tires: Union[Sequence[PolyData], list] = []
        self.links: Union[Sequence[PolyData], list] = []

    def add_ground(self, FL_cp: Node, RL_cp: Node, tire: Tire) -> None:
        """
        ## Add Ground

        Adds ground to pyvista window
        - Ground always points in [0, 0, 1] direction

        Parameters
        ----------
        FL_cp : Node
            Front left contact patch
        RL_cp : Node
            Rear left contact patch
        tire : Tire
            Tire object (expands ground plane up to edges of tire)

        Returns
        ----------
        None
        """
        FL_pos = FL_cp.position
        RL_pos = RL_cp.position

        absolute_center = [((FL_pos + RL_pos) / 2)[0], 0, 0]
        length = abs(FL_pos[0] - RL_pos[0]) + 2 * tire.radius
        width = 2 * FL_pos[1] + tire.width

        self.pl.add_mesh(pv.Plane(center=absolute_center, direction=[0, 0, 1], i_size=length, j_size=width), color="lightblue", opacity=0.75)

    def add_tire(self, center: Sequence[float], direction: Sequence[float], inner_radius: float, outer_radius: float, height: float) -> None:
        """
        ## Add Tire

        Adds tire Actor to pyvista window

        Parameters
        ----------
        center : Sequence[float]
            Center coordinate of tire
        direction : Sequence[float]
            Direction unit vector of tire
        inner_radius : float
            Inner radius of tire
        outer_radius : float
            Outer radius of tire
        height : float
            Width of tire

        Returns
        ----------
        None
        """
        self.pl.add_mesh(pv.CylinderStructured(radius=[inner_radius, outer_radius], height=height, center=center, direction=direction), color="#504050", opacity=0.5)
        self.tires.append(pv.Cylinder(radius=outer_radius, height=height, center=center, direction=direction))
    
    def add_link(self, center: Sequence[float], direction: Sequence[float], radius: float, height: float, color: str = "gray") -> None:
        """
        ## Add Link

        Adds beam (link) Actor to pyvista window

        Parameters
        ----------
        center : Sequence[float]
            Center coordinate of beam
        direction : Sequence[float]
            Direction unit vector of beam
        radius : float
            Radius of beam
        height : float
            Length of beam
        color : str, optional
            Color of beam, by default "gray"

        Returns
        ----------
        None
        """
        self.pl.add_mesh(pv.Cylinder(center=center, direction=direction, radius=radius, height=height), color=color)
        self.links.append(pv.Cylinder(center=center, direction=direction, radius=radius, height=height))
    
    def add_node(self, center: Sequence[float], radius: float = 0.022225 / 2, color: str = "red") -> None:
        """
        ## Add Node

        Adds sphere Actor to pyvista window

        Parameters
        ----------
        center : Sequence[float]
            Center of sphere
        radius : float, optional
            Radius of sphere, by default 0.875/2
        color : str, optional
            Color of sphere, by default "red"
        
        Returns
        ----------
        None
        """
        self.pl.add_mesh(pv.Sphere(radius=radius, center=center), color=color)
        
    def add_slider(self, func: Callable, title: str, bounds: Tuple[float, float], pos: Tuple[Tuple[float, float], Tuple[float, float]]) -> None:
        """
        ## Add Slider

        Adds slider to Plotter window

        Parameters
        ----------
        func : Callable
            Function to execute slider values on
        title : str
            Title of slider
        bounds : Tuple[float, float]
            Limits of slider value
        pos : Tuple[Tuple[float, float], Tuple[float, float]]
            Position of slider (origin at lower left corner of window)
            - Start coordinate [Ratio X, Ratio Y]
            - End Coordinate [Ratio X, Ratio Y]
            - Max coordinate: [1, 1]
            - Min coordinate: [0, 0]
        
        Returns
        ----------
        None
        """
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

    def start_gif(self) -> None:
        """
        ## Start Gif

        Initializes gif from pyvista window

        Parameters
        ----------
        None

        Returns
        ----------
        None
        """
        self.pl.open_gif("jounce_sweep.gif", fps=144)
    
    def write_frame(self) -> None:
        """
        ## Write Frame

        Writes frame to gif from pyvista window

        Parameters
        ----------
        None

        Returns
        ----------
        None
        """
        self.pl.write_frame()
        self.pl.clear_actors()
    
    def clear(self):
        """
        ## Clear

        Clears all Actors from pyvista window

        Parameters
        ----------
        None

        Returns
        ----------
        None
        """
        self.pl.clear_actors()

    def end_gif(self):
        """
        ## End Gif

        Finalizes gif from pyvista window

        Parameters
        ----------
        None

        Returns
        ----------
        None
        """
        self.pl.close()

    def show_grid(self):
        """
        ## Show Grid

        Shows grid in pyvista window

        Parameters
        ----------
        None

        Returns
        ----------
        None
        """
        self.pl.show_grid()

    def show(self):
        """
        ## Show

        Shows pyvista window

        Parameters
        ----------
        None

        Returns
        ----------
        None
        """
        self.pl.show_axes()
        # for tire in self.tires:
        #     for link in self.links:
        #         result = tire.triangulate().boolean_intersection(other_mesh=link.triangulate())
        #         if bool(result) == True: break
        #     if bool(result) == True: break; print("Blow me")
        self.pl.show()
