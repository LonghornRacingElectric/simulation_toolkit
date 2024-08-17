from vehicle_model.suspension_model.suspension_elements.primary_elements.node import Node
from typing import Sequence


class CG(Node):
    """
    ## CG

    Center of gravity object

    Parameters
    ----------
    position : Sequence[float]
        Static position of CG 
    """
    def __init__(self, position: Sequence[float]) -> None:
        self.direction = [0, 0, 1]
        super().__init__(position)

    def plot_elements(self, plotter, verbose):
        """
        ## Plot Elements

        Plots CG

        Parameters
        ----------
        plotter : pv.Plotter
            Plotter object
        """
        if verbose:
            plotter.add_node(center=self.position, radius=1, color="black")