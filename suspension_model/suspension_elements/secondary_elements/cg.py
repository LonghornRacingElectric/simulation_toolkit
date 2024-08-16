from suspension_model.suspension_elements.primary_elements.node import Node


class CG(Node):
    def __init__(self, position: list) -> None:
        self.direction = [0, 0, 1]
        super().__init__(position)

    def plot_elements(self, plotter, verbose):
        if verbose:
            plotter.add_node(center=self.position, radius=1, color="black")