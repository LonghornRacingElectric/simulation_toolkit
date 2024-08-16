import numpy as np


class Node:

    def __init__(self, position: list) -> None:
        self.position = np.array(position)
        self.initial_position = position
        
        self.plotted = False
        self.plotted_node = None

    def reset(self):
        self.position = self.initial_position

    def plot_elements(self, plotter):
        # if self.plotted:
        #     self.plotted_node.points = self.position
        # else:
        if 1e9 not in np.abs(self.position):
            plotter.add_node(center=self.position)

        # self.plotted = True
        