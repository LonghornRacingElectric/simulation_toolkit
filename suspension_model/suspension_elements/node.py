import numpy as np


class Node:

    def __init__(self, position: list) -> None:
        self.position = np.array(position)
        self.initial_position = position

    def reset(self):
        self.position = self.initial_position