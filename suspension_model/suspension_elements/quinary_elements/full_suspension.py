from suspension_model.suspension_elements.quaternary_elements.axle import Axle
from suspension_model.suspension_elements.secondary_elements.cg import CG


class FullSuspension:
    def __init__(self, Fr_axle: Axle, Rr_axle: Axle, cg: CG) -> None:
        self.Fr_axle = Fr_axle
        self.Rr_axle = Rr_axle
        self.cg = cg

        self.elements = [self.Fr_axle, self.Rr_axle, self.cg]

    def roll(self, angle: float):
        self.Fr_axle.roll(angle=angle)
        self.Rr_axle.roll(angle=angle)

    def plot_elements(self, plotter, verbose):
        if verbose:
            # plot roll axis
            pass
        for axle in self.elements:
            axle.plot_elements(plotter=plotter, verbose=verbose)
