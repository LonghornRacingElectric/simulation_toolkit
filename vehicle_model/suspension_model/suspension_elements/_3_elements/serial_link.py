from vehicle_model.suspension_model.suspension_elements._1_elements.link import Link
from vehicle_model.assets.misc_math import unit_vec

import numpy as np


class SerialLink:
    """
    ## Serial Link

    Serial link object
    - Intended for two coincident, fixed links

    Parameters
    ----------
    rigid_link : Link
        Link with negligible deformation

    compliant_link : Link
        Link with significant deformation
    """
    def __init__(self, rigid_link: Link, compliant_link: Link) -> None:
        self.rigid_link = rigid_link
        self.compliant_link = compliant_link

        self.rigid_link.outboard_node.add_listener(self)

    def update(self) -> None:
        """
        ## Update

        - Updates rigid_link to match initial geometry
        - Updates compliant_link to remain coincident with rigid_link
        """
        direction = unit_vec(p1=self.rigid_link.outboard_node.position, p2=self.compliant_link.inboard_node.position)
        new_rigid_inboard = np.array(direction) * self.rigid_link.initial_length + np.array(self.rigid_link.outboard_node.position)

        translation = new_rigid_inboard - np.array(self.rigid_link.inboard_node.initial_position)

        self.rigid_link.inboard_node.translate(translation=translation)