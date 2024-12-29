from vehicle_model.suspension_model.suspension_elements.tertiary_elements.serial_link import SerialLink
from vehicle_model.suspension_model.suspension_elements.secondary_elements.bellcrank import Bellcrank
from vehicle_model.suspension_model.suspension_elements.secondary_elements.spring import Spring
from vehicle_model.suspension_model.suspension_elements.primary_elements.link import Link
from vehicle_model.assets.misc_math import directional_root, rotation_matrix, unit_vec
from typing import Sequence, Union, Tuple, cast
import numpy as np


class PushPullRod:
    """
    ## Push/Pull Rod

    Push/pull rod object

    Parameters
    ----------
    outboard_rod : Link
        Link representing push or pull rod

    spring : Spring
        Spring attached to rigid Link
        
    inboard_rod : Union[Link, None], optional
        Link representing rod from bellcrank to Spring -> frame, by default None

    bellcrank : Union[Bellcrank, None], optional
        Bellcrank connecting outboard_rod to inboard_rod, by default None
    """
    """
    **kwargs : dict[str, Union[str, Link, str]]
    Optional keyword arguments. The following keys are supported:

        `bellcrank` : Bellcrank
            Bellcrank object sharing nodes with push/pull rod and inboard link

        `inboard_link` : Link
            Link object connecting bellcrank to damper

        `damper` : Damper
            Damper object connecting inboard_link to frame
    """
    def __init__(self, outboard_rod: Link, spring: Spring, inboard_rod: Union[Link, None] = None, bellcrank: Union[Bellcrank, None] = None):
        self.outboard_rod: Link = outboard_rod
        self.spring: Spring = spring
        
        self.full: bool
        self.tension: bool

        if bool(inboard_rod) ^ bool(bellcrank):
            raise Exception("PushPullRod must include both inboard_rod and bellcrank")
        elif inboard_rod:
            self.full = True
        else:
            self.full = False
        
        if self.full:
            self.bellcrank: Bellcrank = cast(Bellcrank, bellcrank)
            self.inboard_rod: Link = cast(Link, inboard_rod)

            self.serial_link = SerialLink(rigid_link=self.inboard_rod, compliant_link=self.spring)
            self.bellcrank.nodes[-1].add_listener(self.serial_link)
            self.outboard_rod.outboard_node.add_listener(self)

            self.bellcrank_angle: float = 0
        else:
            self.serial_link = SerialLink(rigid_link=self.outboard_rod, compliant_link=self.spring)
            self.outboard_rod.outboard_node.add_listener(self.serial_link)

    def update(self):
        outb_p = self.outboard_rod.outboard_node

        if not outb_p.rotation_angle:
            differential_point = np.array(outb_p.initial_position) + np.array(outb_p.translation) / 1000
        else:
            diff_rot_mat = rotation_matrix(unit_vec=outb_p.rotation_direction, theta=np.sign(outb_p.rotation_angle) * 0.01 * np.pi / 180)
            differential_point = (np.matmul(diff_rot_mat, (outb_p - outb_p.rotation_origin).position) + outb_p.rotation_origin.position)

        # Determine whether differential step in the rotation direction results in tension or compression
        initial_length = self.outboard_rod.initial_length
        differential_length = np.linalg.norm(np.array(self.outboard_rod.inboard_node.initial_position) - differential_point).__float__()

        # Find direction of rotation
        r_vec = np.array(outb_p.initial_position) - np.array(self.bellcrank.pivot.position)

        if differential_length > initial_length:
            self.tension = True
            f_vec = unit_vec(p1=self.outboard_rod.inboard_node.position, p2=outb_p.initial_position)
        elif differential_length < initial_length:
            self.tension = False
            f_vec = unit_vec(p1=outb_p.initial_position, p2=self.outboard_rod.inboard_node.position)
        else:
            raise Exception("Desired bellcrank rotation inverts linkage")

        moment = np.cross(r_vec, f_vec)
        projected_moment = np.dot(moment, self.bellcrank.pivot_direction)

        if projected_moment > 0:
            self.bellcrank_angle = directional_root(func=self._bellcrank_eqn, x0=0, bounds=(0, np.pi/2), tol=1e-5, args=[])
        else:
            self.bellcrank_angle = directional_root(func=self._bellcrank_eqn, x0=0, bounds=(-np.pi/2, 0), tol=1e-5, args=[])

    def _bellcrank_eqn(self, x: float, args: Sequence):
        self.bellcrank.rotate(x)

        return self.outboard_rod.length - self.outboard_rod.initial_length