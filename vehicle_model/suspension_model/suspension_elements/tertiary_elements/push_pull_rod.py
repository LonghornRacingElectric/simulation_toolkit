from vehicle_model.suspension_model.suspension_elements.primary_elements.link import Link
from vehicle_model.assets.misc_math import rotation_matrix, unit_vec
from typing import Sequence, Union, Tuple
import numpy as np


class PushPullRod:
    """
    ## Push/Pull Rod

    Push/pull rod object

    Parameters
    ----------
    push_pull_rod : Link
        Link representing push or pull rod

    **kwargs : dict[str, Union[str, Link, str]]
        Optional keyword arguments. The following keys are supported:

            `bellcrank` : Bellcrank
                Bellcrank object sharing nodes with push/pull rod and inboard link

            `inboard_link` : Link
                Link object connecting bellcrank to damper

            `damper` : Damper
                Damper object connecting inboard_link to frame
    """
    def __init__(self, push_pull_rod: Link, **kwargs: dict[str, Union[str, Link, str]]):
        pass