from typing import Protocol


class Updateable(Protocol):
    """
    ## Updateable

    Any object that contains the update() method

    Parameters
    ----------
    None
    """
    def update(self) -> None:
        pass