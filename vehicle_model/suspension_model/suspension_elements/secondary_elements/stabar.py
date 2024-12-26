from vehicle_model.suspension_model.suspension_elements.primary_elements.link import Link
from vehicle_model.suspension_model.suspension_elements.primary_elements.node import Node

class Stabar:
    def __init__(self) -> None:
        pass

    """
    This should take 5 objects.

    Torsion bar : Link
    End of lever one : Node
    End of lever two : Node
    End of droplink one : Node
    End of droplink two : Node
    
    This object only needs updating when either droplink moves. Add "end of lever" one and two as children of "end of droplink" one and two, respectively.
    The droplink can be assumed to have constant length (actually technically it has a compliance. maybe the droplinks should be Link objects).

    When the end of either drop link moves, rotate the corresponding "end of lever" about the torsion bar until the droplink distance constraint is met.
    I believe that moving the end of the droplink up will always result in the corresponding "end of lever" also moving up, so constrain nonlinear
    solver to strictly positive or negative rotations, depending on the side and direction in which the end of the droplink moved.

    I've already added a "listeners" attribute to the Node object, so after all objects are initialized, the Stabar object needs to be added as a child
    of the "end of droplink" nodes. This way, any time the lower droplink Nodes move, the stabar is always updated.

    I also need to add a lumped torsional stiffness attribute to Stabar. I think geometry inputs can happen down the road, but not now.

    I believe that's everything for the Stabar, but I also need a way to fix nodes relative to each other (I've done this for pushrods before, but that
    only works in a specific situation where I rotate the pushrod mount about the control arm axis of rotation. Adding this would generalize the problem
    and add a lot of useful functionality, especially because Nightwatch uses this for actuating the stabar). I'm pretty sure this requires adding
    a coordinate system at the Node level and only using custom rotation methods. These methods would modify the Node coordinate system relative
    to the global coordinate system.

    It's very easy to implement translation here (I've already added it). I'm thinking of a way to do rotation, but I'll get some sleep before returning
    to this. I'm hoping there's a better way than what I described above.

    After the Stabar, I need to create the push/pull rod assembly. I can then add both of these to the quarter car model. I also need to add the tires.
    I can probably reuse most of the old code, as everything will still come from the .tir file.

    After that, I need to set up all of the graphics, using the new config file I've planned for. I also need to add the modal displacements, kin reports,
    and YMD generation back. Most of that can be recycled, but the new setup should make YMD generation more concise, and I just want to rewrite the kin
    report generation in a more concise way. The way I did it before was fine for the initial intent of the project, but I kept adding features and it
    got a bit out of hand.

    Once all of this is done, I think I can add instant screw axis representations and start solving some ODEs for transient response.
    
    All of this is probably overkill, but it sets up the framework for some pretty sick modeling tools. Once this is fully functional, maybe we can
    add a full user-interface, add some licensing, and release to the public.

    Okay, I'm gonna push this so I don't lose it, then I'll get some sleep.
    """