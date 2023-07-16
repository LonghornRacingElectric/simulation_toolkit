
class Vector:
    def __init__(self):
        pass

    """
    The __contains__ function allows you to do stuff like:
    
    if "torque_request" in vector:
        print("ok")
    else:
        print("error")
    """
    def __contains__(self, item):
        return hasattr(self, item)

    """
    TODO prevent possible bad bugs by restricting which attrs a system can access.
    Possible implementation is when an attr is set, store a list of the "set" ones so only those can be accessed.
    Accessing an unset one would throw an error to prevent weird, hard-to-detect issues.
    Also add warnings for parameters that weren't used at all.
    """