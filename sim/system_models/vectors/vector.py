
class Vector:
    def __init__(self):
        self.strict = False

    """
    TODO prevent possible sneaky bugs by restricting which attrs a system can access.
    Also add warnings for parameters that weren't used at all.
    """

    def __getattribute__(self, name: str):
        return super().__getattribute__(name)  # TODO replace

    def __setattr__(self, name: str, value):
        return super().__setattr__(name, value)  # TODO replace

    def expect_get(self, names: list[str]):
        pass  # TODO implement

    def expect_set(self, names: list[str]):
        pass  # TODO implement

    def confirm_expectations(self):
        pass  # TODO implement
