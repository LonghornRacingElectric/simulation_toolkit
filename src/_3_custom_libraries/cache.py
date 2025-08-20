from functools import wraps
import copy


def SISO_cache(func):
    cache = {}

    @wraps(func)
    def wrapper(self, *args):
        key = (id(self.__class__), *args)

        if key in cache:
            return copy.deepcopy(cache[key])

        func(self, *args)  # Mutate self in-place
        cache[key] = copy.deepcopy(self)
        return self

    return wrapper