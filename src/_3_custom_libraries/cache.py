from functools import wraps
import copy


def SISO_global_cache(func):
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

def SISO_local_cache(func):
    cache = {}

    @wraps(func)
    def wrapper(self, arg):
        if arg in cache:
            return copy.deepcopy(cache[arg])  # return a copy to avoid mutation

        result = func(self, arg)

        cache[arg] = copy.deepcopy(result)
        return cache[arg]

    return wrapper