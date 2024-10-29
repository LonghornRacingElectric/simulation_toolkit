class my_ccycle():
    def __init__(self, iter):
        self.iter = iter
    def __iter__(self):
        return self
    def __next__(self):
        for val in self.iter:
            return val

a = my_ccycle([1, 2, 3])