from abc import abstractmethod


class Parameter:
    @abstractmethod
    def get(self):
        pass
