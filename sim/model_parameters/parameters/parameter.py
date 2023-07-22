from abc import abstractmethod


class Parameter:
    @abstractmethod
    def get(self):
        pass

    @abstractmethod
    def is_set(self) -> bool:
        pass
