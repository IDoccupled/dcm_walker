class Flag:
    def __init__(self):
        self.__flag = 'idle'
    def set_idle(self):
        self.__flag = 'idle'
    def set_starting(self):
        self.__flag = 'starting'
    def set_running(self):
        self.__flag = 'running'
    def set_stopping(self):
        self.__flag = 'stopping'
    def is_idle(self) -> bool:
        return self.__flag == 'idle'
    def is_starting(self) -> bool:
        return self.__flag == 'starting'
    def is_running(self) -> bool:
        return self.__flag == 'running'
    def is_stopping(self) -> bool:
        return self.__flag == 'stopping'  