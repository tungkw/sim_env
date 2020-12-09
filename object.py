import sim
from object import Object

class Object:
    def __init__(self, obj_name):
        self.simxGetObjectHandle(obj_name)

    def get_image(self):
        return