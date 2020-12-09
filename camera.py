import sim
from object import Object

class Camera(Object):
    def __init__(self, obj_name):
        super(Camera, self).__init__(obj_name)
        self.simxGetObjectHandle(obj_name)

    def get_image(self):
        return