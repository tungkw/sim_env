from .object import Object
from .utils import *

class ProximitySensor(Object):
    def __init__(self, client, name=None, handle=None):
        super().__init__(client, name=name, handle=handle)

    def read(self):
        ret = self.client.simxReadProximitySensor(self.handle, self.client.simxServiceCall())
        if ret [0] and ret[1] == 1:
            res, state, dist, point, obj_handle, normal = ret
            return dist, point, obj_handle, normal
        else:
            return None, None, None, None