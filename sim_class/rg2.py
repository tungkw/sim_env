from .object import Object


class RG2(Object):
    def __init__(self, client, name=None, handle=None):
        super(RG2, self).__init__(client, obj_name=name, handle=handle)

