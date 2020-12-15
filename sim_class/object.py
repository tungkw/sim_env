import numpy as np
from .utils import *

# euler order:

class Object:
    def __init__(self, client, obj_name=None, handle=None):
        self.client = client
        if handle:
            self.handle = handle
            _, self.name = self.client.simxGetObjectName(self.handle, False, self.client.simxServiceCall())
        elif obj_name:
            self.name = obj_name
            success, self.handle = self.client.simxGetObjectHandle(obj_name, self.client.simxServiceCall())
            if not success:
                print("get object handle by name failed")
        else:
            print("create object class failed, either name or handle should be given")

    def get_matrix(self, base_frame_object_handle=-1):
        """
        :param base_frame_object_handle: -1:absolute, handle_parent
        :return: 4x4 transformation matrix
        """
        success, matrix_list = self.client.simxGetObjectMatrix(self.handle, base_frame_object_handle, self.client.simxServiceCall())
        if success:
            matrix = np.identity(4)
            matrix[0] = matrix_list[:4]
            matrix[1] = matrix_list[4:8]
            matrix[2] = matrix_list[8:]
            return matrix
        else:
            return None

    def get_euler(self, base_frame_object_handle=-1):
        """
        :param base_frame_object_handle: -1:absolute, handle_parent
        :return: [alpha,beta,gamma]
        """
        success, euler = self.client.simxGetObjectOrientation(self.handle, base_frame_object_handle, self.client.simxServiceCall())
        if success:
            return np.array(euler)
        else:
            return None

    def get_position(self, base_frame_object_handle=-1):
        """
        :param base_frame_object_handle: -1:absolute, handle_parent
        :return: [X,Y,Z]
        """
        success, position = self.client.simxGetObjectPosition(self.handle, base_frame_object_handle, self.client.simxServiceCall())
        if success:
            return np.array(position)
        else:
            return None


    def get_quaternion(self, base_frame_object_handle=-1):
        """
        :param base_frame_object_handle: -1:absolute, handle_parent
        :return: [Qx,Qy,Qz,Qw]
        """
        success, quaternion = self.client.simxGetObjectQuaternion(self.handle, base_frame_object_handle, self.client.simxServiceCall())
        if success:
            return np.array(quaternion)
        else:
            return None

    def get_pose(self, base_frame_object_handle=-1):
        """
        :param base_frame_object_handle: -1:absolute, handle_parent
        :return: [X,Y,Z,Qx,Qy,Qz,Qw] = position + quaternion
        """
        success, pose = self.client.simxGetObjectPose(self.handle, base_frame_object_handle, self.client.simxServiceCall())
        if success:
            return np.array(pose)
        else:
            return None

    def get_velocity(self):
        success, v, w = self.client.simxGetObjectVelocity(self.handle, self.client.simxSeviceCall())
        if success:
            return np.array(v), np.array(w)
        else:
            return None, None

    def get_parent(self):
        success, parent = self.client.simxGetObjectParent(self.handle, self.client.simxServiceCall())
        if success and parent != -1:
            return Object(self.client, handle=parent)
        else:
            return None

    def get_bounding_box(self):
        _, xmin = get_object_float_parameter(self.client, self.handle, "sim.objfloatparam_objbbox_min_x")
        _, ymin = get_object_float_parameter(self.client, self.handle, "sim.objfloatparam_objbbox_min_y")
        _, zmin = get_object_float_parameter(self.client, self.handle, "sim.objfloatparam_objbbox_min_z")
        _, xmax = get_object_float_parameter(self.client, self.handle, "sim.objfloatparam_objbbox_max_x")
        _, ymax = get_object_float_parameter(self.client, self.handle, "sim.objfloatparam_objbbox_max_y")
        _, zmax = get_object_float_parameter(self.client, self.handle, "sim.objfloatparam_objbbox_max_z")
        return [xmin, ymin, zmin, xmax, ymax, zmax]

    def get_model_box(self):
        _, xmin = get_object_float_parameter(self.client, self.handle, "sim.objfloatparam_modelbbox_min_x")
        _, ymin = get_object_float_parameter(self.client, self.handle, "sim.objfloatparam_modelbbox_min_y")
        _, zmin = get_object_float_parameter(self.client, self.handle, "sim.objfloatparam_modelbbox_min_z")
        _, xmax = get_object_float_parameter(self.client, self.handle, "sim.objfloatparam_modelbbox_max_x")
        _, ymax = get_object_float_parameter(self.client, self.handle, "sim.objfloatparam_modelbbox_max_y")
        _, zmax = get_object_float_parameter(self.client, self.handle, "sim.objfloatparam_modelbbox_max_z")
        return [xmin, ymin, zmin, xmax, ymax, zmax]

    def set_matrix(self, matrix, base_frame_object_handle=-1):
        """
        :param matrix:
        :param base_frame_object_handle:
        :return:
        """
        matrix_ = matrix.reshape(-1).tolist()[:-4]
        success, res = self.client.simxSetObjectMatrix(self.handle, base_frame_object_handle, matrix_, self.client.simxServiceCall())
        if not success:
            print("set matrix failed")

    def set_euler(self, euler, base_frame_object_handle=-1):
        """
        :param euler:
        :param base_frame_object_handle:
        :return:
        """
        success, res = self.client.simxSetObjectOrientation(self.handle, base_frame_object_handle, np.array(euler).tolist(), self.client.simxServiceCall())
        if not success:
            print("set euler failed")

    def set_position(self, position, base_frame_object_handle=-1):
        """
        :param position:
        :param base_frame_object_handle:
        :return:
        """
        success, res = self.client.simxSetObjectPosition(self.handle, base_frame_object_handle, np.array(position).tolist(), self.client.simxServiceCall())
        if not success:
            print("set position failed")

    def set_quaternion(self, quaternion, base_frame_object_handle=-1):
        """
        :param base_frame_object_handle: -1:absolute, handle_parent
        :return: [Qx,Qy,Qz,Qw]
        """
        success, res = self.client.simxSetObjectQuaternion(self.handle, base_frame_object_handle, np.array(quaternion).tolist(), self.client.simxServiceCall())
        if not success:
            print("set quaternion failed")

    def set_pose(self, pose, base_frame_object_handle=-1):
        """
        :param pose:
        :param base_frame_object_handle:
        :return:
        """
        success, res = self.client.simxSetObjectPose(self.handle, base_frame_object_handle, np.array(pose).tolist(), self.client.simxServiceCall())
        if not success:
            print("set pose failed")