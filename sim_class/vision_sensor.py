from .object import Object
from .utils import *
import numpy as np
from scipy.spatial.transform import Rotation as R

# v-rep vision sensor y-axis is along with up-direction, as z-axis along with eye-direction
# normal image y-axis is opposite

# storing in inverse y-axis, i.e. the inverse row

# perspective angle is on y-z plane

class VisionSensor(Object):
    def __init__(self, client, name=None, handle=None):
        super(VisionSensor, self).__init__(client, name=name, handle=handle)

        _, self.xdim = get_object_int_parameter(self.client, self.handle, "sim.visionintparam_resolution_x")
        _, self.ydim = get_object_int_parameter(self.client, self.handle, "sim.visionintparam_resolution_y")
        self.resolution = [self.ydim, self.xdim]
        _, self.angle = get_object_float_parameter(self.client, self.handle, "sim.visionfloatparam_perspective_angle")
        _, self.near_plane = get_object_float_parameter(self.client, self.handle, "sim.visionfloatparam_near_clipping")
        _, self.far_plane = get_object_float_parameter(self.client, self.handle, "sim.visionfloatparam_far_clipping")
        print("camera {} resolution".format(self.name), self.xdim, self.ydim)
        print("camera {} perspective angle".format(self.name), self.angle)

        b = self.ydim/(2*np.tan(self.angle/2))
        self.intrinsic = np.array(
            [
                [b, 0, self.xdim/2],
                [0, b, self.ydim/2],
                [0, 0, 1]
            ]
        )
        print("camera {} intrinsic\n".format(self.name), self.intrinsic)

        self.fix_rotation = np.array(
            [
                [-1, 0, 0, 0],
                [0, -1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ]
        )

    def get_image(self, grey=False):
        success, _, image_bytes = self.client.simxGetVisionSensorImage(self.handle, grey, self.client.simxServiceCall())
        if success:
            if grey:
                image = np.frombuffer(image_bytes, np.uint8, len(image_bytes)).reshape([self.ydim, self.xdim])
            else:
                image = np.frombuffer(image_bytes, np.uint8, len(image_bytes)).reshape([self.ydim, self.xdim, 3])
            image = image[::-1, ...]
            return image
        else:
            return None

    def get_depth(self):
        success, _, image_bytes = self.client.simxGetVisionSensorDepthBuffer(self.handle, True, True, self.client.simxServiceCall())
        if success:
            depth = np.frombuffer(image_bytes, np.float32, len(image_bytes)//4).reshape([self.ydim, self.xdim])
            depth = np.copy(depth[::-1, ...])
            return depth
        else:
            return None

    def get_matrix(self, base_frame_object_handle=-1):
        matrix = super(VisionSensor, self).get_matrix(base_frame_object_handle)
        matrix = np.matmul(self.fix_rotation, matrix)
        return matrix

    def get_euler(self, base_frame_object_handle=-1):
        euler = super(VisionSensor, self).get_euler(base_frame_object_handle)
        euler = (R.from_matrix(self.fix_rotation[:3,:3]) * R.from_euler("xyz", euler, degrees=False)).as_euler("xyz", degrees=False)
        return euler

    def get_quaternion(self, base_frame_object_handle=-1):
        quaternion = super(VisionSensor, self).get_quaternion(base_frame_object_handle)
        quaternion = (R.from_matrix(self.fix_rotation[:3,:3]) * R.from_quat(quaternion)).as_quat()
        return quaternion

    def get_pose(self, base_frame_object_handle=-1):
        pose = super(VisionSensor, self).get_pose(base_frame_object_handle)
        pose[3:] = (R.from_matrix(self.fix_rotation[:3,:3]) * R.from_quat(pose[3:])).as_quat()
        return pose

    def set_matrix(self, matrix, base_frame_object_handle=-1):
        matrix = np.matmul(np.linalg.inv(self.fix_rotation), matrix)
        super(VisionSensor, self).set_matrix(matrix, base_frame_object_handle)

    def set_euler(self, euler, base_frame_object_handle=-1):
        euler = (R.from_matrix(np.linalg.inv(self.fix_rotation[:3,:3])) * R.from_euler("xyz", euler, degrees=False)).as_euler("xyz", degrees=False)
        super(VisionSensor, self).set_euler(euler, base_frame_object_handle)

    def set_quaternion(self, quaternion, base_frame_object_handle=-1):
        quaternion = (R.from_matrix(np.linalg.inv(self.fix_rotation[:3,:3])) * R.from_quat(quaternion)).as_quat()
        super(VisionSensor, self).set_quaternion(quaternion, base_frame_object_handle)

    def set_pose(self, pose, base_frame_object_handle=-1):
        pose[3:] = (R.from_matrix(np.linalg.inv(self.fix_rotation[:3,:3])) * R.from_quat(pose[3:])).as_quat()
        super(VisionSensor, self).set_pose(pose, base_frame_object_handle)


    def set_pov(self, enable=True, focal_blur=False, paras=[0.0, 0.0, 1]):
        """
        :param enable:
        :param focal_blur:
        :param paras: blur_distance=0.0, aperture=0.0, blur_sample=1
        :return:
        """
        if not enable:
            set_object_int_parameter(self.client, self.handle, "sim.visionintparam_render_mode", 0)
        else:
            set_object_int_parameter(self.client, self.handle, "sim.visionintparam_render_mode", 3)
            if focal_blur:
                set_object_int_parameter(self.client, self.handle, "sim.visionintparam_pov_focal_blur", 1)
                set_object_float_parameter(self.client, self.handle, "sim.visionfloatparam_pov_blur_distance", paras[0])
                set_object_float_parameter(self.client, self.handle, "sim.visionfloatparam_pov_aperture", paras[1])
                set_object_int_parameter(self.client, self.handle, "sim.visionintparam_pov_blur_sampled", paras[2])
            else:
                set_object_int_parameter(self.client, self.handle, "sim.visionintparam_pov_focal_blur", 0)
