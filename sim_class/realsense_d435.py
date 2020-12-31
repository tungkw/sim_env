from .vision_sensor import VisionSensor
from .proximity_sensor import ProximitySensor
import numpy as np
import cv2

class RealsenseD435(VisionSensor):
    def __init__(self, client, name=None, handle=None):
        super().__init__(client, name=name, handle=handle)
        farest_point = np.array([[0,0,1]]) @ np.linalg.inv(self.intrinsic).T * self.far_plane
        print(np.linalg.inv(self.intrinsic))
        print(np.array([[0,0,1]]) @ np.linalg.inv(self.intrinsic))
        print(farest_point)
        print(np.linalg.norm(farest_point))
        # res, normal_sensor_handle = self.client.simxExecuteScriptString(
        #     "sim.createProximitySensor("
        #     "   sim.proximitysensor_ray_subtype, "
        #     "   sim.objectspecialproperty_detectable_all, "
        #     "   {options}, "
        #     "   {{0, 0, 0, 0, 0, 0, 0, 0}}, "
        #     "   {{{offset}, {range_}, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}, "
        #     "   nil)".format(
        #         options=0,
        #         offset=self.near_plane,
        #         range_=np.linalg.norm(farest_point)
        #     ),
        #     self.client.simxServiceCall()
        # )
        # if not res or normal_sensor_handle == -1:
        #     print("create realsense normal handle failed")
        #     print(res, normal_sensor_handle)
        # else:
        #     self.normal_sensor = ProximitySensor(self.client, handle=normal_sensor_handle)
        #     self.normal_sensor.set_pose([0,0,0,0,0,0,1], self.handle)
        #     print(self.normal_sensor.get_pose(self.handle))
        #     self.client.simxSetObjectParent(self.normal_sensor.handle, self.handle, True, False, self.client.simxServiceCall())

    def get_depth(self, with_normal=False):
        depth = super(RealsenseD435, self).get_depth()
        cv2.imshow("depth_", depth)
        depth = self.add_noise(depth)
        return depth

    def add_noise(self, z):
        h, w = z.shape

        # axial noise
        I, J = np.meshgrid(np.arange(h), np.arange(w), indexing='ij')
        points = np.stack([J, I, np.ones_like(J)], axis=-1)
        points = points @ np.linalg.inv(self.intrinsic).T
        points = points * np.expand_dims(z, axis=-1)
        # theta_y
        points_l, points_r = np.copy(points), np.copy(points)
        points_l[:, 1:, :] = points[:, :-1, :]
        points_r[:, :-1, :] = points[:, 1:, :]
        dx = points_r - points_l
        cos_theta_y = (dx @ np.array([[1, 0, 0]]).T)[...,0] / np.linalg.norm(dx, axis=-1)
        theta_y = np.arccos(cos_theta_y)
        theta_y[theta_y>np.pi/2] = np.pi - theta_y[theta_y>np.pi/2]
        # theta_x
        points_u, points_b = np.copy(points), np.copy(points)
        points_u[1:, :, :] = points[:-1, :, :]
        points_b[:-1, :, :] = points[1:, :, :]
        dy = points_b - points_u
        cos_theta_x = (dy @ np.array([[0, 1, 0]]).T)[...,0] / np.linalg.norm(dy, axis=-1)
        theta_x = np.arccos(cos_theta_x)
        theta_x[theta_x>np.pi/2] = np.pi - theta_x[theta_x>np.pi/2]
        # axial noise model
        sigma_z = 0.00163 + 0.0007278*z + 0.003949*z**2 + 0.022*z**1.5*theta_y/(np.pi/2-theta_y)**2 + 0.022*z**1.5*theta_x/(np.pi/2-theta_x)**2

        # lateral noise
        grey = (z/1 * 255).astype(np.uint8)
        edges = cv2.Canny(grey, 100, 200)
        sigma_z[edges>0] = sigma_z[edges>0] + 0.0407

        z = z + np.random.normal(0, sigma_z)
        z[np.logical_or(z>self.far_plane, z<self.near_plane)] = self.far_plane
        return z

    def get_normal(self, points):
        res, normal = self.client.simxCallScriptFunction(
            'getNormal@normal_sensor',
            'sim.scripttype_childscript',
            [],
            self.client.simxServiceCall())
        print(res)
        normal = np.array(normal).reshape(self.resolution+[3])
        return normal
