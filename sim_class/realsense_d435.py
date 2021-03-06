from .vision_sensor import VisionSensor
from .proximity_sensor import ProximitySensor
import numpy as np

class RealsenseD435(VisionSensor):
    def __init__(self, client, name=None, handle=None):
        super().__init__(client, name=name, handle=handle)
        # farest_point = np.array([[0,0,1]]) @ np.linalg.inv(self.intrinsic).T * self.far_plane
        # print(np.linalg.inv(self.intrinsic))
        # print(np.array([[0,0,1]]) @ np.linalg.inv(self.intrinsic))
        # print(farest_point)
        # print(np.linalg.norm(farest_point))
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

    def get_depth(self, axial_noise=False, lateral_noise=False):
        depth = super(RealsenseD435, self).get_depth()
        depth = self.add_noise(depth, axial_noise=axial_noise, lateral_noise=lateral_noise)
        return depth

    def add_noise(self, z, axial_noise=False, lateral_noise=False):
        h, w = z.shape
        I, J = np.meshgrid(np.arange(h), np.arange(w), indexing='ij')
        points = np.stack([J, I, np.ones_like(J)], axis=-1)
        points = points @ np.linalg.inv(self.intrinsic).T
        points = points * np.expand_dims(z, axis=-1)
        z[z>=self.far_plane-0.01] = 0


        # axial noise
        if axial_noise:
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
            sigma_z = 0.00163 + 0.0007278*z + 0.003949*(z**2)\
                      + 0.022*(z**1.5)*theta_y/((np.pi/2-theta_y)**2)\
                      + 0.022*(z**1.5)*theta_x/((np.pi/2-theta_x)**2)
            z = z + np.random.normal(scale=sigma_z)
            z[np.logical_or(z>self.far_plane, z<self.near_plane)] = 0

            # random miss
            missing_mask = np.logical_or(np.random.uniform(size=z.shape) > cos_theta_x**0.1, np.random.uniform(size=z.shape) > cos_theta_y**0.1)
            missing_mask = np.logical_and(missing_mask, np.random.uniform(size=z.shape) > z/self.far_plane)
            z[missing_mask] = 0.0

        # lateral noise
        if lateral_noise:
            sigma_l = np.zeros_like(z)
            sigma_l[z>0] = 0.00432
            points_n = np.copy(points)
            points_n[..., 0] = points_n[..., 0] + np.random.normal(scale=sigma_l)
            points_n[..., 1] = points_n[..., 1] + np.random.normal(scale=sigma_l)
            points_n = points_n / points_n[..., [2]] @ self.intrinsic.T
            u = points_n[..., 0]
            v = points_n[..., 1]
            # u = np.copy(J).astype(float) + np.random.normal(scale=sigma_l)
            # v = np.copy(I).astype(float) + np.random.normal(scale=sigma_l)
            valid_j = np.logical_and(u>=0, u<self.xdim)
            u[~valid_j] = J[~valid_j]
            valid_i = np.logical_and(v>=0, v<self.ydim)
            v[~valid_i] = I[~valid_i]
            z = z[v.astype(I.dtype), u.astype(J.dtype)]

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
