from scipy.linalg import logm, expm
from scipy.spatial.transform import Rotation as R

from .utils import *


class Kinematic():
    def __init__(self, arm):
        self.arm = arm
        self.origin_pose = self.arm.joints[-1].get_matrix()
        self.w = []
        self.p = []
        for i in range(len(self.arm.joints)):
            mat = self.arm.joints[i].get_matrix()
            self.w.append(mat[:3, 2])
            self.p.append(mat[:3, 3])

    def fk(self, joints_positions):
        mat = np.eye(4)
        for i in range(len(self.arm.joints)):
            t = np.concatenate([-np.cross(self.w[i], self.p[i]), self.w[i]], axis=0)
            T = expm(twist2mat(t) * joints_positions[i])
            mat = np.matmul(mat, T)
        return np.matmul(mat, self.origin_pose)

    def Ad(self, mat):
        ret = np.eye(6)
        R = mat[:3, :3]
        t = mat[:3, 3]
        ret[:3, :3] = R
        ret[:3, 3:] = np.matmul(screw(t), R)
        ret[3:, 3:] = R
        return ret

    def compute_jacobian(self, joints_positions):
        J = np.zeros([6, len(self.arm.joints)])
        mat = np.eye(4)
        for i in range(len(self.arm.joints)):
            t = np.concatenate([-np.cross(self.w[i], self.p[i]), self.w[i]], axis=0)
            t_m = twist2mat(t)
            Ad = self.Ad(mat)
            J[:, i] = np.matmul(Ad, t.T)
            T = expm(t_m * joints_positions[i])
            mat = np.matmul(mat, T)
        return J

    def ik_DLS(self, pose, lambd=0.1, max_iter=100, threshold=1e-6):
        T_target = np.eye(4)
        T_target[:3, 3] = pose[:3]
        T_target[:3, :3] = R.from_quat(pose[3:]).as_dcm()

        # random start joints positions
        degree = len(self.arm.joints)
        current_joints_positions = np.array(self.arm.get_joints_positions()) + np.random.randn(degree) * np.pi * (5/180)

        # iteration
        # T_target = e^[V] @ T_cur
        T_cur = self.fk(current_joints_positions)
        e_V = np.matmul(T_target, np.linalg.inv(T_cur))
        n = np.linalg.norm(e_V - np.eye(4))
        cnt = 0
        while n >= threshold and cnt < max_iter:
            # v = J @ theta_dot
            v = mat2twist(logm(e_V).real)
            J = self.compute_jacobian(current_joints_positions)

            # damped least squares
            J = np.concatenate([J, np.eye(degree) * lambd], axis=0)
            v = np.concatenate([v, np.zeros(degree)], axis=0)
            theta_dot = np.matmul(np.linalg.inv(np.matmul(J.T, J)), np.matmul(J.T, v))

            # update
            current_joints_positions += theta_dot

            T_cur = self.fk(current_joints_positions)
            e_V = np.matmul(T_target, np.linalg.inv(T_cur))
            n = np.linalg.norm(e_V - np.eye(4))

        target_joints_positions = current_joints_positions % (np.pi*2)
        filter = np.abs(target_joints_positions) > np.pi
        target_joints_positions[filter] = target_joints_positions[filter] - np.pi*2
        return target_joints_positions


    def ik_pinv(self, pose, max_iter=100, threshold=1e-6):
        T_target = np.eye(4)
        T_target[:3, 3] = pose[:3]
        T_target[:3, :3] = R.from_quat(pose[3:]).as_dcm()

        # random start joints positions
        degree = len(self.arm.joints)
        current_joints_positions = np.array(self.arm.get_joints_positions()) + np.random.randn(degree) * np.pi * (5/180)

        # iteration
        # T_target = e^[V] @ T_cur
        T_cur = self.fk(current_joints_positions)
        e_V = np.matmul(T_target, np.linalg.inv(T_cur))
        n = np.linalg.norm(e_V - np.eye(4))
        cnt = 0
        while n >= threshold and cnt < max_iter:
            # v = J @ theta_dot
            v = mat2twist(logm(e_V).real)
            J = self.compute_jacobian(current_joints_positions)

            # pseudo invers bu SVD
            U, d, V_T = np.linalg.svd(J, full_matrices=True)
            D_inv = np.eye(d.shape[0]) * (1/d)
            J_inv = np.matmul(np.matmul(V_T.T, D_inv), U.T)
            theta_dot = np.matmul(J_inv, v)

            # update
            current_joints_positions += theta_dot

            T_cur = self.fk(current_joints_positions)
            e_V = np.matmul(T_target, np.linalg.inv(T_cur))
            n = np.linalg.norm(e_V - np.eye(4))

        target_joints_positions = current_joints_positions % (np.pi*2)
        filter = np.abs(target_joints_positions) > np.pi
        target_joints_positions[filter] = target_joints_positions[filter] - np.pi*2
        return target_joints_positions
