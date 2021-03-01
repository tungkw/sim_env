from .object import Object
from .joint import Joint
from .ompl import OMPL_arm
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.linalg import logm, expm
import time

class UR5(Object):
    def __init__(self, client, name=None, handle=None):
        super(UR5, self).__init__(client, name=name, handle=handle)
        res, joint_handles = self.client.simxGetObjectsInTree(self.handle, "sim.object_joint_type", 0, self.client.simxServiceCall())
        self.joints = [Joint(client, handle=joint_handle) for joint_handle in joint_handles[:6]]
        if not res:
            print("get UR5 arm joints failed")
        self.target = Object(client, name="UR5_target")
        self.tip = Object(client, name="UR5_tip")
        res, self.ik_group_handle = self.client.simxExecuteScriptString("sim.getIkGroupHandle('UR5_IK_Group')", self.client.simxServiceCall())

        self.origin_pose = self.joints[-1].get_matrix()
        self.w = []
        self.p = []
        for i in range(len(self.joints)):
            mat = self.joints[i].get_matrix()
            self.w.append(mat[:3, 2])
            self.p.append(mat[:3, 3])
        self.ompl = OMPL_arm(client, joint_handles)

    def get_joints_positions(self):
        return [joint.get_joint_position() for joint in self.joints]

    def get_joints_target_positions(self):
        return [joint.get_joint_target_position() for joint in self.joints]

    def set_joints_target_positions(self, joints_positions):
        for i, joint in enumerate(self.joints):
            joint.set_joint_target_position(joints_positions[i])

    def get_joints_target_velocities(self):
        return [joint.get_joint_target_velocity() for joint in self.joints]

    def set_joints_target_velocities(self, joints_velocities):
        for i, joint in enumerate(self.joints):
            joint.set_joint_target_velocity(joints_velocities[i])

    def twist2mat(self, twist):
        v = twist[:3]
        w = twist[3:]
        mat = np.zeros([4,4])
        mat[0,1] = -w[2]
        mat[0,2] = w[1]
        mat[1,2] = -w[0]
        mat += -mat.T
        mat[:3, 3] = v
        return mat

    def fk(self, joints_positions):
        mat = np.eye(4)
        for i in range(len(self.joints)):
            t = np.concatenate([-np.cross(self.w[i], self.p[i]), self.w[i]], axis=0)
            T = expm(self.twist2mat(t) * joints_positions[i])
            mat = np.matmul(mat, T)
        return np.matmul(mat, self.origin_pose)

    def compute_jacobian(self, joints_positions):
        return np.eye(6)

    def ik(self, pose):
        T_target = np.eye(4)
        T_target[:3, 3] = pose[:3]
        T_target[:3, :3] = R.from_quat(pose[3:]).as_dcm()

        # random start joints positions
        target_joints_positions = np.random.rand(6) * np.pi

        # iteration
        T_cur = self.fk(target_joints_positions)
        # T_new = e^[V] @ T_old
        V = logm(np.matmul(T_target, np.linalg.inv(T_cur)))
        while np.linalg.norm(V) >= 0.01:
            # V = J @ theta_dot
            J = self.compute_jacobian(target_joints_positions)
            theta_dot = np.matmul(np.linalg.inv(J), V)
            target_joints_positions += theta_dot
            T_cur = self.fk(target_joints_positions)
            V = logm(np.matmul(T_target, np.linalg.inv(T_cur)))

        return target_joints_positions

    def move_to(self, pose):
        start_joints_positions = self.get_joints_positions()
        end_joints_positions = self.ik(pose)
        path = self.ompl.get_path(start_joints_positions, end_joints_positions)

        # follow path
        if path:
            for i in range(len(path)//6):
                self.set_joints_target_positions(path[i*6:(i+1)*6])


