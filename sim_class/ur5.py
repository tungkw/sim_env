from .object import Object
from .joint import Joint
import numpy as np
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
        # res, self.ik_group_handle = self.client.simxExecuteScriptString("sim.getIkGroupHandle(\"UR5_IK_Group\")", self.client.simxServiceCall())
        # ret = self.client.simxExecuteScriptString("sim.computeJacobian({}, 1, nil)".format(self.ik_group_handle), self.client.simxServiceCall())
        # print(ret)
        # cmd = "sim.getIkGroupMatrix({handle}, 0)".format(handle=self.ik_group_handle)
        # res, self.jacobian = self.client.simxExecuteScriptString(cmd, self.client.simxServiceCall())
        # self.jacobian = np.array(self.jacobian).reshape(6, 6)
        # self.inv_jacobian = np.linalg.inv(self.jacobian)

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

    def move_to(self, pose):
        self.target.set_pose(pose)


