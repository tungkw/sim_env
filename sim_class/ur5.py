from .object import Object
from .joint import Joint
from .ompl import OMPL_arm
from .kinematic import Kinematic


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

        self.kinematic = Kinematic(self)
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

    def move_to(self, pose):
        start_joints_positions = self.get_joints_positions()
        # end_joints_positions = self.kinematic.ik_DLS(pose)
        end_joints_positions = self.kinematic.ik_pinv(pose)
        path = self.ompl.get_path(start_joints_positions, end_joints_positions.tolist())

        # follow path
        if path:
            for i in range(len(path)//6):
                self.set_joints_target_positions(path[i*6:(i+1)*6])


