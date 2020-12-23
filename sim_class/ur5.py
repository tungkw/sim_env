from .object import Object
from .joint import Joint


class UR5(Object):
    def __init__(self, client, obj_name=None, handle=None):
        super(UR5, self).__init__(client, obj_name=obj_name, handle=handle)
        res, joint_handles = self.client.simxGetObjectsInTree(self.handle, "sim.object_joint_type", 0, self.client.simxServiceCall())
        self.joints = [Joint(client, handle=joint_handle) for joint_handle in joint_handles[:6]]
        if not res:
            print("get UR5 arm joints failed")
        self.target = Object(client, obj_name="UR5_target")

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
        self.target.set_pose(pose, self.handle)

