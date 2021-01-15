from .object import Object
from .utils import *

class Joint(Object):
    def __init__(self, client, name=None, handle=None):
        super(Joint, self).__init__(client, name, handle)

    # simxGetJointForce
    # simxGetJointMaxForce
    # simxGetJointPosition
    # simxGetJointTargetPosition
    # simxGetJointTargetVelocity
    # simxSetJointForce
    # simxSetJointMaxForce
    # simxSetJointPosition
    # simxSetJointTargetPosition
    # simxSetJointTargetVelocity

    def get_joint_position(self):
        success, position = self.client.simxGetJointPosition(self.handle, self.client.simxServiceCall())
        if success:
            return position
        else:
            print("get joint {} position failed".format(self.name))

    def motor_enable(self, enable=True):
        success, res = set_object_int_parameter(self.client, self.handle, "sim.jointintparam_motor_enabled", int(enable))
        if not success or res != 1:
            print("set joint {} motor enable failed".format(self.name))
            print(res)

    def control_loop_enable(self, enable=True):
        success, res = set_object_int_parameter(self.client, self.handle, "sim.jointintparam_ctrl_enabled ", int(enable))
        if not success or res != 1:
            print("set joint {} control loop failed".format(self.name))
            print(res)

    """
    motor enabled, control loop not enabled
    """
    def get_joint_target_velocity(self):
        success, velocity = self.client.simxGetJointTargetVelocity(self.handle, self.client.simxServiceCall())
        if success:
            return velocity
        else:
            print("get joint {} target velocity failed".format(self.name))

    def set_joint_target_velocity(self, velocity):
        success, res = self.client.simxSetJointTargetVelocity(self.handle, velocity, self.client.simxServiceCall())
        if not success or res != 1:
            print("set joint {} target velocity failed".format(self.name))

    """
    motor enabled, control loop enabled
    """
    def get_joint_target_position(self):
        success, position = self.client.simxGetJointTargetPosition(self.handle, self.client.simxServiceCall())
        if success:
            return position
        else:
            print("get joint {} target position failed".format(self.name))

    def set_joint_target_position(self, position):
        success, res = self.client.simxSetJointTargetPosition(self.handle, position, self.client.simxServiceCall())
        if not success or res != 1:
            print("set joint {} target position failed".format(self.name))
            print(res)

    def set_mode(self, mode):
        cmd = "sim.setJointMode({handle}, {mode}, 1)".format(
            handle = self.handle,
            mode = mode
        )
        ret = self.client.simxExecuteScriptString(cmd, self.client.simxServiceCall())
        print(ret)