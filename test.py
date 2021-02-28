
from b0_python import b0RemoteApi
from sim_class import *
import cv2
import numpy as np
import time
from scipy.spatial.transform import Rotation as R


if __name__ == '__main__':
    with b0RemoteApi.RemoteApiClient('b0RemoteApi_V-REP', 'b0RemoteApi') as client:

        client.simxStartSimulation(client.simxServiceCall())

        time.sleep(3)

        arm = UR5(client, name="UR5")

        ompl = OMPL_arm(client, arm)


        # joints_target_positions = np.array([np.pi/2, 0, -np.pi/2, 0, np.pi/2, 0])
        # # arm.set_joints_target_positions(joints_target_positions)
        # # time.sleep(5)


        # while True:
        #     joints_current_positions = arm.get_joints_positions()
        #     print(joints_current_positions)
        #     print(arm.jacobian @ joints_target_positions.T)
        #     print(joints_target_positions @ arm.jacobian


        t = [0.5, 0.0, 0.5]
        q = R.from_euler("xyz", [180, 0, 0], degrees=True).as_quat().tolist()
        cur_pose = t + q

        arm.move_to(cur_pose)
        time.sleep(5)



        # t = [0.3, 0.3, 0.5]
        # q = R.from_euler("xyz", [180, 0, 0], degrees=True).as_quat().tolist()
        # cur_pose = t + q
        # arm.move_to(cur_pose)

        # t = [0.5, 0.0, 0.5]
        # q = R.from_euler("xyz", [180, 0, 0], degrees=True).as_quat().tolist()
        # cur_pose = t + q
        # arm.move_to(cur_pose)
        # time.sleep(5)

        # cam = VisionSensor(client, name="realsense_D435")
        # pose = cam.get_matrix()
        # print(pose)
        # point = np.array([[0,0,0,1.0]])
        # print(point @ pose.T)

        # client.simxStopSimulation(client.simxServiceCall())

        # doNextStep = True
        #
        #
        # def simulationStepStarted(msg):
        #     simTime = msg[1][b'simulationTime'];
        #     print('Simulation step started. Simulation time: ', simTime)
        #
        #
        # def simulationStepDone(msg):
        #     simTime = msg[1][b'simulationTime'];
        #     print('Simulation step done. Simulation time: ', simTime);
        #     global doNextStep
        #     doNextStep = True
        #
        #
        # client.simxSynchronous(True)
        # client.simxGetSimulationStepStarted(client.simxDefaultSubscriber(simulationStepStarted));
        # client.simxGetSimulationStepDone(client.simxDefaultSubscriber(simulationStepDone));
        # client.simxStartSimulation(client.simxDefaultPublisher())
        #
        # startTime = time.time()
        # while time.time() < startTime + 5:
        #     if doNextStep:
        #         doNextStep = False
        #         client.simxSynchronousTrigger()
        #     client.simxSpinOnce()
        #
        client.simxStopSimulation(client.simxDefaultPublisher())


