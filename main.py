import b0RemoteApi
from sim_class import *
import cv2
import numpy as np
import time

if __name__ == '__main__':
    with b0RemoteApi.RemoteApiClient('test_node', 'b0RemoteApi') as client:


        client.simxStartSimulation(client.simxServiceCall())

        # cube = Object(client, obj_name="test")
        # # cube_test = Object(client, handle=cube.handle)
        # # print(cube_test.name)
        # print(cube.get_model_box())
        #
        #
        cam = VisionSensor(client, "test_cam")
        # cam.set_matrix(np.array([
        #     [1, 0, 0, 0],
        #     [0, -1, 0, 0],
        #     [0, 0, -1, 1],
        #     [0, 0, 0, 1]
        # ]))
        from scipy.spatial.transform import Rotation as R
        pose = np.array([0,0,1] + R.from_euler("xyz", [np.pi,0,0], degrees=False).as_quat().tolist())
        print(pose)
        cam.set_pose(pose)
        print(cam.get_euler())
        print(R.from_quat(cam.get_quaternion()).as_euler("xyz", degrees=False))

        # img = cam.get_image()
        # print(img.shape)
        # cv2.imshow("image", img)
        # cv2.waitKey(0)
        #
        # img = cam.get_image(grey=True)
        # print(img.shape)
        # cv2.imshow("image", img)
        # cv2.waitKey(0)
        #
        # depth = cam.get_depth()
        # print(depth.shape)
        # cv2.imshow("image", depth)
        # cv2.waitKey(0)


        arm = UR5(client, obj_name="UR5")
        print(arm.get_joints_target_positions())
        # arm.set_joints_target_positions([0,0,0,0,0,0])
        # print(arm.get_joints_positions())
        arm.set_joints_target_positions([0,0,0,np.pi,0,0])
        print(arm.get_joints_positions())

        for joint in arm.joints:
            joint.control_loop_enable(False)

        # time.sleep(10)
        #
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
    # client.simxStopSimulation(client.simxDefaultPublisher())


