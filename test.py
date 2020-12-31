
from b0_python import b0RemoteApi
from sim_class import *
import cv2
import numpy as np
import time


if __name__ == '__main__':
    with b0RemoteApi.RemoteApiClient('b0RemoteApi_V-REP', 'b0RemoteApi') as client:

        client.simxStartSimulation(client.simxServiceCall())

        arm = UR5(client, name="UR5")
        arm.move_to(np.array([0,0,0.5,0,0,0,1]))
        time.sleep(5)
        client.simxStopSimulation(client.simxServiceCall())

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

        client.simxStopSimulation(client.simxDefaultPublisher())


