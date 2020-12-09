import sys
print(sys.path)
sys.path.append("C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\remoteApiBindings\python\python")
sys.path.append("C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\remoteApiBindings\lib\lib\Windows")
import sim
import time

if __name__ == '__main__':
    sim.simxFinish(-1) # just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim
    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
    if clientID != -1:
        print('Connected to remote API server')

        # Now try to retrieve data in a blocking fashion (i.e. a service call):
        res, objs = sim.simxGetObjects(clientID, sim.sim_handle_all, sim.simx_opmode_blocking)
        if res == sim.simx_return_ok:
            print('Number of objects in the scene: ', len(objs))
        else:
            print('Remote API function call returned with error code: ', res)

        time.sleep(2)
        sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)

        # Now retrieve streaming data (i.e. in a non-blocking fashion):
        startTime = time.time()
        sim.simxGetIntegerParameter(clientID, sim.sim_intparam_mouse_x,
                                    sim.simx_opmode_streaming)  # Initialize streaming
        while time.time() - startTime < 5:
            returnCode, data = sim.simxGetIntegerParameter(clientID, sim.sim_intparam_mouse_x,
                                                           sim.simx_opmode_buffer)  # Try to retrieve the streamed data
            if returnCode == sim.simx_return_ok:  # After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
                print('Mouse position x: ',
                      data)  # Mouse position x is actualized when the cursor is over CoppeliaSim's window
            time.sleep(0.005)

        # Now send some data to CoppeliaSim in a non-blocking fashion:
        sim.simxAddStatusbarMessage(clientID, 'Hello CoppeliaSim!', sim.simx_opmode_oneshot)

        # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        sim.simxGetPingTime(clientID)

        # Now close the connection to CoppeliaSim:
        sim.simxFinish(clientID)
    else:
        print('Failed connecting to remote API server')
    print('Program ended')


