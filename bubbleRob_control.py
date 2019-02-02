import vrep # access all the VREP elements
import sys
import numpy as np
import time

vrep.simxFinish(-1) # just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',
                        19999,
                        True,
                        True,
                        5000,
                        5) # start a connection

if clientID!=-1:
    print ("Connected to remote API server")
else:
    print("Not connected to remote API server")
    sys.exit("Could not connect")

err_code,l_motor_handle = vrep.simxGetObjectHandle(clientID=clientID,
                                                   objectName="bubbleRob_leftMotor",
                                                   operationMode=vrep.simx_opmode_blocking)

err_code,r_motor_handle = vrep.simxGetObjectHandle(clientID=clientID,
                                                   objectName="bubbleRob_rightMotor",
                                                   operationMode=vrep.simx_opmode_blocking)

err_code,ps_handle = vrep.simxGetObjectHandle(clientID=clientID,
                                              objectName="bubbleRob_sensingNose",
                                              operationMode=vrep.simx_opmode_blocking)

err_code,detectionState,detectedPoint,\
detectedObjectHandle,\
detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID=clientID,
                                                         sensorHandle=ps_handle,
                                                         operationMode=vrep.simx_opmode_streaming)

t = time.time()

while (time.time() - t)<10:

    sensor_val = np.linalg.norm(detectedPoint)
    if sensor_val < 0.2 and sensor_val > 0.01:
        l_steer = -1 / sensor_val
    else:
        l_steer = 1.0

    r_steer = 1.0


    err_code = vrep.simxSetJointTargetVelocity(clientID=clientID,
                                               jointHandle=l_motor_handle,
                                               targetVelocity=l_steer,
                                               operationMode=vrep.simx_opmode_streaming)

    err_code = vrep.simxSetJointTargetVelocity(clientID=clientID,
                                               jointHandle=r_motor_handle,
                                               targetVelocity=r_steer,
                                               operationMode=vrep.simx_opmode_streaming)

    err_code, detectionState, detectedPoint, \
    detectedObjectHandle, \
    detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID=clientID,
                                                               sensorHandle=ps_handle,
                                                               operationMode=vrep.simx_opmode_streaming)

    print(sensor_val, detectedPoint)

vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
print("Done")