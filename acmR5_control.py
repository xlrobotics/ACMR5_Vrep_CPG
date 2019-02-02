import vrep # access all the VREP elements
import sys
import numpy as np
import time
import math
import scipy.io
import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt
from acmR5_fitness import fitness

# mat = scipy.io.loadmat('matlabtrajdata.mat')

class Network:
    def __init__(self, n):
        # TODO: network from csv
        self.w = {}

class Matsuoka:

    def __init__(self, id, tr, ta, beta, w12, w21, A, weij=1, q=1, dt = 0.05, s1=1, s2=1, kp = -0.35, fb1 = 0, fb2 = 0, fk=1):
        # all zero will be equilibrium point, need slightly different initial value to trigger the oscillation
        # inner states of the oscillator
        self.u1=0.00000001
        self.u2=0
        self.v1=0
        self.v2=0

        self.y1=0
        self.y2=0

        self.id=id
        self.fk = fk  # tuning frequency for tau_r and tau_a
        self.tau_r = tr * self.fk
        self.tau_a = ta * self.fk
        self.s1 = s1
        self.s2 = s2
        self.fb1 = fb1
        self.fb2 = fb2

        self.beta = beta
        self.w12 = w12
        self.w21 = w21
        self.weij = weij
        self.q = q
        self.dt = dt
        self.A = A

        self.kp = kp # feedback coefficient for position

        # joint position input command memory, extract current theta command from self.theta[-1]
        self.theta = [0]

        # du1 = 1 / self.tau_r * (-self.u1 - self.beta * self.v1 - self.w12 * self.y2 + self.weij + self.s1)
        # dv1 = 1 / self.tau_a * (-self.v1 + self.y1 ** self.q)
        # self.u1 += du1 * self.dt
        # self.v1 += dv1 * self.dt
        # self.y1 = max(0, self.u1)

    def _step(self, fb = 0):

        # computing feedback terms from the sensor
        self.fb1 = self.kp * fb
        self.fb2 = -self.kp * fb

        du1 = 1/self.tau_r * (-self.u1 - self.beta*self.v1 - self.w12*self.y2 + self.weij + self.s1 + self.fb1)
        dv1 = 1/self.tau_a * (-self.v1 + self.y1**self.q)

        du2 = 1/self.tau_r * (-self.u2 - self.beta*self.v2 - self.w21*self.y1 + self.weij + self.s2 + self.fb2)
        dv2 = 1/self.tau_a * (-self.v2 + self.y2**self.q)

        # RK2
        # k1_u1 = du1 * self.dt
        # k1_v1 = dv1 * self.dt
        # k1_u2 = du2 * self.dt
        # k1_v2 = dv2 * self.dt
        #
        # k2_u1 = 1/self.tau_r * (-(self.u1+k1_u1/2) - self.beta*(self.v1+k1_v1/2) - self.w12*self.y2 + self.weij + self.s1 + self.fb1)
        # k2_v1 = 1/self.tau_a * (-(self.v1+k1_v1/2) + self.y1**self.q)
        # k2_u2 = 1/self.tau_r * (-(self.u2+k1_u2/2) - self.beta*(self.v2+k1_v2/2) - self.w21*self.y1 + self.weij + self.s2 + self.fb2)
        # k2_v2 = 1/self.tau_a * (-(self.v2+k1_v2/2) + self.y2**self.q)
        #
        # self.u1 += k2_u1 * self.dt
        # self.v1 += k2_v1 * self.dt
        # self.u2 += k2_u2 * self.dt
        # self.v2 += k2_v2 * self.dt

        # Euler
        self.u1 += du1 * self.dt
        self.v1 += dv1 * self.dt
        self.u2 += du2 * self.dt
        self.v2 += dv2 * self.dt

        self.y1 = max(0, self.u1)
        self.y2 = max(0, self.u2)

        self.theta.append(self.A*(self.y1 - self.y2))


# def get_values(params, i):
#     q = 1
#
#     Tr = params[0]
#     Ta = params[1]
#     b = params[2]
#     wef = params[3]
#     S = params[4]
#
#     traj = list(mat['theta'+str(i)].T[0])

    # return traj

class robotSim:
    def __init__(self, robotName):
        id = -1
        count = 0
        while id == -1 and count < 10:
            id = vrep.simxStart('127.0.0.1',
                                  19999,
                                  True,
                                  True,
                                  5000,
                                  5)  # start a connection
        self.clientID = id
        if self.clientID == -1:
            print("Not connected to remote API server")
            sys.exit("Could not connect")
        # check if the connection is successfully achieved
        # if self.clientID!=-1:
        #     print ("Connected to remote API server")
        # else:
        #     print("Not connected to remote API server")
        #     sys.exit("Could not connect")

        returnCode, self.robotObj = vrep.simxGetObjectHandle(self.clientID, robotName, vrep.simx_opmode_blocking)
        self.resetFlag = None

        self.robotState = None

    def _reset(self):
        self.resetFlag = vrep.simxRemoveModel(clientID=self.clientID,
                                         objectHandle=self.robotObj,
                                         operationMode=vrep.simx_opmode_streaming) #_wait
        # print('Close Flag:', self.resetFlag)

        time.sleep(2.0)
        self.resetFlag, self.robotObj = vrep.simxLoadModel(clientID=self.clientID,
                                                     modelPathAndName='/Users/xuanliu/Documents/V-REP_PRO_EDU_V3_4_0_Mac/models/robots/mobile/ACM-R5-disabled.ttm',
                                                     operationMode=vrep.simx_opmode_oneshot_wait, # _wait
                                                     options=0)
        # print('Load Flag:', self.resetFlag)

    def update_state(self):
        self.robotState = vrep.simxGetObjectPosition(clientID=self.clientID,
                                                     objectHandle=self.robotObj,
                                                     relativeToObjectHandle=-1,
                                                     operationMode=vrep.simx_opmode_streaming)
        print('current robot position:')
        print(self.robotState)

    def run(self):
        return


def experiment(params):
    vrep.simxFinish(-1)

    robot = robotSim("ACMR")

    kp = params[0]
    w = {}
    for i in range(1, 9):
        w[i] = params[i]
    a = params[9]
    tr = params[10]
    ta = params[11]
    beta = params[12]

    print(params)
    J = {}
    for i in range(1, 9):
        J[i] = Matsuoka(id = i, tr = tr, ta = ta, beta = beta, w12 = w[i], w21 = w[i], A = a, kp = kp)
    #
    # pos = {}
    # for i in range(1, 9):
    #     pos[i] = []
    # Setting CPG oscillator output via PD controller(tuned and set in Vrep env) to the joints

    # t_init = time.time()

    # Read ACMR vJoints
    N = 6  # 8 Joints
    strV = 'ACMR_vJoint#'  # Joint Names
    jointV = strV
    jointsV = {}
    # Read Object Handle for all joints
    returnCode, jointsV[1] = vrep.simxGetObjectHandle(robot.clientID, jointV, vrep.simx_opmode_blocking)

    for i in range(N + 1):
        jointV = strV + str(i)
        returnCode, jointsV[i + 2] = vrep.simxGetObjectHandle(robot.clientID, jointV, vrep.simx_opmode_blocking)

        # vrep.simxStartSimulation(robot.clientID, vrep.simx_opmode_oneshot)
    # robot.update_state()
    # sx = robot.robotState[1][0]
    # sy = robot.robotState[1][1]

    # sx, sy = 0.6536598205566406, -1.9246925115585327
    robot.update_state()
    sx, sy = 0.0, 0.0
    for j in range(300):
        # if j == 2:
        #     robot.update_state()
        #     sx = robot.robotState[1][0]
        #     sy = robot.robotState[1][1]

        t0 = time.time()

            # vrep.simxGetJointPosition()
        # for i in range(1, 9):
        #     res, pos_temp = vrep.simxGetJointPosition(robot.clientID, jointsV[i], vrep.simx_opmode_streaming)
        #         # pos = pos * 180 / math.pi
        #     pos[i].append(pos_temp)

        for i in range(1, 9):
            J[i]._step()

        for i in range(1, 9):
            returnCode = vrep.simxSetJointTargetPosition(robot.clientID, jointsV[i], J[i].theta[-1],
                                                             vrep.simx_opmode_streaming)

        t_ = time.time() - t0
            # pause .01 sec
        time.sleep(.05 - t_)

    robot.update_state()
    ex, ey = robot.robotState[1][0], robot.robotState[1][1]

    robot._reset()

    time.sleep(.05)
    # print("Done")

    # print('cost ', time.time() - t_init, 'seconds.')

    # Shut down the rempote API
    # vrep.simxStopSimulation(robot.clientID, vrep.simx_opmode_oneshot)
    # print('Simulation stopped.')
    value = fitness(start_x=sx, start_y=sy, end_x=ex, end_y=ey, up_time=200*0.05)
    print('current score:', value)

    return value


# Close all opened connections
# vrep.simxFinish(-1)
#
# robot = robotSim("ACMR")
#
# # test_theta1 = []
# J = {}
# J[1] = Matsuoka(id = 1, tr=0.1642, ta=0.6418, beta=8.2401,  w12=5.3571, w21=5.3571, A=0.8539)
# J[2] = Matsuoka(id = 2, tr=0.1276, ta=0.6971, beta=7.5937,  w12=5.4866, w21=5.4866, A=0.7666)
# J[3] = Matsuoka(id = 3, tr=0.1746, ta=0.7378, beta=9.5590,  w12=5.1743, w21=5.1743, A=0.7863)
# J[4] = Matsuoka(id = 4, tr=0.2367, ta=0.5084, beta=8.9907,  w12=6.0466, w21=6.0466, A=0.9515)
# J[5] = Matsuoka(id = 5, tr=0.2674, ta=0.6203, beta=14.9224, w12=9.2390, w21=9.2390, A=1.5598)
# J[6] = Matsuoka(id = 6, tr=0.3250, ta=0.6065, beta=16.000,  w12=9.0270, w21=9.0270, A=1.5434)
# J[7] = Matsuoka(id = 7, tr=0.3527, ta=0.5620, beta=15.2292, w12=8.7594, w21=8.7594, A=1.4544)
# J[8] = Matsuoka(id = 8, tr=0.2862, ta=0.7722, beta=18.2860, w12=9.8201, w21=9.8201, A=1.2749)

# pos = {}
# for i in range(1, 9):
#     pos[i] = []
# Setting CPG oscillator output via PD controller(tuned and set in Vrep env) to the joints
if __name__ == '__main__':

    count = 10

    t_init = time.time()

    while count>0:
        count -= 1

        vrep.simxFinish(-1)

        robot = robotSim("ACMR")

        # test_theta1 = []
        J = {}
        J[1] = Matsuoka(id=1, tr=0.1642, ta=0.6418, beta=8.2401, w12=5.3571, w21=5.3571, A=0.8539)
        J[2] = Matsuoka(id=2, tr=0.1276, ta=0.6971, beta=7.5937, w12=5.4866, w21=5.4866, A=0.7666)
        J[3] = Matsuoka(id=3, tr=0.1746, ta=0.7378, beta=9.5590, w12=5.1743, w21=5.1743, A=0.7863)
        J[4] = Matsuoka(id=4, tr=0.2367, ta=0.5084, beta=8.9907, w12=6.0466, w21=6.0466, A=0.9515)
        J[5] = Matsuoka(id=5, tr=0.2674, ta=0.6203, beta=14.9224, w12=9.2390, w21=9.2390, A=1.5598)
        J[6] = Matsuoka(id=6, tr=0.3250, ta=0.6065, beta=16.000, w12=9.0270, w21=9.0270, A=1.5434)
        J[7] = Matsuoka(id=7, tr=0.3527, ta=0.5620, beta=15.2292, w12=8.7594, w21=8.7594, A=1.4544)
        J[8] = Matsuoka(id=8, tr=0.2862, ta=0.7722, beta=18.2860, w12=9.8201, w21=9.8201, A=1.2749)

        pos = {}
        for i in range(1, 9):
            pos[i] = []


        # Read ACMR vJoints
        N = 6  # 8 Joints
        strV = 'ACMR_vJoint#'  # Joint Names
        jointV = strV
        jointsV = {}
        # Read Object Handle for all joints
        returnCode, jointsV[1] = vrep.simxGetObjectHandle(robot.clientID, jointV, vrep.simx_opmode_blocking)

        for i in range(N + 1):
            jointV = strV + str(i)
            returnCode, jointsV[i + 2] = vrep.simxGetObjectHandle(robot.clientID, jointV, vrep.simx_opmode_blocking)

        # vrep.simxStartSimulation(robot.clientID, vrep.simx_opmode_oneshot)
        robot.update_state()

        for j in range(200):
            t0 = time.time()

            # vrep.simxGetJointPosition()
            for i in range(1, 9):
                res, pos_temp = vrep.simxGetJointPosition(robot.clientID, jointsV[i], vrep.simx_opmode_streaming)
                # pos = pos * 180 / math.pi
                pos[i].append(pos_temp)


            for i in range(1,9):
                J[i]._step(pos[i][-1])

            for i in range(1,9):
                returnCode = vrep.simxSetJointTargetPosition(robot.clientID, jointsV[i], J[i].theta[-1], vrep.simx_opmode_streaming)

            t_ = time.time() - t0
            # pause .01 sec
            time.sleep(.05 - t_)

        robot.update_state()

        robot._reset()

        time.sleep(.05)
        print("Done", count)

    print(count,' runs cost ', time.time()-t_init, 'seconds.')

    # Shut down the rempote API
    vrep.simxStopSimulation(robot.clientID, vrep.simx_opmode_oneshot)
    print('Simulation stopped.')

    # plt.plot(J[1].theta, color='r')
    # plt.plot(pos[1], color='b')
    # # plt.plot(J1.theta, color='g')
    #
    # plt.show()
