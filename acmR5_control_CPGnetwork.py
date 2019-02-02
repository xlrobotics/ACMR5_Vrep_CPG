import vrep # access all the VREP elements
import sys
import numpy as np
import time
import math
import scipy.io
import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt

mat = scipy.io.loadmat('matlabtrajdata.mat')

class Network:
    def __init__(self):
        # TODO: network from csv
        self.we = {}
        for i in range(1,9):
            self.we[i] = {}

        self.we[1][2] = 2.5601
        self.we[2][3] = 5.2436
        self.we[3][4] = -1.8004
        self.we[4][5] = 0.134
        self.we[5][6] = 1.8298
        self.we[6][7] = 4.2504
        self.we[7][8] = 7.5926
        self.we[8][1] = 5.2436

        self.wf = {}
        for i in range(1, 9):
            self.wf[i] = {}

        self.wf[1][2] = 3.2163
        self.wf[2][3] = 3.7014
        self.wf[3][4] = -1.2252
        self.wf[4][5] = -4.8968
        self.wf[5][6] = 0.9931
        self.wf[6][7] = 1.1212
        self.wf[7][8] = 1.442
        self.wf[8][1] = -2.2252

class Matsuoka:

    def __init__(self, id,
                 A=1.0,
                 w12 =2.0399, w21=2.0399,
                 tr = 1.0607, ta=0.2773,
                 beta=10.0, #10
                 weij=1.0, wfij=1.0,
                 q=1.0,
                 dt = 0.05,
                 s1=1.0, s2=1.0,
                 kp = -.01,
                 fb1 = 0, fb2 = 0,
                 fk=1):

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
        self.wfij = wfij
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

    def _step(self, fb = 0, y1j = 1, y2j = 1):

        # computing feedback terms from the sensor
        self.fb1 = self.kp * fb
        self.fb2 = -self.kp * fb

        # extensor
        du1 = 1/self.tau_r * (-self.u1 - self.beta*self.v1 - self.w12*self.y2 - self.weij*y1j + self.s1 + self.fb1)
        dv1 = 1/self.tau_a * (-self.v1 + self.y1**self.q)

        # flexor
        du2 = 1/self.tau_r * (-self.u2 - self.beta*self.v2 - self.w21*self.y1 - self.wfij*y2j + self.s2 + self.fb2)
        dv2 = 1/self.tau_a * (-self.v2 + self.y2**self.q)

        # Euler forward? RK4?
        self.u1 += du1 * self.dt
        self.v1 += dv1 * self.dt
        self.u2 += du2 * self.dt
        self.v2 += dv2 * self.dt

        self.y1 = max(0, self.u1)
        self.y2 = max(0, self.u2)

        self.theta.append(self.A*(self.y1 - self.y2))

S = {}
S[1] = -4.2782
S[2] = 0.9728
S[3] = 2.4138
S[4] = 2.645
S[5] = -5.7754
S[6] = -7.3844
S[7] = -1.4242
S[8] = 7.4953

def get_values(params, i):
    q = 1

    Tr = params[0]
    Ta = params[1]
    b = params[2]
    wef = params[3]
    S = params[4]

    traj = list(mat['theta'+str(i)].T[0])

    return traj


# Close all opened connections
vrep.simxFinish(-1)

# get the client id of the vrep sim
clientID=vrep.simxStart('127.0.0.1',
                        19999,
                        True,
                        True,
                        5000,
                        5) # start a connection

# check if the connection is successfully achieved
if clientID!=-1:
    print ("Connected to remote API server")
else:
    print("Not connected to remote API server")
    sys.exit("Could not connect")

# getting the output from the CPG based controller
theta1 = get_values([0.1642, 0.6418, 8.2401,  5.3571, 0.8539], 1)
theta2 = get_values([0.1276, 0.6971, 7.5937,  5.4866, 0.7666], 2)
theta3 = get_values([0.1746, 0.7378, 9.5590,  5.1743, 0.7863], 3)
theta4 = get_values([0.2367, 0.5084, 8.9907,  6.0466, 0.9515], 4)
theta5 = get_values([0.2674, 0.6203, 14.9224, 9.2390, 1.5598], 5)
theta6 = get_values([0.3250, 0.6065, 16.0000, 9.0270, 1.5434], 6)
theta7 = get_values([0.3527, 0.5620, 15.2292, 8.7594, 1.4544], 7)
theta8 = get_values([0.2862, 0.7722, 18.2860, 9.8201, 1.2749], 8)

# test_theta1 = []

net = Network()

J = {}
# J[1] = Matsuoka(id = 1, tr=0.1642, ta=0.6418, beta=8.2401,  w12=5.3571, w21=5.3571, A=0.8539)
# J[2] = Matsuoka(id = 2, tr=0.1276, ta=0.6971, beta=7.5937,  w12=5.4866, w21=5.4866, A=0.7666)
# J[3] = Matsuoka(id = 3, tr=0.1746, ta=0.7378, beta=9.5590,  w12=5.1743, w21=5.1743, A=0.7863)
# J[4] = Matsuoka(id = 4, tr=0.2367, ta=0.5084, beta=8.9907,  w12=6.0466, w21=6.0466, A=0.9515)
# J[5] = Matsuoka(id = 5, tr=0.2674, ta=0.6203, beta=14.9224, w12=9.2390, w21=9.2390, A=1.5598)
# J[6] = Matsuoka(id = 6, tr=0.3250, ta=0.6065, beta=16.000,  w12=9.0270, w21=9.0270, A=1.5434)
# J[7] = Matsuoka(id = 7, tr=0.3527, ta=0.5620, beta=15.2292, w12=8.7594, w21=8.7594, A=1.4544)
# J[8] = Matsuoka(id = 8, tr=0.2862, ta=0.7722, beta=18.2860, w12=9.8201, w21=9.8201, A=1.2749)
for i in range(1, 9):
    k = i-1
    if k == 0:
        k = 8
    J[i] = Matsuoka(id = i, A=S[i], weij=net.we[k][i], wfij=net.wf[k][i])

# J[1] = Matsuoka(id = 1, tr=0.1642, ta=0.6418, beta=8.2401,  w12=5.3571, w21=5.3571, A=S[1],  weij=net.we[8][1], wfij=net.wf[8][1])
# J[2] = Matsuoka(id = 2, tr=0.1276, ta=0.6971, beta=7.5937,  w12=5.4866, w21=5.4866, A=S[2],  weij=net.we[1][2], wfij=net.wf[1][2])
# J[3] = Matsuoka(id = 3, tr=0.1746, ta=0.7378, beta=9.5590,  w12=5.1743, w21=5.1743, A=S[3],  weij=net.we[2][3], wfij=net.wf[2][3])
# J[4] = Matsuoka(id = 4, tr=0.2367, ta=0.5084, beta=8.9907,  w12=6.0466, w21=6.0466, A=S[4],  weij=net.we[3][4], wfij=net.wf[3][4])
# J[5] = Matsuoka(id = 5, tr=0.2674, ta=0.6203, beta=14.9224, w12=9.2390, w21=9.2390, A=S[5],  weij=net.we[4][5], wfij=net.wf[4][5])
# J[6] = Matsuoka(id = 6, tr=0.3250, ta=0.6065, beta=16.000,  w12=9.0270, w21=9.0270, A=S[6],  weij=net.we[5][6], wfij=net.wf[5][6])
# J[7] = Matsuoka(id = 7, tr=0.3527, ta=0.5620, beta=15.2292, w12=8.7594, w21=8.7594, A=S[7],  weij=net.we[6][7], wfij=net.wf[6][7])
# J[8] = Matsuoka(id = 8, tr=0.2862, ta=0.7722, beta=18.2860, w12=9.8201, w21=9.8201, A=S[8],  weij=net.we[7][8], wfij=net.wf[7][8])

# for i in range(951):
#     J1._step()


# Read ACMR vJoints
N = 6  # 8 Joints
strV = 'ACMR_vJoint#' # Joint Names
jointV = strV
jointsV = {}
# Read Object Handle for all joints
returnCode,jointsV[1] = vrep.simxGetObjectHandle(clientID,jointV,vrep.simx_opmode_blocking)

for i in range(N+1):
    jointV = strV + str(i)
    returnCode,jointsV[i+2] = vrep.simxGetObjectHandle(clientID,jointV,vrep.simx_opmode_blocking)


# pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8 = [],[],[],[],[],[],[],[]
pos = {}
for i in range(1, 9):
    pos[i] = []
# Setting CPG oscillator output via PD controller(tuned and set in Vrep env) to the joints
for j in range(500):
    t0 = time.time()
    # vrep.simxGetJointPosition()
    for i in range(1, 9):
        res, pos_temp = vrep.simxGetJointPosition(clientID, jointsV[i], vrep.simx_opmode_streaming)
        # pos = pos * 180 / math.pi
        pos[i].append(pos_temp)
    # res, pos = vrep.simxGetJointPosition(clientID, jointsV[2], vrep.simx_opmode_streaming)
    # # pos = pos * 180 / math.pi
    # pos2.append(pos)
    # res, pos = vrep.simxGetJointPosition(clientID, jointsV[3], vrep.simx_opmode_streaming)
    # # pos = pos * 180 / math.pi
    # pos3.append(pos)
    # res, pos = vrep.simxGetJointPosition(clientID, jointsV[4], vrep.simx_opmode_streaming)
    # # pos = pos * 180 / math.pi
    # pos4.append(pos)
    # res, pos = vrep.simxGetJointPosition(clientID, jointsV[5], vrep.simx_opmode_streaming)
    # # pos = pos * 180 / math.pi
    # pos5.append(pos)
    # res, pos = vrep.simxGetJointPosition(clientID, jointsV[6], vrep.simx_opmode_streaming)
    # # pos = pos * 180 / math.pi
    # pos6.append(pos)
    # res, pos = vrep.simxGetJointPosition(clientID, jointsV[7], vrep.simx_opmode_streaming)
    # # pos = pos * 180 / math.pi
    # pos7.append(pos)
    # res, pos = vrep.simxGetJointPosition(clientID, jointsV[8], vrep.simx_opmode_streaming)
    # # pos = pos * 180 / math.pi
    # pos8.append(pos)

    for i in range(1,9):
        k = i - 1 if i > 1 else 8
        J[i]._step(fb=pos[i][-1], y1j=J[k].y1, y2j=J[k].y2)
        # J[i]._step(y1j=J[k].y1, y2j=J[k].y2)
        # J[i]._step(fb=pos[i][-1])
    # returnCode = vrep.simxSetJointTargetPosition(clientID,jointsV[1],theta1[j],vrep.simx_opmode_streaming)
    # returnCode = vrep.simxSetJointTargetPosition(clientID,jointsV[2],theta2[j],vrep.simx_opmode_streaming)
    # returnCode = vrep.simxSetJointTargetPosition(clientID,jointsV[3],theta3[j],vrep.simx_opmode_streaming)
    # returnCode = vrep.simxSetJointTargetPosition(clientID,jointsV[4],theta4[j],vrep.simx_opmode_streaming)
    # returnCode = vrep.simxSetJointTargetPosition(clientID,jointsV[5],theta5[j],vrep.simx_opmode_streaming)
    # returnCode = vrep.simxSetJointTargetPosition(clientID,jointsV[6],theta6[j],vrep.simx_opmode_streaming)
    # returnCode = vrep.simxSetJointTargetPosition(clientID,jointsV[7],theta7[j],vrep.simx_opmode_streaming)
    # returnCode = vrep.simxSetJointTargetPosition(clientID,jointsV[8],theta8[j],vrep.simx_opmode_streaming)
    for i in range(1,9):
        returnCode = vrep.simxSetJointTargetPosition(clientID, jointsV[i], J[i].theta[-1], vrep.simx_opmode_streaming)

    t_ = time.time()-t0
    print("time cost:",t_)
    # pause .01 sec
    time.sleep(.05 - t_)

# Delete the rempote API
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
print("Done")


plt.plot(J[1].theta, color='r')
plt.plot(pos[1], color='b')
# plt.plot(J1.theta, color='g')

plt.show()
