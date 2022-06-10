import pinocchio as pin
from pinocchio.utils import *
from os.path import dirname, join, abspath
from pinocchio.visualize import GepettoVisualizer
from pinocchio.robot_wrapper import RobotWrapper
import time
import scipy.optimize
from scipy.optimize import fmin_bfgs, fmin_slsqp
from scipy.optimize import minimize
from numpy.linalg import pinv,solve
from matplotlib import pyplot as plt
import numpy as np
# Make sure gepetto-gui is already running

# pinocchio_model_dir = join("/opt/openrobots", "share")
# model_path = join(pinocchio_model_dir, "example-robot-data/robots")

pinocchio_model_dir = join("/home/vassil/TU_Delft/Internship/catkin_ws/src", "quadrupedal_loco/unitree_ros")
model_path = join(pinocchio_model_dir, "robots")

robotName = "go1" # change to a1 for the A1 robot

if robotName == "go1":
    mesh_dir = model_path
    urdf_filename = "go1.urdf"
    urdf_model_path = join(join(model_path, "go1_description/urdf"), urdf_filename)
elif robotName == "a1":
    mesh_dir = model_path#pinocchio_model_dir
    urdf_filename = "a1.urdf"
    urdf_model_path = join(join(model_path, "a1_description/urdf"), urdf_filename)

robot = RobotWrapper.BuildFromURDF(urdf_model_path, mesh_dir)
 
# alias
model = robot.model
data = robot.data

NQ, NV = model.nq, model.nv
 
# create valid random position
q = pin.randomConfiguration(model)
robot.setVisualizer(GepettoVisualizer())
robot.initViewer()
robot.loadViewerModel("pinocchio")
robot.display(q)

#print(model.getFrameId("FR_foot_fixed")) # print the foot index index
IDX_TOOL = model.getFrameId("FR_foot_fixed") # Get the frame of the FR foot (and its basis)
IDX_BASIS = model.getFrameId("FR_foot_fixed") - 1
 
pin.framesForwardKinematics(model, data, q)
Mtool = data.oMf[IDX_TOOL]
Mbasis = data.oMf[IDX_BASIS]

def place(name, M):
    robot.viewer.gui.applyConfiguration(name, pin.SE3ToXYZQUAT(M).tolist())
    robot.viewer.gui.refresh()
 
def Rquat(x, y, z, w):
    q = pin.Quaternion(x, y, z, w)
    q.normalize()
    return q.matrix()


def generateTrajectory(centre,length):
    a = 0.1 # horizontal axis (major)
    b = 0.05 # vertical axis (minor)
    t = np.linspace(0,2*np.pi,length)
    x = a*np.cos(t) + centre[0]
    y = b*np.sin(t) + centre[1]
    # plt.plot(x,y)
    # plt.show()
    return x,y

def positionError(q,qprev,Mgoal,robot):
    #des = pin.log(Mgoal).vector
    des = Mgoal.translation
    pin.forwardKinematics(robot.model, robot.data, q)  # Compute joint placements
    pin.updateFramePlacements(robot.model, robot.data)      # Also compute operational frame placements
    Mtool = robot.data.oMf[IDX_TOOL]  # Get placement from world frame o to frame f oMf
    cur = Mtool.translation
    #err = pin.log(Mtool.inverse() * Mgoal).vector # This is one way of computing an error in SO3
    err = np.linalg.norm(des-cur) # Calculate norm of end-effector position error
    qvel = np.abs(q-qprev)%(2*np.pi)

    qvel = np.linalg.norm(qvel) # Calculate norm of joint "velocity" (i.e. change of joint angle)

    # Define error cost Q and joint velocity cost Qj
    Q = 10
    Qj = 0.01
    cost = err*Q*err #+ qvel*Qj*qvel
    

    return cost

def constraintFun_vel(q,q1,qprev):
    qvel = np.abs(q-qprev)%(2*np.pi)/0.01
    
    # if np.any(q < -2):
    #     print('error1')
    #     print(qvel)
    #     time.sleep(5)
    # if np.any(qvel > 2):
    #     print('error')
    #     print(qvel)
    #     time.sleep(5)

    limits = np.tile(np.array([30.1]),12)


    ineq = limits - qvel

    return ineq


def constraintFun_lo(q,q1,qprev):
    
    limits = np.tile(np.array([-1.0471975512,-0.663225115758,-2.72271363311]),4)


    ineq_lo = -limits + q


    return ineq_lo

def constraintFun_up(q,q1,qprev):

    limits = np.tile(np.array([1.0471975512,2.96705972839,-0.837758040957]),4)


    ineq_up = limits - q

    return ineq_up

def fbgs_opt(q,robot,Mgoal):
    try:  # This is needed for the first iteration (when there is no qprev)
        qprev
    except:
        qprev = q

    # xopt_bfgs = fmin_bfgs(positionError, q, args = (qprev,Mgoal,robot))
    ineq_lo = {
        "type" : "ineq",
        "fun" : constraintFun_lo,
        "args" : (q,qprev)
    }
    ineq_up = {
        "type" : "ineq",
        "fun" : constraintFun_up,
        "args" : (q,qprev)
    }

    ineq_vel = {
        "type" : "ineq",
        "fun" : constraintFun_vel,
        "args" : (q,qprev)
    }
#  
    xopt_bfgs = minimize(positionError,q,args = (qprev,Mgoal,robot),method='SLSQP',constraints=[ineq_lo,ineq_up,ineq_vel])
    qprev = q

    return xopt_bfgs


# Define the end-effector goal position 
origin = pin.SE3(np.eye(3), np.matrix([0, 0, 0]).T)  # Display the origin coordinate axes
Mgoal = pin.SE3(np.eye(3), np.matrix([0.2, -0.12, -0.2]).T) # Display the desired goal axis
robot.viewer.gui.addXYZaxis('world/framegoal', [3., 3.,1., 3.], 0.015, 2) # colour, size, length of axis
robot.viewer.gui.addXYZaxis('world/origin', [0., 0., 0., 0.], 0.015, 4)
robot.viewer.gui.refresh()
place('world/framegoal', Mgoal)
place('world/origin',origin)


T = 5
dt = 0.005

centre = np.array([0.2,-0.35])
x,y = generateTrajectory(centre,T/dt)
#x = np.linspace(0.1,0.3,100)
robot.viewer.gui.removeLightSources = False

qStore = []
q = pin.neutral(model)
# video_dir = "/home/vassil/TU_Delft/Internship/catkin_ws/src/pinocchio_test/video_IK/frame"
# robot.viewer.gui.startCapture('python-pinocchio',video_dir,"jpg") # 1st arg is window name (can be seen in gepetto-gui)
for iter in range(5):
    for i in range(int(T/dt)):
        Mgoal = pin.SE3(np.eye(3), np.matrix([x[int(T/dt)-1-i], -0.12, y[int(T/dt)-1-i]]).T) # Desired goal
        place('world/framegoal', Mgoal)
        # T = 30
        #for i in range(T):
            # pin.forwardKinematics(robot.model, robot.data, q)  # Compute joint placements
            # pin.updateFramePlacements(robot.model, robot.data)      # Also compute operational frame placements
            # Mtool = robot.data.oMf[IDX_TOOL]  # Get placement from world frame o to frame f oMf
           
            # print(nu)
            # J = pin.computeFrameJacobian(robot.model, robot.data, q, IDX_TOOL)
            
            # vq = pinv(J).dot(nu)
            # damp = 1e-12
            # #vq =  J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), nu))
            # dt = 0.5
            # q = pin.integrate(robot.model, q, vq * dt)
        q = fbgs_opt(q,robot,Mgoal)
        q = q.x
        print(q[3:6])
        qStore.append(q)
        robot.display(q)
        time.sleep(0.0000001)
# robot.viewer.gui.stopCapture('python-pinocchio')
plt.plot(np.array(qStore))
plt.show()