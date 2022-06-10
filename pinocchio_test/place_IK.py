import pinocchio as pin
from pinocchio.utils import *
from os.path import dirname, join, abspath
from pinocchio.visualize import GepettoVisualizer
from pinocchio.robot_wrapper import RobotWrapper
import time
# Starting gepetto server and give a time
from numpy.linalg import pinv
 
 
pinocchio_model_dir = join("/opt/openrobots", "share")
model_path = join(pinocchio_model_dir, "example-robot-data/robots/ur_description")
mesh_dir = pinocchio_model_dir
urdf_filename = "ur5_gripper.urdf"
urdf_model_path = join(join(model_path, "urdf"), urdf_filename)

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

print(model.getFrameId("tool0")) # print the tool index
IDX_TOOL = 22
IDX_BASIS = 21
 
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
# Define the end-effector goal position 
Mgoal = pin.SE3(Rquat(.4, .02, -.5, .7), np.matrix([.2, -.4, .7]).T) 
robot.viewer.gui.addXYZaxis('world/framegoal', [1., 0., 0., 1.], .015, 4)
place('world/framegoal', Mgoal)


T = 1000
for i in range(T):
    pin.forwardKinematics(robot.model, robot.data, q)  # Compute joint placements
    pin.updateFramePlacements(robot.model, robot.data)      # Also compute operational frame placements
    Mtool = robot.data.oMf[IDX_TOOL]  # Get placement from world frame o to frame f oMf
    nu = pin.log(Mtool.inverse() * Mgoal).vector # The log is the differentiation in Lie group geometry I think(?)
    J = pin.computeFrameJacobian(robot.model, robot.data, q, IDX_TOOL)
    
    vq = pinv(J).dot(nu)
    dt = 0.5
    q = pin.integrate(robot.model, q, vq * dt)
    robot.display(q)
    time.sleep(0.5)
