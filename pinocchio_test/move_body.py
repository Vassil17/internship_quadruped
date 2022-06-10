import pinocchio
import sys
from sys import argv
from os.path import dirname, join, abspath
from pinocchio.visualize import GepettoVisualizer
from pprint import pprint
import time
# This path refers to Pinocchio source code but you can define your own directory here.
pinocchio_model_dir = join("/opt/openrobots", "share")

# You should change here to set up your own URDF file or just pass it as an argument of this example.
model_path = join(pinocchio_model_dir,"example-robot-data/robots") if len(argv)<2 else argv[1]
mesh_dir = model_path#pinocchio_model_dir
urdf_model_path = join(model_path,"go1_description/urdf/go1.urdf")
# Load the urdf model

model, collision_model, visual_model = pinocchio.buildModelsFromUrdf(urdf_model_path, mesh_dir)
print('model name: ' + model.name)
print('model name: ' + model.name)
 
# Create data required by the algorithms
data, collision_data, visual_data  = pinocchio.createDatas(model, collision_model, visual_model)

# Sample a random configuration
q        = pinocchio.randomConfiguration(model)
print('q: %s' % q.T)
 
# Perform the forward kinematics over the kinematic tree
pinocchio.forwardKinematics(model,data,q)
 
pinocchio.updateGeometryPlacements(model, data, collision_model, collision_data)
pinocchio.updateGeometryPlacements(model, data, visual_model, visual_data)
# Print out the placement of each joint of the kinematic tree
for name, oMi in zip(model.names, data.oMi):
    print(("{:<24} : {: .2f} {: .2f} {: .2f}"
          .format( name, *oMi.translation.T.flat )))

viz = GepettoVisualizer(model, collision_model, visual_model)
 
# Initialize the viewer.
try:
    viz.initViewer()
except ImportError as err:
    print("Error while initializing the viewer. It seems you should install gepetto-viewer")
    print(err)
    sys.exit(0)
 
try:
    viz.loadViewerModel("pinocchio")
except AttributeError as err:
    print("Error while loading the viewer model. It seems you should start gepetto-viewer")
    print(err)
    sys.exit(0)
 
# Display a robot configuration.
q0 = pinocchio.neutral(model)
viz.display(q0)
#time.sleep(0.5)
#viz.display(pinocchio.randomConfiguration(model))
# for i in range(10):
#     viz.display(pinocchio.randomConfiguration(model))
#     time.sleep(0.5)

# Print out the placement of each joint of the kinematic tree
print("\nJoint placements:")
for name, oMi in zip(model.names, data.oMi):
    print(("{:<24} : {: .2f} {: .2f} {: .2f}"
          .format( name, *oMi.translation.T.flat )))
 
# Print out the placement of each collision geometry object
print("\nCollision object placements:")
for k, oMg in enumerate(collision_data.oMg):
    print(("{:d} : {: .2f} {: .2f} {: .2f}"
          .format( k, *oMg.translation.T.flat )))
 
# Print out the placement of each visual geometry object
print("\nVisual object placements:")
for k, oMg in enumerate(visual_data.oMg):
    print(("{:d} : {: .2f} {: .2f} {: .2f}"
          .format( k, *oMg.translation.T.flat )))


# pprint(dir(model.frames))
frames = model.frames.tolist()
print(frames)
print(model.frames[model.getFrameId("FR_foot_fixed")])
