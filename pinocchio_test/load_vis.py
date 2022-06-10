from __future__ import print_function 
import numpy as np
from numpy.linalg import norm, solve
import pinocchio
import sys
import os
from os.path import dirname, join, abspath


# Load the URDF model.
# Conversion with str seems to be necessary when executing this file with ipython
pinocchio_model_dir = join(dirname(dirname(str(abspath(__file__)))),"unitree_ros/robots/go1_description")

model_path = join(pinocchio_model_dir,"xacro")

urdf_filename = "go1.champ.urdf"
urdf_model_path = join(model_path,urdf_filename)
 
urdf_model_path = "/home/vassil/TU_Delft/Internship/catkin_ws/src/unitree_ros/robots/go1_description/xacro/go1.champ.urdf"

model = pinocchio.buildModelFromUrdf(urdf_model_path)

data  = model.createData()

JOINT_ID = 6
oMdes = pinocchio.SE3(np.eye(3), np.array([.1, 0., 0.]))
  
q      = pinocchio.neutral(model)
eps    = 1e-4
IT_MAX = 1000
DT     = 1e-1
damp   = 1e-12
  
i=0
while True:
    pinocchio.forwardKinematics(model,data,q)
    dMi = oMdes.actInv(data.oMi[JOINT_ID])
    err = pinocchio.log(dMi).vector
    if norm(err) < eps:
        success = True
        break
    if i >= IT_MAX:
        success = False
        break
    J = pinocchio.computeJointJacobian(model,data,q,JOINT_ID)
    v = - J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
    q = pinocchio.integrate(model,q,v*DT)
    if not i % 10:
        print('%d: error = %s' % (i, err.T))
    i += 1
  
if success:
    print("Convergence achieved!")
else:
    print("\nWarning: the iterative algorithm has not reached convergence to the desired precision")
  
print('\nresult: %s' % q.flatten().tolist())
print('\nfinal error: %s' % err.T)