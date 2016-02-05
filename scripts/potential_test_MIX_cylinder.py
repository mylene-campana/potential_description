#/usr/bin/env python
# Script which goes with potential_description package.
# Load planar 'robot' cylinder and concave obstacles.


from hpp.corbaserver.potential import Robot
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver import Client
import time
import sys
import matplotlib.pyplot as plt
sys.path.append('/local/mcampana/devel/hpp/src/test-hpp/script')

kRange = 5
robot = Robot ('potential')
robot.setJointBounds('base_joint_xy', [-kRange, kRange, -kRange, kRange])
ps = ProblemSolver (robot)
cl = robot.client

# q = [x, y, theta] # (z not considered since planar)
q1 = [-2, 0, 1, 0]; q2 = [2, 0, 1, 0]

ps.setInitialConfig (q1); ps.addGoalConfig (q2)
cl.obstacle.loadObstacleModel('potential_description','cylinder_obstacle','')

ps.createOrientationConstraint ("orConstraint", "base_joint_rz", "", [1,0,0,0], [0,0,1]) # OK
ps.setNumericalConstraints ("constraints", ["orConstraint"])

ps.solve ()

ps.configAtParam(0,2)
ps.configAtParam(0,6)
ps.configAtParam(0,13)
ps.getWaypoints (0)[1]

ps.optimizePath(0)
ps.pathLength(0)
ps.pathLength(1)

ps.configAtParam(1,2)
ps.configAtParam(1,6)
ps.configAtParam(1,13)
ps.getWaypoints (1)[1]

len(ps.getWaypoints (0))
cl.problem.getIterationNumber()


"""
from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (cl, r)
#r.loadObstacleModel ("potential_description","obstacles_concaves","obstacles_concaves")
r.loadObstacleModel ("potential_description","cylinder_obstacle","cylinder_obstacle")
"""

## Debug Optimization Tools ##############

import matplotlib.pyplot as plt
num_log = 11602
from parseLog import parseNodes, parsePathVector
from mutable_trajectory_plot import planarPlot, addNodePlot, addPathPlot

collConstrNodes = parseNodes (num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:228: qCollConstr = ')
collNodes = parseNodes (num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:222: qColl = ')

x1initLine = 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:184: x0+alpha*p -> x1='
x1finishLine = 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:186: finish path parsing'
x0Path = parsePathVector (num_log, x1initLine, x1finishLine, 1, 2)
x1Path = parsePathVector (num_log, x1initLine, x1finishLine, 2, 2)
x2Path = parsePathVector (num_log, x1initLine, x1finishLine, 3, 2)
x3Path = parsePathVector (num_log, x1initLine, x1finishLine, 4, 2)
x4Path = parsePathVector (num_log, x1initLine, x1finishLine, 5, 2)
x5Path = parsePathVector (num_log, x1initLine, x1finishLine, 6, 2)
x6Path = parsePathVector (num_log, x1initLine, x1finishLine, 7, 2)


plt = planarPlot (cl, 0, 1, plt) # initialize 2D plot with obstacles and path
plt = addPathPlot (cl, x0Path, '0.9', 1, plt)
plt = addPathPlot (cl, x1Path, '0.8', 1, plt)
plt = addPathPlot (cl, x2Path, '0.7', 1, plt)
plt = addPathPlot (cl, x3Path, '0.6', 1, plt)
plt = addPathPlot (cl, x4Path, '0.5', 1, plt)
plt = addPathPlot (cl, x5Path, '0.4', 1, plt)
plt = addPathPlot (cl, x6Path, '0.3', 1, plt)
plt = addNodePlot (collConstrNodes, 'bo', 'qConstr', plt)
plt = addNodePlot (collNodes, 'ro', 'qCol', plt)
plt.show() # will reset plt


#####################################################################

## DEBUG commands
robot.isConfigValid(q1)
cl.robot.distancesToCollision()
from numpy import *
argmin(cl.robot.distancesToCollision()[0])
r( cl.problem.configAtParam(0,5) )
cl.problem.optimizePath (0)
cl.problem.clearRoadmap ()
cl.problem.resetGoalConfigs ()


