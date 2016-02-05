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
q1 = [-4, 4, 1, 0]; q2 = [4, -4, 1, 0] # obstS 1
#q1 = [-4, 4, 0]; q2 = [4, -4, 0] # obstS 1

ps.setInitialConfig (q1); ps.addGoalConfig (q2)
cl.obstacle.loadObstacleModel('potential_description','obstacles_concaves','obstacles_concaves')

#ps.createOrientationConstraint ("orConstraint", "base_joint_rz", "", [1,0,0,0], [0,0,1])
#ps.setNumericalConstraints ("constraints", ["orConstraint"])

ps.solve ()
ps.pathLength(0)

ps.addPathOptimizer("GradientBased")
ps.optimizePath (0)
ps.numberPaths()
ps.pathLength(ps.numberPaths()-1)

pp(ps.numberPaths()-1)

ps.clearPathOptimizers()
ps.addPathOptimizer('RandomShortcut')
ps.optimizePath (0)
ps.pathLength(1)


len(ps.getWaypoints (0))

ps.removeObstacleFromJoint ('obstacles_concaves', 'base_joint_xy',True,True)
ps.removeObstacleFromJoint ('base_joint_rz', 'obstacles_concaves',True,True)

"""
from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (cl, r)
r.loadObstacleModel ("potential_description","obstacles_concaves","obstacles_concaves")
"""

#q1 = [2.4, -4.6, 1.0, 0.0]; q2 = [-0.4, 4.6, 1.0, 0.0] # obstS 2
#cl.obstacle.loadObstacleModel('potential_description','cylinder_obstacle','')

## Debug Optimization Tools ##############

import matplotlib.pyplot as plt
num_log = 12903
from parseLog import parseNodes, parsePathVector
from mutable_trajectory_plot import planarPlot, addNodePlot, addPathPlot

collConstrNodes = parseNodes (num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:318: qCollConstr = ')
collNodes = parseNodes (num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:308: qColl = ')

x1initLine = 'INFO:/local/mcampana/devel/hpp/src/hpp-core/include/hpp/core/path-optimization/gradient-based.hh:86: x0+alpha*p -> x1='
x1finishLine = 'INFO:/local/mcampana/devel/hpp/src/hpp-core/include/hpp/core/path-optimization/gradient-based.hh:89: finish path parsing'
skipLines = 1 # skip useless lines while parsing path, usually skip rotation parts
x0Path = parsePathVector (num_log, x1initLine, x1finishLine, 1, skipLines)
x1Path = parsePathVector (num_log, x1initLine, x1finishLine, 2, skipLines)
x2Path = parsePathVector (num_log, x1initLine, x1finishLine, 3, skipLines)
x3Path = parsePathVector (num_log, x1initLine, x1finishLine, 4, skipLines)
x4Path = parsePathVector (num_log, x1initLine, x1finishLine, 5, skipLines)
x5Path = parsePathVector (num_log, x1initLine, x1finishLine, 6, skipLines)
x6Path = parsePathVector (num_log, x1initLine, x1finishLine, 7, skipLines)


plt = planarPlot (cl, 0, ps.numberPaths()-1, plt) # initialize 2D plot with obstacles and path
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

"""
plt = planarPlot (cl, 0, 1, plt) # initialize 2D plot with obstacles and path
plt = addNodePlot (collConstrNodes, 'bo', 'qConstr', plt)
plt = addNodePlot (collNodes, 'ro', 'qCol', plt)
plt = addPathPlot (cl, x0Path, 'm', 1, plt)
plt = addPathPlot (cl, x1Path, 'g', 1, plt)
plt = addPathPlot (cl, x2Path, 'b', 1, plt)
plt = addPathPlot (cl, x3Path, 'y', 1, plt)
plt = addPathPlot (cl, x4Path, 'c', 1, plt)
plt = addPathPlot (cl, x5Path, '0.8', 1, plt)
plt = addPathPlot (cl, x6Path, '0.5', 1, plt)
plt.show() # will reset plt
"""
#####################################################################

from trajectory_plot import planarConcObstaclesPlot # case multiple concaves obstacles
planarConcObstaclesPlot(cl, 1, '', 0) # don't plot "equirepartis" nodes


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


# Plot Trajectory, x, y, dist_obst #
from trajectory_plot import planarConcObstaclesPlot # case multiple concaves obstacles
planarConcObstaclesPlot(cl, 0, '', 0)

# Gradients arrows Plot on 2D plan graph (with trajectory) #
from parseLog import parseGrad, parseConfig
num_log = 10435 # TO_FILL, and UPDATE following line numbers
q_list = parseConfig(num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/potential-method.cc:381: q(x,y): ')
grad_list = parseGrad(num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/potential-method.cc:383: grad(x,y): ')
q_rand_list = parseConfig(num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/diffusing-planner.cc:121: q_rand = ') # diffusingPlanner
#q_list = parseConfig(num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/superposed-potential-method.cc:???: q(x,y): ')
#grad_list = parseGrad(num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/superposed-potential-method.cc:???: grad(x,y): ')
#q_rand_list = parseConfig(num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/superposed-planner.cc:105: q_rand = ') # superposedPlanner

from trajectory_plot import gradArrowsConcPlot # concave
gradArrowsConcPlot(cl, q_list, grad_list, q_rand_list, 0)

from trajectory_plot import gradArrowsPlot # cylinder
gradArrowsPlot(cl, q_list, grad_list, q_rand_list)

# Visibility-PRM verification :
num_log = 7770
from parseLog import parseNodes
guardNodes = parseNodes (num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/visibility-prm-planner.cc:176: q is a guard node: ')
from trajectory_plot import planarConcObstaclesSpecNodesPlot
planarConcObstaclesSpecNodesPlot(cl, 0, '', 0, guardNodes) # guards in blue

