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

ps.selectPathPlanner ("VisibilityPrmPlanner")
ps.selectPathValidation ("Dichotomy", 0.)

ps.solve ()
ps.pathLength(0)
len(ps.getWaypoints (0))

import numpy as np

ps.addPathOptimizer("Prune")
ps.optimizePath (0)
ps.numberPaths()
ps.pathLength(ps.numberPaths()-1)
len(ps.getWaypoints (ps.numberPaths()-1))

ps.clearPathOptimizers()
#cl.problem.setAlphaInit (0.05)
ps.addPathOptimizer("GradientBased")
ps.optimizePath (1)
ps.numberPaths()
ps.pathLength(ps.numberPaths()-1)
tGB = cl.problem.getTimeGB ()
timeValuesGB = cl.problem.getTimeValues ()
gainValuesGB = cl.problem.getGainValues ()
newGainValuesGB = ((1-np.array(gainValuesGB))*100).tolist() #percentage of initial length-value

ps.clearPathOptimizers()
ps.addPathOptimizer('RandomShortcut')
ps.optimizePath (0)
ps.pathLength(ps.numberPaths()-1)

ps.clearPathOptimizers()
ps.addPathOptimizer('PartialShortcut')
ps.optimizePath (0)
ps.pathLength(ps.numberPaths()-1)

pp(ps.numberPaths()-1)


## -------------------------------------
import matplotlib.pyplot as plt
from plotfunctions import optAndConcatenate, getValIndex, computeMeansVector, reducedVectors, curvPlot, curvSdPlot
# OPTIMIZE AND Concatenate RS PRS values:
globalTimeValuesRS = []; globalGainValuesRS = []
globalTimeValuesPRS = []; globalGainValuesPRS = []
nbOpt = 10 # number of launchs of RS and PRS
optAndConcatenate (cl, ps, 1, nbOpt, 'RandomShortcut', globalTimeValuesRS, globalGainValuesRS)
optAndConcatenate (cl, ps, 1, nbOpt, 'PartialShortcut', globalTimeValuesPRS, globalGainValuesPRS)

nbPoints = 100 # number of points in graph
tVec = np.arange(0,tGB,tGB/nbPoints)
moyVectorRS = []; sdVectorRS = []; moyVectorPRS = []; sdVectorPRS = [];
computeMeansVector (nbOpt, tVec, moyVectorRS, sdVectorRS, globalTimeValuesRS, globalGainValuesRS)
computeMeansVector (nbOpt, tVec, moyVectorPRS, sdVectorPRS, globalTimeValuesPRS, globalGainValuesPRS)

tReduceVectorRS = []; meanReduceVectorRS = []; sdReduceVectorRS = [];
tReduceVectorPRS = []; meanReduceVectorPRS = []; sdReduceVectorPRS = [];
reducedVectors (tVec, moyVectorRS, sdVectorRS, tReduceVectorRS, meanReduceVectorRS, sdReduceVectorRS)
reducedVectors (tVec, moyVectorPRS, sdVectorPRS, tReduceVectorPRS, meanReduceVectorPRS, sdReduceVectorPRS)

# Plot lengthGain (t);
plt.axis([-.0005, tGB+0.0005, 81, 101])
plt.xlabel('t (s)'); plt.ylabel('Relative length reduction (%)')
vectorLengthGB = len (timeValuesGB)
plt.plot([0,tGB], [newGainValuesGB[vectorLengthGB-1],newGainValuesGB[vectorLengthGB-1]], 'b--')
plt.plot(0, 100, 'bo'); plt.plot([0,timeValuesGB[0]], [100,100], 'b', linewidth=1.5)
plt = curvSdPlot (plt, tGB, tReduceVectorRS, meanReduceVectorRS, sdReduceVectorRS, '0.55', 0.8, 0.0001)
plt = curvSdPlot (plt, tGB, tReduceVectorPRS, meanReduceVectorPRS, sdReduceVectorPRS, '0.55', 0.8, 0.0001)
plt = curvPlot (plt, tGB, timeValuesGB, newGainValuesGB, 'o', 'b', 1.5)
plt.plot([0,tReduceVectorRS[0]], [100,100], 'r', linewidth=1.1)
plt = curvPlot (plt, tGB, tReduceVectorRS, meanReduceVectorRS, '*', 'r', 1.5)
plt.plot([0,tReduceVectorPRS[0]], [100,100], 'g', linewidth=0.8)
plt = curvPlot (plt, tGB, tReduceVectorPRS, meanReduceVectorPRS, '+', 'g', 1.5)
plt.show()

# For different alpha_init
tmax = max(max(tGB,tGB2),max(tGB3,tGB4))
plt.axis([-.001, tmax+0.001, 80, 101])
plt.xlabel('t (s)'); plt.ylabel('Relative remaining length (%)')
vectorLengthGB = len (timeValuesGB)
#plt.plot([0,tGB], [newGainValuesGB[vectorLengthGB-1],newGainValuesGB[vectorLengthGB-1]], 'b--')
plt.plot(0, 100, 'bo'); plt.plot([0,timeValuesGB[0]], [100,100], 'b', linewidth=1.5)
plt = curvPlot (plt, tmax, timeValuesGB, newGainValuesGB, 'o', 'b', 1.5)
vectorLengthGB2 = len (timeValuesGB2)
plt.plot(0, 100, 'g*'); plt.plot([0,timeValuesGB2[0]], [100,100], 'g', linewidth=1.5)
plt = curvPlot (plt, tmax, timeValuesGB2, newGainValuesGB2, '*', 'g', 1.5)
vectorLengthGB3 = len (timeValuesGB3)
plt.plot(0, 100, 'r+'); plt.plot([0,timeValuesGB3[0]], [100,100], 'r', linewidth=1.5)
plt = curvPlot (plt, tmax, timeValuesGB3, newGainValuesGB3, '+', 'r', 1.5)
vectorLengthGB4 = len (timeValuesGB4)
plt.plot(0, 100, 'c+'); plt.plot([0,timeValuesGB4[0]], [100,100], 'c', linewidth=1.5)
plt = curvPlot (plt, tmax, timeValuesGB4, newGainValuesGB4, '+', 'c', 1.5)
plt.show()

## -------------------------------------

ps.clearPathOptimizers(); ps.addPathOptimizer("GradientBased")
cl.problem.setAlphaInit (0.05)
ps.optimizePath (0); tGB2 = cl.problem.getTimeGB ()
timeValuesGB2 = cl.problem.getTimeValues (); gainValuesGB2 = cl.problem.getGainValues ()
newGainValuesGB2 = ((1-np.array(gainValuesGB2))*100).tolist() #percentage of initial length-value

cl.problem.setAlphaInit (0.3)
ps.optimizePath (0); tGB3 = cl.problem.getTimeGB ()
timeValuesGB3 = cl.problem.getTimeValues (); gainValuesGB3 = cl.problem.getGainValues ()
newGainValuesGB3 = ((1-np.array(gainValuesGB3))*100).tolist() #percentage of initial length-value

cl.problem.setAlphaInit (0.4)
ps.optimizePath (0); tGB4 = cl.problem.getTimeGB ()
timeValuesGB4 = cl.problem.getTimeValues (); gainValuesGB4 = cl.problem.getGainValues ()
newGainValuesGB4 = ((1-np.array(gainValuesGB4))*100).tolist() #percentage of initial length-value

## -------------------------------------



from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (cl, r)
r.loadObstacleModel ("potential_description","obstacles_concaves","obstacles_concaves")

import numpy as np
dt = 0.1
nPath = 0
points = []
for t in np.arange(0., cl.problem.pathLength(nPath), dt):
    points.append ([cl.problem.configAtParam(nPath, t)[0],cl.problem.configAtParam(nPath, t)[1],0])

r.client.gui.addCurve ("initCurvPath",points,[1,0.3,0,1])
r.client.gui.addToGroup ("initCurvPath", r.sceneName)

nPath = 1
points = []
for t in np.arange(0., cl.problem.pathLength(nPath), dt):
    points.append ([cl.problem.configAtParam(nPath, t)[0],cl.problem.configAtParam(nPath, t)[1],0])

r.client.gui.addCurve ("optimCurvPathc",points,[0,0.3,1,1])
r.client.gui.addToGroup ("optimCurvPathc", r.sceneName)

r.client.gui.removeFromGroup ("optimCurvPath", r.sceneName)


## -------------------------------------
# Simple 2D trajectory plot (in Matplotlib)
import matplotlib.pyplot as plt
from mutable_trajectory_plot import planarPlot
#plt = planarPlot (cl, 0, ps.numberPaths()-1, plt) # if launched from potential_description
plt = planarPlot (cl, 0, ps.numberPaths()-1, plt, 1.2, 5) # if launched from robot_2d_description
plt.show()

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

## DEBUG commands
robot.isConfigValid(q1)
cl.robot.distancesToCollision()
from numpy import *
argmin(cl.robot.distancesToCollision()[0])
r( cl.problem.configAtParam(0,5) )
cl.problem.optimizePath (0)
cl.problem.clearRoadmap ()
cl.problem.resetGoalConfigs ()
#q1 = [2.4, -4.6, 1.0, 0.0]; q2 = [-0.4, 4.6, 1.0, 0.0] # obstS 2
#cl.obstacle.loadObstacleModel('potential_description','cylinder_obstacle','')

