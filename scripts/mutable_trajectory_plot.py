#/usr/bin/env python
# Author : Mylene Campana
# Script to plot some graphs after launching a corbaserver and solving 
# the problem. The main DIFFERENCE here is that the plot is always returned so that 
# plots can be added later in the Python interface.
# Use has to call himself "plt.show()"

#import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math

dt = 0.02 # global drawing step size


# --------------------------------------------------------------------#

# Plot 2D trajectory and concaves obstacles for potential scenario. nPath is the path number.
# And special nodes in another color.
def planarPlot (cl, nPath, plt):
    Tvec = np.arange(0., cl.problem.pathLength(nPath), dt)
    
    # Plot obstacles :
    plt.gcf().gca().add_artist(plt.Circle((0.9,0.4),.6,color=(1, 0.4, 0.3))) # orange
    plt.text(0.9, 0.4, r'obst1', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((1.5,0.4),.6,color=(1, 0.4, 0.3))) # orange
    plt.text(1.5, 0.3, r'obst1bis', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((1.2,-3.5),1.2,color='green'))
    plt.text(1.2, -3.5, r'obst2', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((2.6,2.5),1.2,color='green'))
    plt.text(2.6, 2.5, r'obst6', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((-2.5,1.2),.6,color='gray'))
    plt.text(-2.5, 1.2, r'obst4', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((-1,4),.6,color='gray'))
    plt.text(-1, 4, r'obst5', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((-2.8,-2.2),1.2,color='b'))
    plt.text(-2.8, -2.2, r'obst3', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((3.8,-1),.6,color='b'))
    plt.text(3.8, -1, r'obst7', fontsize=11)
    plt.gcf().gca().add_artist(plt.Rectangle((-0.4,-0.9),0.8,1.8,color='r'))
    plt.text(0, 0, r'obst_base', fontsize=11)
    #plt.gcf().gca().add_artist(plt.Rectangle((0.6,0.1),1.4,0.6,color='r')) # (1.3,0.4)
    plt.gcf().gca().add_artist(plt.Circle((1.5,-1.8),0.6,color='b'))
    plt.text(1.5, -1.8, r'obst8', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((2., 1.2),0.4,color=(0, 0.5, 1))) # light_blue
    plt.text(2., 1.2, r'obst9', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((-1.6,1.4),0.5,color=(1, 0.5, 0))) # yellow
    plt.text(-1.6, 1.4, r'obst10', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((-1.5,2.3),0.5,color=(1, 0.5, 0))) # yellow
    plt.text(-1.5, 2.3, r'obst11', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((-0.9,3.),0.5,color=(1, 0.5, 0))) # yellow
    plt.text(-0.9, 3., r'obst12', fontsize=11)
    
    """i = 0
    for n in cl.problem.nodes() :
        if i>1: # avoid 2 first nodes (init and goal)
            plt.plot(n[0], n[1], 'ro')
            plt.text(n[0]+.02, n[1], r'qNew%i' %(i), fontsize=8)
        i=i+1
    """
    for t in Tvec:
        plt.plot([cl.problem.configAtParam(nPath, t)[0], \
                     cl.problem.configAtParam(nPath, t+dt)[0]], \
                     [cl.problem.configAtParam(nPath, t)[1], \
                     cl.problem.configAtParam(nPath, t+dt)[1]], 'k',  label="optim." if t == 0. else "")
    
    if (nPath == 1): # plot also initial path to compare
        for t in np.arange(0., cl.problem.pathLength(0), dt):
            plt.plot([cl.problem.configAtParam(0, t)[0], \
                     cl.problem.configAtParam(0, t+dt)[0]], \
                     [cl.problem.configAtParam(0, t)[1], \
                     cl.problem.configAtParam(0, t+dt)[1]], 'r', label="init." if t == 0. else "")
    
    plt.legend()
    plt.axis([-5, 5, -5, 5])
    plt.xlabel('x'); plt.ylabel('y')
    plt.title('trajectory'); plt.grid()
    plt.plot(cl.problem.nodes()[0][0], cl.problem.nodes()[0][1], 'go')
    plt.plot(cl.problem.nodes()[1][0], cl.problem.nodes()[1][1], 'go')
    plt.text(cl.problem.nodes()[0][0]+.1, cl.problem.nodes()[0][1]+.1, r'q_init', fontsize=10)
    plt.text(cl.problem.nodes()[1][0]+.1, cl.problem.nodes()[1][1]+.1, r'q_end', fontsize=10)
    return plt

# --------------------------------------------------------------------#

def planarPlotCylinder (cl, nPath, plt):
    Tvec = np.arange(0., cl.problem.pathLength(nPath), dt)
    
    plt.gcf().gca().add_artist(plt.Circle((0,0),.5,color='r')) # Plot red cylinder
    
    """i = 0
    for n in cl.problem.nodes() :
        if i>1: # avoid 2 first nodes (init and goal)
            plt.plot(n[0], n[1], 'ro')
            plt.text(n[0]+.02, n[1], r'qNew%i' %(i), fontsize=8)
        i=i+1
    """
    for t in Tvec:
        plt.plot([cl.problem.configAtParam(nPath, t)[0], \
                     cl.problem.configAtParam(nPath, t+dt)[0]], \
                     [cl.problem.configAtParam(nPath, t)[1], \
                     cl.problem.configAtParam(nPath, t+dt)[1]], 'k', linewidth=1.5, label="optim." if t == 0. else "")
    
    if (nPath == 1): # plot also initial path to compare
        for t in np.arange(0., cl.problem.pathLength(0), dt):
            plt.plot([cl.problem.configAtParam(0, t)[0], \
                     cl.problem.configAtParam(0, t+dt)[0]], \
                     [cl.problem.configAtParam(0, t)[1], \
                     cl.problem.configAtParam(0, t+dt)[1]], 'r', label="init." if t == 0. else "")
    
    plt.legend()
    plt.axis([-5, 5, -5, 5])
    plt.xlabel('x'); plt.ylabel('y')
    plt.title('trajectory'); plt.grid()
    plt.plot(cl.problem.nodes()[0][0], cl.problem.nodes()[0][1], 'go')
    plt.plot(cl.problem.nodes()[1][0], cl.problem.nodes()[1][1], 'go')
    plt.text(cl.problem.nodes()[0][0]+.1, cl.problem.nodes()[0][1]+.1, r'q_init', fontsize=10)
    plt.text(cl.problem.nodes()[1][0]+.1, cl.problem.nodes()[1][1]+.1, r'q_end', fontsize=10)
    return plt

# --------------------------------------------------------------------#

# Plot 2D nodes (from parseLog) with given color and text.
# For example, nodeName = r'qCol'    and   nodeColor = 'bo'
def addNodePlot (nodeList, nodeColor, nodeName, plt):
    i = 0
    for n in nodeList :
        plt.plot(n[0], n[1], nodeColor)
        plt.text(n[0]+.02, n[1], nodeName+'%i' %(i), fontsize=8)
        i = i+1
    return plt

# --------------------------------------------------------------------#

# Plot 2D trajectory (parsed from parseLog), with no label.
# For example, nodeName = r'qCol'    and   nodeColor = 'bo'
def addPathPlot (cl, path, pathColor, plt):
    size = len(path) # number of lines = 2*nbSegments
    print "pathSize= "+str(size)
    i = 0
    while(i < size-1):
        # plot segment
        plt.plot([path[i][0], path[i+1][0]], [path[i][1], path[i+1][1]], pathColor) 
        i = i+1 # go to next segment
    # Add first and last segment
    plt.plot([cl.problem.nodes()[0][0], path[0][0]], [cl.problem.nodes()[0][1], path[0][1]], pathColor)
    plt.plot([cl.problem.nodes()[1][0], path[size-1][0]], [cl.problem.nodes()[1][1], path[size-1][1]], pathColor)
    return plt

