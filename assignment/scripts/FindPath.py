#!/usr/bin/env python

from PathPlanner import PathPlanner
import numpy as np

'''PATH PLANNING'''

def findPath(start,goal,res,rad):
    #read in map
    map = np.loadtxt("map.txt")

    #create path planner using image of the map processed above
    test_planner = PathPlanner(map)
    test_planner.PRM(res)

    #plan path
    output = test_planner.plan_path(start,goal,rad)

    print("Path:")
    print(output)
    print("-------------------------")

    return output

