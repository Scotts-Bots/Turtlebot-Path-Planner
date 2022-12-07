#!/usr/bin/env python

import random
from Graph import Graph
import numpy as np
#from skimage import io 

'''PATH PLANNER CLASS'''
#Can use different path planning techniques on the same data.
#Data can be read from the terminal or from a file if structured the correct way.

class PathPlanner:

    def __init__(self, map):
        #get input data
        self.binary_map = map

        #For PRM
        self.PRM_exec = False
        self.PRM_points = []

    #check if the two points are close and no obstacles in between
    '''def check_start_and_end_closeness(self,start,end,radius):
        a = np.array(start)
        b = np.array(end)
        dist = np.linalg.norm(a-b)
        if dist <= radius:
            i0 = np.min([a[0],b[0]])
            i1 = np.max([a[0],b[0]])
            j0 = np.min([a[1],b[1]])
            j1 = np.max([a[1],b[1]])
            for i in range(i0,i1):
                for j in range(j0,j1):
                    if (not self.binary_map[i][j]): return False

            return True
        else:
            return False'''
        

    #plan a path using PRM for specified start and end coordinates
    def plan_path(self, start, end, radius):

        #check if PRM has been implemented (i.e. the graph exists)
        if (self.PRM_exec):
            if (self.binary_map[start] and self.binary_map[end]):
                #add start and end nodes to the graph if they don't exist
                if (start not in self.points):
                    self.graph.addMilestone([start[0],start[1]]) 
                    start_milestone = self.graph.milestones[-1]
                #get milestone if it does exist
                else:
                    start_milestone = self.graph.getMilestoneFromCoords(start)

                if (end not in self.points):
                    self.graph.addMilestone([end[0],end[1]])
                    end_milestone = self.graph.milestones[-1]
                else:
                    end_milestone = self.graph.getMilestoneFromCoords(end)
                
                #set start and end nodes in graph
                self.graph.setStartandEnd(start_milestone.milestoneNumber,end_milestone.milestoneNumber)

                #TODO: find nearest neighbours for start and end nodes
                self.graph.linearNearNeighbours(radius)

                #TODO: call path plan here
                path = self.graph.dijkstra_algorithm()
                if path == None: return
                # self.print_path(path)
                return path

            else:
                print("Invalid coordinate/s. One or both of them intersect an obstacle. Try new coordinates.")
                return

        else:
            print("No graph has been built. Call PRM() first to build graph.")
            return

    #print map
    '''def print_map(self):
        io.imshow(self.binary_map)'''

    #cleans data to get integer coordinate pairs
    def get_coords(self, line):
        coord1,coord2 = line.split(';')
        coord1 = list(map(int,coord1.split(',')))
        coord2 = list(map(int,coord2.split(',')))

        return (coord1, coord2)

    #print details in console
    '''def print_details(self):
        print("--------------------------------------------------------------------")
        print("    start:  ",self.start)
        print("     stop:  ",self.stop)
        print("      map:  ",self.binary_map)
        print("--------------------------------------------------------------------")'''
    
    #print the resulting path in a specific way
    def print_path(self,path):
        for waypoint in path:
            print(str(waypoint[0])+","+str(waypoint[1]))

    #Probabilistic Roadmap (PRM)
    def PRM(self, resolution):          #resolution is how many sample points you want
        #Rebuild graph everytime PRM is called
        self.graph = Graph(self.binary_map)
        res = 0
        self.points = []

        #create valid sample points (i.e. does not intersect with obstacles)
        while res < resolution:
            samplePoint = [random.randint(0,self.binary_map.shape[0]-1),random.randint(0,self.binary_map.shape[1]-1)]
            #print(samplePoint)
            if (self.binary_map[samplePoint[0]][samplePoint[1]] and samplePoint not in self.points):
                #check if the radius of this point is clear to avoid wall collisions
                no_obstacles = True
                for h in range(-1,2):
                    for v in range(-1,2):
                        no_obstacles = self.binary_map[samplePoint[0]+h][samplePoint[1]+v]
                        if (not no_obstacles): break
                    if (not no_obstacles): break
                if(not no_obstacles): continue

                self.graph.addMilestone([samplePoint[0],samplePoint[1]])
                self.points.append(samplePoint)
                res+=1

        #find nearest neighbours within a fixed radius for each point
        #self.graph.linearNearNeighbours(radius)

        #indicates a PRM graph exists 
        self.PRM_exec = True