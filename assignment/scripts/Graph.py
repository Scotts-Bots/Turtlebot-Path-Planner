#!/usr/bin/env python

import numpy as np
import sys

'''MILESTONE CLASS'''

class Milestone:
    def __init__(self, x, y, n):
        self.milestoneNumber = n
        self.x = x
        self.y = y
        self.adjacencies = []
        self.distances = dict()
        self.position = (x,y)

    def addAdjacency(self,v):
        self.adjacencies.append(v)

    def isAdjacent(self, v):
        return v in self.adjacencies

    def getDegree(self):
        return len(self.adjacencies)


'''GRAPH CLASS'''

class Graph:
    def __init__(self,map):
        self.image_map = map
        self.milestones = []
        self.start = [] #Empty means they haven't been initialized
        self.end = []

    #set start and end labels
    def setStartandEnd(self, start, end):
        self.start = start
        self.end = end

    #add a milestone
    def addMilestone(self,coords):
        newMilestone = Milestone(coords[0],coords[1],len(self.milestones))
        self.milestones.append(newMilestone)

    #returns a milestone object corresponding to its number label
    def getMilestone(self,n):
        return self.milestones[n]
    
    #returns a milestone object corresponding to its coordinates
    def getMilestoneFromCoords(self,coords):
        for m in self.milestones:
            if coords[0] == m.x and coords[1] == m.y:
                return m
        print("Milestone doesn't exist.")

    #adds an edge from v1 to v2 (undirected)
    def addEdge(self,v1,v2):
        m1 = self.getMilestone(v1)
        m1.addAdjacency(v2)
        m2 = self.getMilestone(v2)
        m2.addAdjacency(v1)

    #display all milestones
    def printMilestones(self):
        print("-------------------------------")
        for i in self.milestones:
            print(i.milestoneNumber,i.x,i.y,"  adj:   ", i.adjacencies, i.distances)
        print("-------------------------------")      

    #voxel traversal algorithm - CREDITS BELOW
    '''def voxelTraversal(self,u,v):
        #initialization phase
        X,Y = u
        uv = v - u 
        stepX,stepY = 1,1 #assumption: we always move in positive direction
        tMaxX,tMaxY = 1.5,1.5
        tDeltaX,tDeltaY = 1,1

        #incremental phase
        crossed_cells = []
        temp_cell = u
        while (temp_cell != v):
            if (tMaxX < tMaxY):
                tMaxX += tDeltaX
                X += stepX
            else:
                tMaxY += tDeltaY
                Y += stepY
            temp_cell = (X,Y)
            if temp_cell not in crossed_cells:
                crossed_cells.append(temp_cell)
        
        return crossed_cells '''

    '''def LNNforSinglePoint(self,r,milestone):
        #find a single point withing the radius
        for n in range(m+1,numMilestones):
            a = np.array([self.milestones[m].x,self.milestones[m].y])
            b = np.array([self.milestones[n].x,self.milestones[n].y])
            dist = np.linalg.norm(a-b)
            #print("dist  ", m, n, dist)

            #check if the point is within the radius of our current point
            if dist <= r: '''
    
    #find find fixed-radius (r) near neighbours for each milestone (m) in a grid
    def linearNearNeighbours(self,r):
        numMilestones = len(self.milestones)
        for m in range(numMilestones):
            for n in range(m+1,numMilestones):
                a = np.array([self.milestones[m].x,self.milestones[m].y])
                b = np.array([self.milestones[n].x,self.milestones[n].y])
                dist = np.linalg.norm(a-b)
                #print("dist  ", m, n, dist)

                #check if the point is within the radius of our current point
                if dist <= r: 
                    #check if the edge between these two points intersect an obstacle

                    if np.abs(b[0]-a[0]) != 0:
                        #line between those two points
                        grad = (b[1]-a[1])/(b[0]-a[0])
                        y_intercept = a[1] - grad*a[0]
                        f = lambda p: grad*p + y_intercept

                        if grad != 0:
                            if a[0] < b[0]:   
                                #check x-axis cells in between (and including) origin and target cells
                                midcondition = True
                                for x in range(a[0],b[0]+1):
                                    if b[0]+1 > self.image_map.shape[0]: continue
                                    y = int(np.floor(f(x)))
                                    if a[1] < b[1]:
                                        for k in range(a[1],b[1]):
                                            midcondition = midcondition and self.image_map[x,k]
                                    else:
                                        for k in range(b[1],a[1]):
                                            midcondition = midcondition and self.image_map[x,k]
                                    if (not midcondition): break
                                if (not midcondition): continue
                            
                            else:
                                #check x-axis cells in between origin and target cells
                                midcondition = True
                                for x in range(b[0],a[0]+1):
                                    if a[0]+1 > self.image_map.shape[0]: continue
                                    y = int(np.floor(f(x)))
                                    if a[1] < b[1]:
                                        for k in range(a[1],b[1]):
                                            midcondition = midcondition and self.image_map[x,k]
                                    else:
                                        for k in range(b[1],a[1]):
                                            midcondition = midcondition and self.image_map[x,k]
                                    if (not midcondition): break
                                if (not midcondition): continue

                        #zero gradient case - line in x direction only
                        else:
                            midcondition = True
                            if a[0] < b[0]:
                                for x in range(a[0]+1,b[0]):
                                    midcondition = self.image_map[x,a[1]]
                                    if (not midcondition): break
                                if (not midcondition): continue
                            else:
                                for x in range(b[0]+1,a[0]):
                                    midcondition = self.image_map[x,a[1]]
                                    if (not midcondition): break
                                if (not midcondition): continue                            

                    #division by zero case - line in y direction only
                    else:
                        midcondition = True
                        if a[1] < b[1]:
                            for y in range(a[1]+1,b[1]):
                                midcondition = self.image_map[a[0],y]
                                if (not midcondition): break
                            if (not midcondition): continue
                        else:
                            for y in range(b[1]+1,a[1]):
                                midcondition = self.image_map[a[0],y]
                                if (not midcondition): break
                            if (not midcondition): continue

                    #add edge between two milestones
                    self.addEdge(m,n)
                    self.milestones[m].distances[n] = dist
                    self.milestones[n].distances[m] = dist

    def dijkstra_algorithm(self):
        unvisited_nodes = list(range(len(self.milestones)))
        start_node = self.start
        end_node = self.end
    
        # We'll use this dict to save the cost of visiting each node and update it as we move along the graph   
        shortest_path = {}
    
        # We'll use this dict to save the shortest known path to a node found so far
        previous_nodes = {}
    
        # We'll use max_value to initialize the "infinity" value of the unvisited nodes   
        max_value = sys.maxsize
        for node in unvisited_nodes:
            shortest_path[node] = max_value
        # However, we initialize the starting node's value with 0   
        shortest_path[start_node] = 0
        
        # The algorithm executes until we visit all nodes
        while unvisited_nodes:
            # The code block below finds the node with the lowest score
            current_min_node = None
            for node in unvisited_nodes: # Iterate over the nodes
                if current_min_node == None:
                    current_min_node = node
                elif shortest_path[node] < shortest_path[current_min_node]:
                    current_min_node = node
                    
            # The code block below retrieves the current node's neighbors and updates their distances
            neighbors = self.getMilestone(current_min_node).adjacencies
            for neighbor in neighbors:
                tentative_value = shortest_path[current_min_node] + self.getMilestone(current_min_node).distances[neighbor]
                if tentative_value < shortest_path[neighbor]:
                    shortest_path[neighbor] = tentative_value
                    # We also update the best path to the current node
                    previous_nodes[neighbor] = current_min_node
    
            # After visiting its neighbors, we mark the node as "visited"
            unvisited_nodes.remove(current_min_node)
        
        #print(previous_nodes, shortest_path)

        #obtain output path
        final_path = []
        final_path.append(end_node) #add end node
        try:
            val = previous_nodes[end_node]
            while val != start_node:
                final_path.append(val)
                val = previous_nodes[val]
            final_path.append(start_node) #add start node
        except:
            print("Path does not exist to goal. Try running program again with a higher resolution and/or higher radius.")
            print()
            return

        #print(final_path)

        #obtain final path in waypoint coordinates
        waypoints = []
        tot_wps = len(final_path)-1
        for i in range(tot_wps+1):
            waypoints.append([self.getMilestone(final_path[tot_wps-i]).x,self.getMilestone(final_path[tot_wps-i]).y])

        return waypoints


'''CREDITS FOR DJIKSTRA'S ALGORITHM'''
#https://www.udacity.com/blog/2021/10/implementing-dijkstras-algorithm-in-python.html