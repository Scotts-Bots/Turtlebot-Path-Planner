import numpy as np
import doIntersect
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
    def __init__(self):
        self.milestones = []

    #add a milestone
    def addMilestone(self,newMilestone):
        self.milestones.append(newMilestone)

    #returns a milestone object corresponding to its number label
    def getMilestone(self,n):
        return self.milestones[n]

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

    #check if a line intersects any sides of a rectangle
    def intersectsBox(self, a, b, obstacle):
        #line segment p1q1
        p1 = doIntersect.Point(a[0],a[1])
        q1 = doIntersect.Point(b[0],b[1])
        x0 = doIntersect.Point(obstacle[0][0],obstacle[0][1])               # x0 - - - - - - x1
        x1 = doIntersect.Point(obstacle[0][0],obstacle[1][1])               #  |              |
        x2 = doIntersect.Point(obstacle[1][0],obstacle[0][1])               #  |              |
        x3 = doIntersect.Point(obstacle[1][0],obstacle[1][1])               # x2 - - - - - - x3

        #check top side - x0x1
        if (doIntersect.doIntersect(p1,q1,x0,x1)): return True
        
        #check bottom side - x2x3
        elif (doIntersect.doIntersect(p1,q1,x2,x3)): return True

        #check left side - x0x2
        elif (doIntersect.doIntersect(p1,q1,x0,x2)): return True

        #check right side - x1x3
        elif (doIntersect.doIntersect(p1,q1,x1,x3)): return True

        #no intersection with rectangle
        else: return False


    #find fixed-radius (r) near neighbours for each milestone (m)
    def naiveNearNeighbours(self, r, obstacles):
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
                    obstacleInWay = False
                    for obs in obstacles:
                        if self.intersectsBox(a,b,obs):
                            obstacleInWay = True
                            break

                    if (not obstacleInWay):
                        #print(dist)
                        self.addEdge(m,n)
                        self.milestones[m].distances[n] = dist
                        self.milestones[n].distances[m] = dist


    def dijkstra_algorithm(self):
        unvisited_nodes = list(range(len(self.milestones)))
        start_node = len(unvisited_nodes)-2
        end_node = len(unvisited_nodes)-1
    
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