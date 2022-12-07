import numpy as np
import random
from Graph import Graph, Milestone

'''PATH PLANNER CLASS'''
#Can use different path planning techniques on the same data.
#Data can be read from the terminal or from a file if structured the correct way.

class PathPlanner:

    def __init__(self, filename=None):
        #get input data from
        if filename == None:
            print("fill in details")
            self.input_data = self.read_input()
        else:
            self.input_data = self.file_read_input(filename)

        #cleaning data
        self.start,self.stop = self.get_coords(self.input_data[0])
        self.obstacles = []
        for i in range(1,len(self.input_data)):
            self.obstacles.append(self.get_coords(self.input_data[i]))

        #calculate a theoretical max
        self.border = np.max(self.obstacles)

        self.print_details()


    #function to get input data from terminal
    def read_input(self):
        x = input()
        output = []

        #get input data from terminal input
        while x != "-1":
            output.append(x)
            x = input()

        return output


    #function to get input data from a file
    def file_read_input(self, filename):
        file = open(filename+".txt","r")

        output = []
        x = file.readline().strip()
        while x != "-1":
            output.append(x)
            x = file.readline().strip()

        file.close()
        return output


    #cleans data to get integer coordinate pairs
    def get_coords(self, line):
        coord1,coord2 = line.split(';')
        coord1 = list(map(int,coord1.split(',')))
        coord2 = list(map(int,coord2.split(',')))

        return (coord1, coord2)


    #print details in console
    def print_details(self):
        print("--------------------------------------------------------------------")
        print("    start:  ",self.start)
        print("     stop:  ",self.stop)
        print("obstacles:  ",self.obstacles)
        print("--------------------------------------------------------------------")

    #check if a point intersects any of the obstacles.
    def intersects_obstacle(self, point):
        x, y = point
        
        #check if there is an interaction with any obstacle for the input point
        for block in self.obstacles:
            horizontalIntersect =  x >= block[0][0] and x <= block[1][0]
            verticalIntersect =  y >= block[0][1] and y <= block[1][1]
            if horizontalIntersect and verticalIntersect:
                return True #intersection found
        
        return False #no intersection with obstacles found
    
    #print the resulting path in a specific way
    def print_path(self,path):
        for waypoint in path:
            print(str(waypoint[0])+","+str(waypoint[1]))

    #Probabilistic Roadmap (PRM)
    def PRM(self, resolution, radius):          #resolution is how many sample points you want
        graph = Graph()
        res = 0

        #create valid sample points (i.e. does not intersect with obstacles)
        while res < resolution:
            samplePoint = [random.randint(0,self.border+10),random.randint(0,self.border+10)]
            #print(samplePoint)
            if (not self.intersects_obstacle(samplePoint)):
                graph.addMilestone(Milestone(samplePoint[0],samplePoint[1],res))
                res+=1

        #add start and end goals with the assumption that they are not intersecting obstacles!
        graph.addMilestone(Milestone(self.start[0],self.start[1],res))
        res += 1
        graph.addMilestone(Milestone(self.stop[0],self.stop[1],res))

        #find nearest neighbours within a fixed radius for each point
            #graph.printMilestones()
        graph.naiveNearNeighbours(radius,self.obstacles)
            #graph.printMilestones()

        #run a-star to find a path in our graph
        path = graph.dijkstra_algorithm()
        if path == None: return
        self.print_path(path)
