This custom path planning module is implemented in the assignment. So far it only has a simple implementation of Probabilistic Roadmap (PRM). To run it separately on a specified environment, follow the instructions below:

- Create a file that specifies the 2D flat space. The format for this is as follows: first line has two sets of coordinates, the first being the starting point and the second being the finishing point. The next n lines specify the rectangular obstacles - first coordinate is the top-left corner and the second coordinate is the bottom-right corner for this rectangle. The last line must be -1 so the code knows when to finish reading this file.
- Create a PathPlanner object in main.py with your file name as input, then call PRM.
- Change resolution and radius values if need be for PRM.
- Run main.py