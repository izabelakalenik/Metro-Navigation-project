# Metro-Navigation-project
Project for the Artificial Intelligence course - 6th semester.

This project is a Python-based navigation application for metro systems, 
allowing users to find the optimal path between an origin and a destination based on different preference criteria. 
The application considers the following preferences separately:
* **Time:** Minimize the time taken to reach the destination.
* **Distance:** Minimize the total distance traveled.
* **Line Changes:** Minimize the number of metro line changes.
The user can input the origin and destination by the names of the stations or in Cartesian coordinates. The application assumes straight-line distances between user locations and metro stations.

## Algorithms
* Depth First Search (DFS)
* Breadth First Search (BFS)
* Uniform Cost Search (UCS)
* A Search*
* Improved A Search*: Accounts for walking times to and from metro stations.

