# Motion Planning.

Currently consists the implementation of Dijkstra's, A\*, RRT and RRT\* algorithms and a pure pursuit path tracking controller. A MapGenerator application that allows to generate simple .pgm images is also provided, to evaluate algorithms on different types of maps without the need of sensors to create maps.

## Dijkstra's algorithm.
Finds a shortest path tree from a single source node, by building a set of nodes that have minimum distance from the source.

The graph has the following:

* vertices, or nodes, denoted in the algorithm by *v* or *u*;
* weighted edges that connect two nodes: (*u,v*) denotes an edge, and w(*u,v*) denotes its weight. 

This is done by initializing three values:

* _dist_, an array of distances from the source node *s* to each node in the graph, initialized the following way: *dist*(*s*) = 0; and for all other nodes *v*, *dist*(*v*) = infinity. This is done at the beginning because as the algorithm proceeds, the *dist* from the source to each node *v* in the graph will be recalculated and finalized when the shortest distance to *v* is found
* *Q*, a min priority queue of all nodes in the graph. At the end of the algorithm's progress, *Q* will be empty.
* *S*, an empty set, to indicate which nodes the algorithm has visited.

The algorithm proceeds as follows:

* While *Q* is not empty, pop the node *v*, that is not already in *S*, from *Q* with the smallest *dist*(*v*). In the first run, source node *s* will be chosen because *dist*(*s*) was initialized to 0. In the next run, the next node with the smallest distdistdist value is chosen.
* Add node *v* to *S*, to indicate that *v* has been visited
Update *dist* values of adjacent nodes of the current node *v* as follows: for each new adjacent node *u*,

if *dist*(*v*) + *weight(u,v)* < *dist(u)*, there is a new minimal distance found for *u*, so update *dist(u)* to the new minimal distance value;
otherwise, no updates are made to *dist(u)*.

The algorithm has visited all nodes in the graph and found the smallest distance to each node. *dist* now contains the shortest path tree from source *s*.

The animation below illustrates the working of the algorithm.
![](images/Dijkstra_Animation.gif)

## RRT Vs A* algorithm. 
![](images/A_starVs_RRT.png)

## RRT Vs RRT*
RRT*: Red RRT: Blue\
![](images/RRT_Vs_RRT_Star.png)

## Pure Pursuit path tracking with path planned by A*.
![](images/A_star_with_pure_pursuit.gif)
