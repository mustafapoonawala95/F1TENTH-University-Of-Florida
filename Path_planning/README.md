# Motion Planning.

Currently consists the implementation of the following path planning algorithms.   
1. Dijkstra's  
2. A\*  
3. RRT  
4. RRT\*  <br/>

<!-- -->

Each path planning algorithm subscribes to the __/map__ topic for the occupancy grid map, __/gt_pose__ for the car's position and __/move_base_simple/goal__ for the goal position. The calculated path is then published on the appropriately named topic.<br/><br/>
A pure pursuit path tracking controller and MapGenerator application that allows to generate simple .pgm images are also provided to evaluate algorithms on different types of maps without the need of sensors to create maps.


## Dijkstra's algorithm.
Finds a shortest path tree from a single source node, by building a set of nodes that have minimum distance from the source.

The graph has the following:

* vertices, or nodes, denoted in the algorithm by *v* or *u*;
* weighted edges that connect two nodes: *(u,v)* denotes an edge, and *w(u,v)* denotes its weight. 

This is done by initializing three values:

* _dist_, an array of distances from the source node *s* to each node in the graph, initialized the following way: *dist**(s)* = 0; and for all other nodes *v*, *dist(v)* = infinity. This is done at the beginning because as the algorithm proceeds, the *dist* from the source to each node *v* in the graph will be recalculated and finalized when the shortest distance to *v* is found
* *Q*, a min priority queue of all nodes in the graph. The priority queue is evaluated based on the *dist* values of the nodes. At the end of the algorithm's progress, *Q* will be empty.
* *S*, an empty set, to indicate which nodes the algorithm has visited.

The algorithm proceeds as follows:

* While *Q* is not empty, pop the node *v*, that is not already in *S*, from *Q* with the smallest *dist(v)*. In the first run, source node *s* will be chosen because *dist(s)* was initialized to 0. In the next run, the next node with the smallest *dist* value is chosen.
* Add node *v* to *S*, to indicate that *v* has been visited
Update *dist* values of adjacent nodes of the current node *v* as follows: for each new adjacent node *u*,

if *dist*(*v*) + *weight(u,v)* < *dist(u)*, there is a new minimal distance found for *u*, so update *dist(u)* to the new minimal distance value;
otherwise, no updates are made to *dist(u)*.

The algorithm terminates when the goal node has been visited in the graph and the smallest distance from *s* to the goal node has been found. *dist* now contains the shortest path tree from source *s* to all other visited nodes.

The animation below illustrates the working of the algorithm.  

<p>
    <img src="images/Dijkstra_Animation.gif" width="300" height="300">
    <em>Image credit: Wikipedia</em>
</p>

## A\* algorithm.
The implementation of the A\* algorithm is almost identical to the Dijkstra's algorithm. The only difference is that the metric for judging nodes is not solely based on the node's *dist* value, but the sum of *dist* and a heuristic *h*. The heuristic *h* of a node is an estiamte of the cost of reaching the goal from that node.  
Let the quantity *f* = *dist* + *h*, then the priority in A\* will be evaluated on the basis of *f* instead of *dist* like in dijkstra. This way the next node to be searched (popped from the priority queue) will be the one with lowest *f*. This translates to the "Best first search" approach since the node with the lowest *f* is __likely__ to be the best. This reduces the number of nodes to be searched to find the path to the goal as nodes that are __less likely__ to be on the shortest path are searched less often.  <br/><br/>
The difference between the number of nodes searched by A\* and dijkstra can be seen in the following image where the blue cells are the ones searched and the yellow ones are the final path.    

<p>
    <img src="images/A_star_vs_dijkstra.png" width="750" height="500">
</p>

## RRT algorithms.
The dijkstra's and A\* algorithm are search based algorithms, where the shortest path to the goal is searched from the existing nodes. RRT and RRT\* are sampling based algorithms that do not use an existing graph of the map but create a tree structure by sampling from the continuous configuration space.

### RRT
* An RRT grows a tree rooted at the starting point by using random samples from the search space. As each sample is drawn, a connection is attempted between it and the nearest state in the tree. 
* If the connection is feasible (passes entirely through free space and obeys any constraints), this results in the addition of the new state to the tree. 
* With uniform sampling of the search space, the probability of expanding an existing state is proportional to the size of its Voronoi region. As the largest Voronoi regions belong to the states on the frontier of the search, this means that the tree preferentially expands towards large unsearched areas.
* The length of the connection between the tree and a new state is frequently limited by a growth factor. If the random sample is further from its nearest state in the tree than this limit allows, a new state at the maximum distance from the tree along the line to the random sample is used instead of the random sample itself. The random samples can then be viewed as controlling the direction of the tree growth while the growth factor determines its rate. This maintains the space-filling bias of the RRT while limiting the size of the incremental growth. 

<p>
    <img src="images/rrt.gif" width="500" height="500">
    <em>Image credit: Maurice Rahme</em>
</p>

## RRT Vs A* algorithm. 
![](images/A_starVs_RRT.png)

## RRT Vs RRT*                          
RRT*: Red RRT: Blue
![](images/RRT_Vs_RRT_Star.png)

## Pure Pursuit path tracking with path planned by A*.
![](images/A_star_with_pure_pursuit.gif)
