# Motion Planning.

Currently consists the implementation of Dijkstra's, A star and RRT algorithms and a MapGenerator application that allows to generate simple .pgm images.

## Map Generator.
The application asks for size of the map followed by option to generate multiple rectangular obstacles on the map. With a little work on pen and paper, one can generate complex maps. 
![MapGenerator](https://user-images.githubusercontent.com/66659752/121116584-c8301f80-c7e4-11eb-8a15-9224a4354cfc.PNG)

## Dijkstra algorithm.
Dijkstra's algorithm solves for the optimal path. Note that the number of nodes searched are 3927.
![Dij484](https://user-images.githubusercontent.com/66659752/121966652-01b0df80-cd3d-11eb-9981-e0c34c5ca261.PNG)


## A* algorithm.
A* algorithm solves for the optimal path. Note that the number of nodes searched are 3394.
![AStar484](https://user-images.githubusercontent.com/66659752/121966718-1beabd80-cd3d-11eb-8a53-c5e6cdb7a2be.PNG)

## RRT Vs A* algorithm. 
![](images/A_starVs_RRT.png)

## RRT Vs RRT*
RRT*: Red RRT: Blue
![](images/RRT_Vs_RRT_Star.png)