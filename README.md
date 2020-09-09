# RRT_STAR
C++ implementation of RRT* algorithm [1].

A rapidly exploring random tree (RRT) is an algorithm planned to search non-convex high-dimensional spaces by randomly building a tree. The tree is constructed by random samples drawn from the search space and is influenced to grow towards unsearched areas of the problem. The method grows a tree rooted in the starting configuration from the search space by using random samples. A connection is attempted between the sample and the nearest state in the tree, as and when each sample is drawn. Suppose the connection is feasible then the addition of the new state to the tree. RRT growth can be influenced by changing the probability of sampling states.

RRT* modifies RRT to consider both the incoming and outgoing edges of new vertices to improve the costs-to-come of existing vertices. The algorithm finds those that locally minimize cost-to-come and maintains a tree. Both algorithms are probabilistically complete and almost-surely asymptotically optimal. RRT* maintains a tree through rewiring. Instead of connecting new vertices to the nearest vertex in the tree, it connects them to the nearby vertex that minimizes their cost-to-come. It then considers whether this new vertex can lower the costs-to-come of existing vertices. Any neighbouring vertices that can be improved are rewired to be descendants of the new vertex. This allows existing vertices to benefit from future samples and simultaneously enhances the tree while searching the problem domain.

Description of RRT* algorithm: 
1. Pick a random node "N_rand".
2. Find the closest node "N_Nearest" from explored nodes to branch out towards "N_rand".
3. Steer from "N_Nearest" towards "N_rand": interpolate if node is too far away. The interpolated Node is "N_new"
4. Check if an obstacle is between new node and nearest nod.
5. Update cost of reaching "N_new" from "N_Nearest", treat it as "cmin". For now, "N_Nearest" acts as the parent node of "N_new".
6. Find nearest neighbors with a given radius from "N_new", call them "N_near"
7. In all members of "N_near", check if "N_new" can be reached from a different parent node with cost lower than Cmin, and without colliding with the obstacle. Select the node that results in the least cost and update the parent of "N_new".
8. Add N_new to node list.
9. Rewire the tree if possible. Search through nodes in "N_near" and see if changing their parent to "N_new" lowers the cost of the path. If so, rewire the tree and add them as the children of "N_new" and update the cost of the path.
10. Continue until maximum number of nodes is reached or goal is hit.
11. After returning an initial solution, the algorithm will iterate to improve the initial solution.

In this implementation, it is assumed that the robot is dimensionless and the algorithm is not responsible for generating velocities and commanding wheels. 

A [MATLAB code](https://github.com/Ali-tp/RRTSTAR/blob/master/Mfiles/plot_path.m) has been written to plot the generated paths by the RRT* algorithm. A sample output of the implemented RRT* algorithm is shown below.

![Sample output](https://github.com/Ali-tp/RRTSTAR/blob/master/Mfiles/Paths-1.png)

Note that in this figure, the top left point is the starting point and the bottom right point is the destination.

### Installation:
```shell
$ mkdir -p ~/RRTStar_ws/src
$ cd RRTStar_ws/src
$ git clone https://github.com/Ali-tp/RRTSTAR.git
$ cd ..
$ mkdir build
$ cd build
$ sudo cmake ../src/RRTSTAR
$ sudo make
```

### Run the RRT* algorithm:
```shell
$ cd ~/RRTStar_ws/src/RRTSTAR
$ ./RRTstar
```

[1] S. Karaman and E. Frazzoli, “Incremental sampling-based algorithms for optimal motion planning,” in Proc. Robotics: Science and Systems (RSS), 2010
