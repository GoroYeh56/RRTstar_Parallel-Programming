/**
 *  This is the main file that calls the RRT* algorithm. 
 *  First, the algorithm generates a plan (vector of points) which is a first viable solution.
 *  Next, it calls the RRT* algorithm on the previouslly built plan to optimize it.
 */


// TODOs:

// MPI, OpenMP(Pthread), CUDA

// 0. Add openmp flag!
// 1. Measure time & Parallelize
// 2. Compare sequential time & Parallelize time
// 3. Generate different test cases. (Size of the map)
// 4. Visualize random exploration points. (Visible point)



#include"RRTstar.h"
#include"omp.h"
#include <string> // string concatination.


#define TIME

/* ----------- Parameters --------------- */
// const int K = 1000; // 1000 points.
std::string OBSTACLES_FILE = "Mfiles//Obstacles_c4.txt";


//main function
int main(int argc, char** argv)
{

    #ifdef TIME 
        float total_time = 0;
        float starttime, endtime;
        starttime = omp_get_wtime();
    #endif


    if(argc != 5){
        std::cout<<"Error format, :=> ./RRT K <first_path_name> <opt_path_name> <available_pts_name>\n" ;
        return 0;
    }

    int K = atoi(argv[1]);
    std::string FIRST_PATH_FILE = "Mfiles//FirstPath/" + std::string(argv[2]);
    std::string OPTIMIZE_PATH_FILE =  "Mfiles//OptPath/" + std::string(argv[3]);
    // std::string OPTIMIZE_PATH_FILE = "Mfiles//Path_after_MAX_ITER.txt";
    std::string AVAILABLE_PATH_FILE =  "Mfiles//AvailablePts/" + std::string(argv[4]);


    //define start and end positions
    Point start_pos(25,475);
    Point end_pos(475, 25);
    
    //define the raduis for RRT* algorithm (Within a radius of r, RRT* will find all neighbour nodes of a new node).
    float rrt_radius = 25;
    //define the radius to check if the last node in the tree is close to the end position
    float end_thresh = 10;
    //instantiate RRTSTAR class
    RRTSTAR* rrtstar = new RRTSTAR(start_pos,end_pos, rrt_radius, end_thresh);

    //set the width and height of the world
    rrtstar->world->setWorldWidth(500.0);
    rrtstar->world->setWorldHeight(500.0);

    // set step size and max iterations. If the values are not set, the default values are max_iter=5000 and step_size=10.0
    // rrtstar->setMaxIterations(10000);
    rrtstar->setMaxIterations(100);
    // rrtstar->setStepSize(10.0);
    rrtstar->setStepSize(10.0);

    //Create obstacles
    //Obstacle 1
    Point ob1_1(0, 400); //position of the top left point of obstacle 1
    Point ob1_2(350, 350.0); //position of the bottom right point of obstacle 1
    rrtstar->world->addObstacle(ob1_1, ob1_2);//create obstacle 1
    //Obstacle 2;
    Point ob2_1(150, 150.0); //position of the top left point of obstacle 2
    Point ob2_2(500, 100.0); //position of the bottom right point of obstacle 2
    rrtstar->world->addObstacle(ob2_1, ob2_2);//create obstacle 2

    //Obstacle 3
    Point ob3_1(30, 30); //position of the top left point of obstacle 3
    Point ob3_2(60, 60); //position of the bottom right point of obstacle 3
    rrtstar->world->addObstacle(ob3_1, ob3_2);//create obstacle 3
    //Obstacle 4;
    Point ob4_1(250, 200.0); //position of the top left point of obstacle 4
    Point ob4_2(350, 300.0); //position of the bottom right point of obstacle 4

    rrtstar->world->addObstacle(ob4_1, ob4_2);//create obstacle 4
    //Save obstacles to  file;
    rrtstar->world->saveObsToFile(OBSTACLES_FILE);

    //clear saved paths from previous run
    // rrtstar->savePlanToFile({}, "Mfiles//first_viable_path.txt", {});
    // rrtstar->savePlanToFile({}, "Mfiles//Path_after_MAX_ITER.txt", {});
    rrtstar->savePlanToFile({}, FIRST_PATH_FILE, {});
    rrtstar->savePlanToFile({}, OPTIMIZE_PATH_FILE, {});



    // RRT* Algorithm
    /*
     Description of RRT* algorithm: 
    1. Pick a random node "N_rand".
    2. Find the closest node "N_Nearest" from explored nodes to branch out towards "N_rand".
    3. Steer from "N_Nearest" towards "N_rand": interpolate if node is too far away. The interpolated Node is "N_new"
    4.  Check if an obstacle is between new node and nearest nod.
    5. Update cost of reaching "N_new" from "N_Nearest", treat it as "cmin". For now, "N_Nearest" acts as the parent node of "N_new".
    6. Find nearest neighbors with a given radius from "N_new", call them "N_near"
    7. In all members of "N_near", check if "N_new" can be reached from a different parent node with cost lower than Cmin, and without colliding
    with the obstacle. Select the node that results in the least cost and update the parent of "N_new".
    8. Add N_new to node list.
    9. Rewire the tree if possible. Search through nodes in "N_near" and see if changing their parent to "N_new" lowers the cost of the path. If so, rewire the tree and
    add them as the children of "N_new" and update the cost of the path.
    10. Continue until maximum number of nodes is reached or goal is hit.
    */

/*
    std::cout << "Starting RRT* Algorithm..." << std::endl;
    //search for the first viable solution
    std::vector<Point> initial_solution =rrtstar->planner();
    
    //save initial solution
    rrtstar->savePlanToFile(initial_solution, FIRST_PATH_FILE, "First viable solution . This file contains vector of points of the generated path.");
    if (!initial_solution.empty()) {
        std::cout << "First Viable Solution Obtained after " << rrtstar->getCurrentIterations() << " iterations" << std::endl;
        std::cout << "Cost is " << rrtstar->lastnode->cost << std::endl;
        std::cout << "Saving the generated plan (vector of points)" << std::endl;
    }
    std::vector<Point> optimized_solution;
    //search for the optimized paths
    while (rrtstar->getCurrentIterations() < rrtstar->getMaxIterations() && !initial_solution.empty())
    {
        std::cout << "=========================================================================" << std::endl;
        std::cout << "The algorithm continues iterating on the current plan to improve the plan" << std::endl;
        optimized_solution = rrtstar->planner();
        std::cout << "More optimal solution has obtained after " << rrtstar->getCurrentIterations() << " iterations" << std::endl;
        std::cout << "Cost is " << rrtstar->m_cost_bestpath << std::endl;
    }
    //save optimized solution
    // "Mfiles//Path_after_MAX_ITER.txt"
    rrtstar->savePlanToFile(optimized_solution, OPTIMIZE_PATH_FILE, " Optimized Solution after maximum provided iteration.This file contains vector of points of the generated path.");
    if (!optimized_solution.empty()) {
        std::cout << "Exceeded max iterations!" << std::endl;
        std::cout << "Saving the generated plan (vector of points)" << std::endl;
    }
   
*/


    /* ------------------------------------------------------

            Rapidly-Random Exploration Tree

    --------------------------------------------------------*/

    std::cout<<"Start executing RRT algorithm...\n";
    std::vector<Point> Available_Points = rrtstar->RRT_Explore(K);

    // Save "Avalible point! .txt"
    rrtstar->savePlanToFile(Available_Points, AVAILABLE_PATH_FILE, "Saved a vector of reachable workspace points.");


    // rrtstar->savePlanToFile(rrtstar->get_available_points(), AVAILABLE_PATH_FILE, "Saved a vector of reachable workspace points.");


    #ifdef TIME 
        endtime = omp_get_wtime();
        total_time = endtime - starttime;
        printf("Execution time: %.6f seconds\n",total_time);
    #endif

    //free up the memory
    delete rrtstar;
}
