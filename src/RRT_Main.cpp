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

// 0107:

// 1. Random points: a probability : > : x_rand = x_goal else x_rand = random()
// 2. Implement Parallel methods
// 3. See PRM ?
// 4. Bi-direction RRT
// 5. Goal-based RRT
// 6. PRT



/* -----------------------  Description of RRT* algorithm: --------------------- * 
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
*   ---------------------------------------------------------------------------- */


#include"RRTstar.h"
#include"omp.h"
#include <string> // string concatination.
#include <cstdio>
#define TIME


/* -------------- Functions used ------------------ */
void Initialize_Environment_0(RRTSTAR* rrtstar, int K, std::string OBSTACLES_FILE);
void Initialize_Environment_4(RRTSTAR* rrtstar, int K, std::string OBSTACLES_FILE);
void Initialize_Environment_12(RRTSTAR* rrtstar, int K, std::string OBSTACLES_FILE);
void Initialize_Environment_Maze(RRTSTAR* rrtstar, int K, std::string OBSTACLES_FILE);

std::vector<Point> Generate_First_Path(RRTSTAR* rrtstar, std::string FIRST_PATH_FILE);
void Generate_Optimized_Path(RRTSTAR* rrtstar, std::string OPTIMIZE_PATH_FILE, std::vector<Point> initial_solution);



/* ---------- Global Variables (Parameters) --------------- */

// const int K = 1000; // 1000 points.

// World size
/* 
    3 Size here:
    3000 * 3000
    1500 * 1500
    1000 * 1000
     500 *  500

    3 Type of Obstacles;
    0 Obstacle
    4 Obstacles
    12 Obstacles
    MAZE structure.

*/

enum {
    FREE_SPACE, 
    FOUR_OBS, 
    TWELVE_OBS, 
    MAZE
};


int ENV_TYPE;
int WORLD_WIDTH, WORLD_HEIGHT;

// Obstacle file name
// std::string OBSTACLES_FILE = "Mfiles//Obstacles_500_4.txt";
// std::string OBSTACLES_FILE = "Mfiles//Obstacles_1500_maze.txt";
// std::string OBSTACLES_FILE = "Mfiles//Obstacles_500_0.txt";

float rrt_radius = 25;  // findNearNeighbors :=> raduis for RRT* algorithm (Within a radius of r, RRT* will find all neighbour nodes of a new node).
                        // This radius is for re-wiring RRT tree (re-assign parent node to minimize path cost (optimizing paths))
float end_thresh = 10;  // goal_threshold :=> the radius to check if the last node in the tree is close to the end position



int main(int argc, char** argv)
{

    #ifdef TIME 
        float total_time = 0;
        float starttime, endtime;
        starttime = omp_get_wtime();
    #endif

    if(argc != 5){
        std::cout<<"Error format, :=> ./RRT < K > < version > < World Size > < World Type >\n" ;
        return 0;
    }

    std::string world_width;
    /* ----------- Read input from user -------- */
    // std::cout<<"Enter World Size (500 / 1500 / 3000) :\n";
    // std::cin>>world_width;
    // WORLD_HEIGHT = WORLD_WIDTH = atoi(world_width.c_str());
    
    // std::cin>>world_width;
    WORLD_HEIGHT = WORLD_WIDTH = atoi(argv[3]);
    // std::cout<<"World size: "<<WORLD_WIDTH<<"\n";


    // int input_type;
    // std::cout<<"Enter World Type\n";
    // std::cout<<"0 -> Free space\n";  
    // std::cout<<"1 -> Four Obstacles\n";   
    // std::cout<<"2 -> Twelve Obstacles\n";  
    // std::cout<<"3 -> Maze\n";  
    // std::cin>>input_type;
    // ENV_TYPE = input_type;
    ENV_TYPE = atoi(argv[4]);
    // std::cout<<"You enter: "<<ENV_TYPE<<std::endl;
    // Read user input
    int K = atoi(argv[1]);
    std::string version = argv[2];

    std::cout<<"World size: "<<WORLD_WIDTH<<"\n";
    std::cout<<"World type: "<<ENV_TYPE<<"\n";
    std::cout<<"Version: "<<version<<"\n";
    std::cout<<"K: "<<K<<"\n";


	cpu_set_t cpuset;
	sched_getaffinity(0, sizeof(cpuset), &cpuset);
	unsigned long long ncpus = CPU_COUNT(&cpuset);
    std::cout<<"Number of cpus: "<<ncpus <<"\n";

    std::string FIRST_PATH_FILE = "Mfiles//FirstPath/first_path_"+std::to_string(int(WORLD_WIDTH))+"_"+ std::to_string(ENV_TYPE) + "_v"+ version + ".txt";
    std::string OPTIMIZE_PATH_FILE =  "Mfiles//OptPath/opt_path_"+std::to_string(int(WORLD_WIDTH))+"_"+ std::to_string(ENV_TYPE) + "_v"+version + ".txt";
    std::string AVAILABLE_PATH_FILE =  "Mfiles//AvailablePts/avail_pts_"+std::to_string(int(WORLD_WIDTH))+"_"+ std::to_string(ENV_TYPE) + "_v"+version + ".txt";
    std::string NODES_PATH_FILE = "Mfiles//Nodes/nodes_pts_"+std::to_string(int(WORLD_WIDTH))+"_"+ std::to_string(ENV_TYPE) + "_v" + version + ".txt";
    std::string OBSTACLES_FILE = "Mfiles//Obstacles/Obstacles_"+std::to_string(int(WORLD_WIDTH))+"_" + std::to_string(ENV_TYPE)+".txt";

    int start_x, start_y, goal_x, goal_y;
    /* ---------- Setting Start & Goal location ------------- */
    switch(ENV_TYPE){
        case FREE_SPACE:
            std::cout<<"choose FREE_SPACE\n";          
            start_x = WORLD_WIDTH/2;
            start_y = WORLD_HEIGHT/2;
            goal_x = 10;
            goal_y = WORLD_HEIGHT-10;
        break;
        case FOUR_OBS: // Start from left top to right down
            std::cout<<"choose FOUR_OBS\n";          
            start_x = 25;
            start_y = WORLD_HEIGHT-25; 
            goal_x = WORLD_WIDTH-25;
            goal_y = 25;
        break; 
        case TWELVE_OBS:
            std::cout<<"choose TWELVE_OBS\n";           
            start_x = 25;
            start_y = WORLD_HEIGHT-25; 
            goal_x = WORLD_WIDTH-25;
            goal_y = 25;
        break;
        case MAZE:
            std::cout<<"choose Maze\n";         
            start_x = WORLD_WIDTH-10;
            start_y = WORLD_HEIGHT-10; 
            goal_x = 10;
            goal_y = 10;
        break;
        default:
            start_x = WORLD_WIDTH/2;
            start_y = WORLD_HEIGHT/2;
            goal_x = 10;
            goal_y = WORLD_HEIGHT-10;
        break;
    }

    Point start_pos(start_x, start_y);    
    Point end_pos(goal_y, goal_y);

    /* --------- Instantiate RRTSTAR class ------------- */
    RRTSTAR* rrtstar = new RRTSTAR(start_pos,end_pos, rrt_radius, end_thresh);
    
    /* ---------- Initialize Environment ------------- */
    switch(ENV_TYPE){
        case FREE_SPACE:
            Initialize_Environment_0(rrtstar, K, OBSTACLES_FILE);
        break;
        case FOUR_OBS:
            Initialize_Environment_4(rrtstar, K, OBSTACLES_FILE);
        break; 
        case TWELVE_OBS:
            Initialize_Environment_12(rrtstar, K, OBSTACLES_FILE);
        break;
        case MAZE:
            Initialize_Environment_Maze(rrtstar, K, OBSTACLES_FILE);
        break;
        default:
            Initialize_Environment_4(rrtstar, K, OBSTACLES_FILE);
        break;

    }
   

    std::cout << "Starting RRT* Algorithm..." << std::endl;
    
    /* --------- First Path ------------- */
    std::vector<Point> initial_solution = Generate_First_Path(rrtstar, FIRST_PATH_FILE);
    /* --------- Optimize Path ------------- */
    Generate_Optimized_Path(rrtstar, OPTIMIZE_PATH_FILE, initial_solution);
    /* --------- Save RRT Tree(nodes) ------------- */
    rrtstar->savePlanToFile(rrtstar->get_nodes_points(), NODES_PATH_FILE, "Saved a vector of rrt <nodes> points.");

    #ifdef TIME 
        endtime = omp_get_wtime();
        total_time = endtime - starttime;
        printf("\n\nExecution time: %.6f seconds\n",total_time);
        std::cout<<"\n --------------------------------------------- End------------------------------------------------------\n\n"<<std::endl;
    #endif

    //free up the memory
    delete rrtstar;
}


// 0 Obstacle
void Initialize_Environment_0(RRTSTAR* rrtstar, int K, std::string OBSTACLES_FILE){
    rrtstar->world->setWorldWidth(WORLD_WIDTH);
    rrtstar->world->setWorldHeight(WORLD_HEIGHT);

    // set step size and max iterations. If the values are not set, the default values are max_iter=5000 and step_size=10.0
    // rrtstar->setMaxIterations(10000);
    rrtstar->setMaxIterations(K);
    // rrtstar->setStepSize(10.0);
    rrtstar->setStepSize(10.0);

    //Create obstacles

    //Save obstacles to  file;
    rrtstar->world->saveObsToFile(OBSTACLES_FILE);
}

// 4 Obstacles
void Initialize_Environment_4(RRTSTAR* rrtstar, int K, std::string OBSTACLES_FILE){
    rrtstar->world->setWorldWidth(WORLD_WIDTH);
    rrtstar->world->setWorldHeight(WORLD_HEIGHT);

    // set step size and max iterations. If the values are not set, the default values are max_iter=5000 and step_size=10.0
    // rrtstar->setMaxIterations(10000);
    rrtstar->setMaxIterations(K);
    // rrtstar->setStepSize(10.0);
    rrtstar->setStepSize(10.0);

    //Create obstacles

    // TODO : read a .txt file
    //        and automatically set obstacles!
    int ob_1_x, ob_1_y, ob_1_width, ob_1_height;
    int ob_2_x, ob_2_y, ob_2_width, ob_2_height;
    int ob_3_x, ob_3_y, ob_3_width, ob_3_height;
    int ob_4_x, ob_4_y, ob_4_width, ob_4_height;

    ob_1_x = 0;
    ob_1_y = WORLD_HEIGHT-300;
    ob_1_width = WORLD_WIDTH/3;
    ob_1_height = 100;


    ob_2_x =  WORLD_WIDTH/2;
    ob_2_y = WORLD_HEIGHT/2;
    ob_2_width = WORLD_HEIGHT/6;
    ob_2_height = WORLD_HEIGHT/6;


    ob_3_x = WORLD_WIDTH*2/3;
    ob_3_y = 350;
    ob_3_width = WORLD_WIDTH/3;
    ob_3_height = 200;


    ob_4_x = WORLD_WIDTH/5;;
    ob_4_y = WORLD_HEIGHT/5;
    ob_4_width = WORLD_WIDTH/8;
    ob_4_height = WORLD_HEIGHT/4;


    //Obstacle 1
    Point ob1_1( ob_1_x,  ob_1_y); //position of the top left point of obstacle 1
    Point ob1_2(ob_1_x+ob_1_width,  ob_1_y+ob_1_height); //position of the bottom right point of obstacle 1
    rrtstar->world->addObstacle(ob1_1, ob1_2);//create obstacle 1
    //Obstacle 2;
    Point ob2_1(ob_2_x,  ob_2_y); //position of the top left point of obstacle 2
    Point ob2_2(ob_2_x+ob_2_width,  ob_2_y+ob_2_height); //position of the bottom right point of obstacle 2
    rrtstar->world->addObstacle(ob2_1, ob2_2);//create obstacle 2

    //Obstacle 3
    Point ob3_1(ob_3_x,  ob_3_y); //position of the top left point of obstacle 3
    Point ob3_2(ob_3_x+ob_3_width,  ob_3_y+ob_3_height); //position of the bottom right point of obstacle 3
    rrtstar->world->addObstacle(ob3_1, ob3_2);//create obstacle 3
    //Obstacle 4;
    Point ob4_1(ob_4_x,  ob_4_y); //position of the top left point of obstacle 4
    Point ob4_2(ob_4_x+ob_4_width,  ob_4_y+ob_4_height); //position of the bottom right point of obstacle 4

    rrtstar->world->addObstacle(ob4_1, ob4_2);//create obstacle 4
    //Save obstacles to  file;
    rrtstar->world->saveObsToFile(OBSTACLES_FILE);
}

// 12 Obstacles
void Initialize_Environment_12(RRTSTAR* rrtstar, int K, std::string OBSTACLES_FILE){
    rrtstar->world->setWorldWidth(WORLD_WIDTH);
    rrtstar->world->setWorldHeight(WORLD_HEIGHT);

    // set step size and max iterations. If the values are not set, the default values are max_iter=5000 and step_size=10.0
    // rrtstar->setMaxIterations(10000);
    rrtstar->setMaxIterations(K);
    // rrtstar->setStepSize(10.0);
    rrtstar->setStepSize(10.0);

    //Create obstacles
    int left_top_x, left_top_y;
    int box_width = 30;
    int box_height = 30;
    // Generate 12 (3*4) cubes
    for(int row=0; row<3; row++){
        for(int col=0; col<4; col++){
            left_top_x = col*(WORLD_WIDTH/4);
            left_top_y = row*(WORLD_HEIGHT/3);
            Point left_top(left_top_x, left_top_y);
            Point right_bottom(left_top_x+box_width, left_top_y+box_height);
            rrtstar->world->addObstacle(left_top, right_bottom);
        }
    }
    rrtstar->world->saveObsToFile(OBSTACLES_FILE);
}

// Maze
void Initialize_Environment_Maze(RRTSTAR* rrtstar, int K, std::string OBSTACLES_FILE){
    rrtstar->world->setWorldWidth(WORLD_WIDTH);
    rrtstar->world->setWorldHeight(WORLD_HEIGHT);

    // set step size and max iterations. If the values are not set, the default values are max_iter=5000 and step_size=10.0
    // rrtstar->setMaxIterations(10000);
    rrtstar->setMaxIterations(K);
    // rrtstar->setStepSize(10.0);
    rrtstar->setStepSize(10.0);

    // create a Maze
    //Create obstacles
    int left_top_x, left_top_y;
    int box_width = 50;
    int box_height = 50;
    // Generate 12 (3*4) cubes
    int num_of_col = WORLD_WIDTH/(2*box_width);
    int num_of_row = WORLD_HEIGHT/(2*box_height);

    for(int row=0; row<num_of_row; row++){
        for(int col=0; col< num_of_col; col++){
            // if(col==0 && row==0) continue;
            left_top_x = (col)*(2*box_width)+box_width/2;
            left_top_y = (row)*(2*box_height)+box_height/2;
            Point left_top(left_top_x, left_top_y);
            Point right_bottom(left_top_x+box_width, left_top_y+box_height);
            rrtstar->world->addObstacle(left_top, right_bottom);
        }
    }

    //Save obstacles to  file;
    rrtstar->world->saveObsToFile(OBSTACLES_FILE);
}




//search for the first viable solution
std::vector<Point> Generate_First_Path(RRTSTAR* rrtstar, std::string FIRST_PATH_FILE){

    /* --------- First Path ------------- */
    //search for the first viable solution
    std::vector<Point> initial_solution =rrtstar->planner();
    rrtstar->savePlanToFile(initial_solution, FIRST_PATH_FILE, "First viable solution . This file contains vector of points of the generated path.");
    if (!initial_solution.empty()) {
        std::cout << "First Viable Solution Obtained after " << rrtstar->getCurrentIterations() << " iterations" << std::endl;
        std::cout << "Cost is " << rrtstar->lastnode->cost << std::endl;
        std::cout << "Saving the generated plan (vector of points)" << std::endl;
    }

    return initial_solution;

}

void Generate_Optimized_Path(RRTSTAR* rrtstar, std::string OPTIMIZE_PATH_FILE, std::vector<Point> initial_solution){

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

}


    // NOT USED  BELOW

    /* ------------------------------------------------------

            Rapidly-Random Exploration Tree

    --------------------------------------------------------*/

    // std::cout<<"Start executing RRT algorithm...\n";
    // std::vector<Point> Available_Points = rrt->RRT_Explore(K);

    // Save "Avalible point! .txt"
    // rrt->savePlanToFile(Available_Points, AVAILABLE_PATH_FILE, "Saved a vector of reachable workspace points.");
    // rrtstar->savePlanToFile(rrtstar->get_available_points(), AVAILABLE_PATH_FILE, "Saved a vector of reachable workspace points.");
