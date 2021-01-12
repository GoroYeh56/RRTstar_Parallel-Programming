/**
 *  This file contains classes and methods to implement RRT* algorithm
 */


#include"RRTstar.h"

#include "pthread.h"

#include "omp.h"

#define TIME

// IO time.
// Comm time (reduce & pthread_join)
// Computation time (in main.)

#define PARALLEL  // if define parallel, accelerate! using OpenMP or MPI?
// #define DEBUG_FINDNEAREST
// #define PARALLEL_FINE_PARENT

// todo: try lower epsilon here.
const float EPSILON = 0.9; // epsilon-greedy for faster convergence

Node::Node() {
    parent= nullptr;
    cost=0;
}

Node::~Node(){

}

//RRTSTAR class Constructors

RRTSTAR::RRTSTAR(Point start_pos, Point end_pos, float radius, float end_thresh) {
    //set the default values and set the fist node as the staring point
    world = new World;
    startPoint = start_pos;
    destination = end_pos;
    root = new Node;
    root->parent = nullptr;
    root->position = startPoint;
    root->cost = 0.0;
    lastnode = root;
    nodes.push_back(root);
    Available_Points.push_back(root->position);

    m_step_size = 10;
    m_max_iter = 5000;
    m_rrstar_radius = radius;
    m_destination_threshhold = end_thresh;
    m_num_itr = 0;
    m_cost_bestpath = 0;

    this->IO_time = 0;
    this->IO_starttime;
    this->IO_endtime  = 0;
    this->Comm_time = 0;
    this->Compute_time = 0;


    #ifdef PARALLEL
    std::cout<<"\n===== Parallel version RRT*! =====\n\n";
    #endif

}

//Destructor
RRTSTAR::~RRTSTAR()
{
    delete world;
    deleteNodes(root);
}

//RRTSTAR methods

/* ---------------------
    RRT algorithm here.
    No use, since found that RRT* already implement!
    We just take RRTSTAR->nodes' position! vector of <Point>
----------------------- */

std::vector<Point> RRTSTAR::RRT_Explore(int K) {

    // 1. K iterations (sample K points)
    #ifdef PARALLEL
    // #pragma omp parallel for
    #endif  
    for (int iter=0; iter < K; iter++){
        this->m_num_itr++;
        // 2. random sample a Point.
        Node plan_n_rand = this->getRandomNode();
        while(plan_n_rand.position.m_x == 0 || plan_n_rand.position.m_y==0){
            Node plan_n_rand = this->getRandomNode();
        } 
        // 3. Find the nearest neighbor
        Node* plan_n_nearest = this->findNearest(plan_n_rand.position);  //Find the closest node to the new random node.
        // 4.  Steer from this n_neighbor to the n_rand     
        Point plan_p_new = this->steer(plan_n_rand, plan_n_nearest); //Steer from "N_Nearest" towards "N_rand": interpolate if node is too far away.
        
        // 6. If the new node didn't hit obstacles, append to the graph.
        //    Note: Check Obstacle here!   
        if (!this->world->checkObstacle(plan_p_new, plan_n_nearest->position)){            
            Node* plan_n_new = new Node; //create new node to store the position of the steered node.
            plan_n_new->position = plan_p_new; //create new node from the streered new point
            this->insertNode(plan_n_nearest, plan_n_new);//Add N_new to node list.
        }
    }
    // 7. return the graph (A vector of Points)
    return this->get_nodes_points();
        
}


std::vector<Point> RRTSTAR::get_nodes_points(){
    std::vector<Point> nodes_points;
    // Accelerate I/O here. 
    for (size_t i = 0; i < this->nodes.size(); i++) { // It goes from a node near destination to the root
        nodes_points.push_back(this->nodes[i]->position);
    }
    return nodes_points;
}



/* ---------- Pthread Version ------------
    1. When finding nearest neightbors, partition nodes into 4 ranges. (4 threads)
    2. Each thread find the nearest neighbor.
    3. Reduce the the minimum thread.

-----------------------------------------*/

const int num_threads = 4;
const int DIST_MAX = 100000;

typedef struct thread_info{
    /*  Basic components */
	int thread_id;
    int start_index;    // each thread search from RRT*->node start_index -> end index
    int end_index;
    RRTSTAR* rrtstar;
    /*  For 1. find the Nearest point */
    Point new_point;
    /*  For 2. find neighbors */
    int radius; 
    /* For 3. findParent_thread */
    Node* n_new;
    Node* n_nearest;
    std::vector<Node*> v_n_near;
}thread_info;

pthread_t threads[num_threads]; // a threads_array. ->pthreads
thread_info thread_INFO[num_threads];  // arguments for each thread.    

Node* nearest_node_threads[num_threads]; // return from each P thread.
// 1. findNearest
// need 'new point' information
void* findNearest_thread(void* my_thread){
	
	thread_info* thread_t = (thread_info*)my_thread;
    int id =  thread_t->thread_id;
    int start = thread_t->start_index;
    int end = thread_t->end_index;
    Point new_point = thread_t->new_point;
    RRTSTAR* rrtstar = thread_t->rrtstar;

    float fn_minDist = FLT_MAX;//set the minimum distance to the maximum number possible
    Node* fn_closest = NULL;
    int min_index;
    // from start to end
    for (size_t i=start; i < end; i++) { //iterate through all nodes of the tree to find the closest to the new node
    // for (size_t i=0; i < this->nodes.size(); i++) { //iterate through all nodes of the tree to find the closest to the new node

        float fn_dist = rrtstar->distance(new_point, rrtstar->nodes[i]->position);
        
        if (fn_dist < fn_minDist) {
            fn_minDist = fn_dist;
            fn_closest = rrtstar->nodes[i];
            fn_closest->cur_dist = fn_dist;
            #if (defined DEBUG_FINDNEAREST && defined PARALLEL)
            printf("t%d pick index: %d, closest_point(%f,%f). Orig_cost:%f, fn_dist:%f\n",id, i, rrtstar->nodes[i]->position.m_x, rrtstar->nodes[i]->position.m_y, fn_minDist, fn_dist);
            #endif
            min_index = i;
        }
    }
    nearest_node_threads[id] = fn_closest;
	pthread_exit(NULL);
}

std::vector<Node*> near_neighbors[num_threads]; 
// 2. findNearest_neighbors
void* findNearest_neighbors_thread(void* my_thread){
	
	thread_info* thread_t = (thread_info*)my_thread;
    int id =  thread_t->thread_id;
    int start = thread_t->start_index;
    int end = thread_t->end_index;
    int radius = thread_t->radius;
    Point new_point = thread_t->new_point;
    RRTSTAR* rrtstar = thread_t->rrtstar;

    for (size_t i=start; i < end; i++) { //iterate through all nodes to see which ones fall inside the circle with the given radius.
        if (rrtstar->distance(new_point, rrtstar->nodes[i]->position) < radius) {
            near_neighbors[id].push_back(rrtstar->nodes[i]);
        }
    }
	pthread_exit(NULL);
}


// 3. findParent
// input: nearest neighbors
// ourput: new parent (Node*) for the new_node
Node* new_Parent_threads[num_threads]; // return from each P thread.
void* findParent_thread(void* my_thread) {

	thread_info* thread_t = (thread_info*)my_thread;
    int id =  thread_t->thread_id;
    int start = thread_t->start_index;
    int end = thread_t->end_index;
    int radius = thread_t->radius;
    Point new_point = thread_t->new_point;
    RRTSTAR* rrtstar = thread_t->rrtstar;

    Node* n_new = thread_t->n_new;
    Node* fp_n_parent = thread_t->n_nearest;
    std::vector<Node*> v_n_near = thread_t->v_n_near;   
 
    float fp_cmin = rrtstar->getCost(fp_n_parent) + rrtstar->pathCost(fp_n_parent, n_new); // Update cost of reaching "N_new" from "N_Nearest"

    for (size_t j = start; j < end; j++) { //In all members of "N_near", check if "N_new" can be reached from a different parent node with cost lower than Cmin, and without colliding with the obstacle.
        Node* fp_n_near = v_n_near[j];
        if (!rrtstar->world->checkObstacle(fp_n_near->position, n_new->position) &&
            (rrtstar->getCost(fp_n_near) + rrtstar->pathCost(fp_n_near, n_new)) < fp_cmin) {
            fp_n_parent = fp_n_near; // a near node with minimun cost of path
            fp_cmin = rrtstar->getCost(fp_n_near) + rrtstar->pathCost(fp_n_near, n_new); //update the cost of path
            fp_n_parent->cur_dist =  fp_cmin;

        }
    }
    new_Parent_threads[id] = fp_n_parent ;
	pthread_exit(NULL);
}







std::vector<Point> RRTSTAR::planner_pthread() {
    // while iter < MAX_Iterations
    while (this->m_num_itr<this->m_max_iter)
    {
        #ifdef DEBUG_FINDNEAREST
        std::cout<<"Iteration: "<<this->m_num_itr<<std::endl;
        #endif

        this->m_num_itr++;
        // Node plan_n_rand = this->getRandomNode(); //Pick a random node
        Node plan_n_rand = this->RandomNode_Epsilon();
        if (plan_n_rand.position.m_x!=0 && plan_n_rand.position.m_y!=0) {

        
        Node* plan_n_nearest;

        /* Remains the same in one iteration */
        int num_of_nodes = this->nodes.size();
        int nodes_per_thread;
        int remainder;       
        
        
        if(num_of_nodes < num_threads){
            plan_n_nearest = this->findNearest(plan_n_rand.position);  
        }
        else{
        /* --------- Pthread  -----------*/ 
            #ifdef DEBUG_FINDNEAREST
            std::cout<<"Creating threads..."<<std::endl;
            #endif
            nodes_per_thread = num_of_nodes/num_threads;
            remainder = num_of_nodes%num_threads;
            for (int t = 0; t < num_threads; t++) {
                thread_INFO[t].thread_id = t;
                // std::cout<<" thread_INFO["<<t<<"] id: "<<t<<std::endl;
                thread_INFO[t].new_point = plan_n_rand.position;
                // std::cout<<" thread_INFO["<<t<<"] . new_point: "<<plan_n_rand.position<<std::endl;
                thread_INFO[t].rrtstar = this;
                // std::cout<<" thread_INFO[].rrtstar"<<std::endl;
                // printf("thread %d, r: %llu, k:%llu, num_threads:%llu\n",thread_INFO[t].thread_id, thread_INFO[t].r, thread_INFO[t].k,thread_INFO[t].num_threads);
                if(t==0){
                    thread_INFO[t].start_index = 0;
                    thread_INFO[t].end_index = 0 + nodes_per_thread + remainder;
                }else{
                    thread_INFO[t].start_index = t*nodes_per_thread + remainder;
                    thread_INFO[t].end_index = (t*nodes_per_thread + remainder) + nodes_per_thread;
                }
                #ifdef DEBUG_FINDNEAREST
                std::cout<<"t"<<t<<" pick from "<<thread_INFO[t].start_index<<" to "<<thread_INFO[t].end_index<<", total "<<num_of_nodes<<"nodes\n";
                #endif
                // printf("thread %d, r: %llu, k:%llu, num_op:%llu\n",thread_INFO[t].thread_id, thread_INFO[t].r, thread_INFO[t].k,thread_INFO[t].num_op);
                // printf("In main: creating thread %d\n", t);
                int rc = pthread_create(&threads[t], NULL, findNearest_thread, (void*)&thread_INFO[t]); 
                // rc = pthread_create(&threads[t], NULL, hello, (void*)&ID[t]); // ID would be the argument passesd to function "hello(threadid)"
                if (rc) {
                    printf("ERROR; return code from pthread_create() is %d\n", rc);
                    exit(-1);
                }
            }

            // join pthread.
            // Add Barrier 
            float Comm_starttime =  omp_get_wtime();

            for(int i=0; i<num_threads; i++) pthread_join(threads[i],NULL);
            // Reduce on the nearest neighbor.
            
            for(int i=0; i<num_threads; i++){
                int min_dist = DIST_MAX;
                if(nearest_node_threads[i]->cur_dist < min_dist)
                    plan_n_nearest = nearest_node_threads[i];
            }

            float Comm_endtime =  omp_get_wtime();
            this->Comm_time += Comm_endtime - Comm_starttime;
           
        }


            // Node* plan_n_nearest = this->findNearest(plan_n_rand.position);  
            // std::cout<<"\n Nearest neighbor: ("<<plan_n_nearest->position.m_x <<", "<<plan_n_nearest->position.m_y <<") \n";
            Point plan_p_new = this->steer(plan_n_rand, plan_n_nearest); //Steer from "N_Nearest" towards "N_rand": interpolate if node is too far away.
            if (!this->world->checkObstacle(plan_p_new, plan_n_nearest->position)) { // Check if an obstacle is between new node and nearest nod.
                    Node* plan_n_new = new Node; //create new node to store the position of the steered node.
                    plan_n_new->position = plan_p_new; //create new node from the streered new point
                    
                    
                    
                    /*--------------- Create 4 buffers and Reduce! (Merge from four threas) ----------- */
                 
                    std::vector<Node*> plan_v_n_near; //create a vector for neighbor nodes
                    // int num_of_nodes = this->nodes.size(); remains the same!
                    if(num_of_nodes < num_threads){
                        this->findNearNeighbors(plan_n_new->position, this->m_rrstar_radius, plan_v_n_near);
                    }
                    else{
                   
                        #ifdef DEBUG_FINDNEAREST
                        std::cout<<"Creating threads..."<<std::endl;
                        #endif
                        for (int t = 0; t < num_threads; t++) {
                            thread_INFO[t].thread_id = t;
                            thread_INFO[t].new_point = plan_n_new->position;
                            thread_INFO[t].rrtstar = this;
                            thread_INFO[t].radius = this->m_rrstar_radius;
                          
                            if(t==0){
                                thread_INFO[t].start_index = 0;
                                thread_INFO[t].end_index = 0 + nodes_per_thread + remainder;
                            }else{
                                thread_INFO[t].start_index = t*nodes_per_thread + remainder;
                                thread_INFO[t].end_index = (t*nodes_per_thread + remainder) + nodes_per_thread;
                            }
                            #ifdef DEBUG_FINDNEAREST
                            std::cout<<"t"<<t<<" pick from "<<thread_INFO[t].start_index<<" to "<<thread_INFO[t].end_index<<", total "<<num_of_nodes<<"nodes\n";
                            #endif
                            int rc = pthread_create(&threads[t], NULL, findNearest_neighbors_thread, (void*)&thread_INFO[t]); 
                            if (rc) {
                                printf("ERROR; return code from pthread_create() is %d\n", rc);
                                exit(-1);
                            }
                        }

                        // join pthread. : near_neighbors[id]
                        // Add Barrier 

                        float Comm_starttime =  omp_get_wtime();
                        for(int i=0; i<num_threads; i++) pthread_join(threads[i],NULL);
                        // Merge the nearest neighbors  
                        for(int i=0; i<num_threads; i++){
                            plan_v_n_near.insert(plan_v_n_near.end(),near_neighbors[i].begin(),near_neighbors[i].end());
                            // for(int j=0; j< near_neighbors[i].size(); j++)
                            //     plan_v_n_near.push_back(near_neighbors[i][j]);
                        }
                        float Comm_endtime =  omp_get_wtime();
                        this->Comm_time += Comm_endtime - Comm_starttime;
                    
                    }
                                    
                    
                    // this->findNearNeighbors(plan_n_new->position, this->m_rrstar_radius, plan_v_n_near); // Find nearest neighbors with a given radius from new node.
                    
                    // ESSENCE of RRT* 1 :=> Re-assign edges (re assign the parent of a node)
                    // Node* plan_n_parent=this->findParent(plan_v_n_near,plan_n_nearest,plan_n_new); //Find the parent of the given node (the node that is near and has the lowest path cost)
                    
                     /* -------------Re selecting parents ----------*/                   
                    Node* plan_n_parent;
                    #ifndef PARALLEL_FINE_PARENT
                    plan_n_parent=this->findParent(plan_v_n_near,plan_n_nearest,plan_n_new); 
                    #endif
                    #ifdef PARALLEL_FIND_PARENT
                    int num_of_neighbors = plan_v_n_near.size();
                    nodes_per_thread = num_of_neighbors/num_threads;
                    remainder = num_of_neighbors%num_threads;
                    if(num_of_neighbors < num_threads){
                        plan_n_parent=this->findParent(plan_v_n_near,plan_n_nearest,plan_n_new); 
                    }
                    else{

                        #ifdef DEBUG_FINDNEAREST
                        std::cout<<"Creating threads..."<<std::endl;
                        #endif


                        for (int t = 0; t < num_threads; t++) {
                            thread_INFO[t].thread_id = t;
                            thread_INFO[t].new_point = plan_n_new->position;
                            thread_INFO[t].rrtstar = this;
                            thread_INFO[t].n_new = plan_n_new;
                            thread_INFO[t].n_nearest = plan_n_nearest;
                            thread_INFO[t].v_n_near = plan_v_n_near;

                            if(t==0){
                                thread_INFO[t].start_index = 0;
                                thread_INFO[t].end_index = 0 + nodes_per_thread + remainder;
                            }else{
                                thread_INFO[t].start_index = t*nodes_per_thread + remainder;
                                thread_INFO[t].end_index = (t*nodes_per_thread + remainder) + nodes_per_thread;
                            }
                            #ifdef DEBUG_FINDNEAREST
                            std::cout<<"t"<<t<<" pick from "<<thread_INFO[t].start_index<<" to "<<thread_INFO[t].end_index<<", total "<<num_of_nodes<<"nodes\n";
                            #endif
                            int rc = pthread_create(&threads[t], NULL, findParent_thread , (void*)&thread_INFO[t]); 
                            if (rc) {
                                printf("ERROR; return code from pthread_create() is %d\n", rc);
                                exit(-1);
                            }
                        }

                        // join pthread. : plan_n_parent
                        // Add Barrier 
                        for(int i=0; i<num_threads; i++) pthread_join(threads[i],NULL);
                        // Reduce on plan_n_parent.
                        for(int i=0; i<num_threads; i++){
                            int min_dist = DIST_MAX;
                            if( new_Parent_threads[i] ->cur_dist < min_dist)
                                plan_n_parent = new_Parent_threads[i];
                        }
                
                    }
                    #endif
                    
                    #ifdef DEBUG_P
                        std::cout<<"New parent: ("<<plan_n_parent->position.m_x << ", "<<plan_n_parent->position.m_y<<") \n";
                    #endif

                    // Re-allocate this edge (from parent->child(new_node))
                    this->insertNode(plan_n_parent, plan_n_new);//Add N_new to node list.
                    
                    // ESSENCE of RRT* 2 :=> Re-wire the RRT Tree!
                    this->reWire(plan_n_new, plan_v_n_near); //rewire the tree

                    if (this->reached() && this->bestpath.empty()) { //find the first viable path
                        return this->generatePlan(this->lastnode);
                    }

                    if (!this->bestpath.empty()) { //find more optimal paths
                        if (this->reached()) {
                            // If we get a better path (lower cost)!
                            if (this->lastnode->cost < this->m_cost_bestpath) {
                                return this->generatePlan(this->lastnode);
                            }
                        }
                        else {
                            // Havent reach the goal, ??? Why here.
                            // return the nearest point to the destination(goal)
                            Node* Plan_NearNodeEnd = this->findNearest(this->destination);
                            if (Plan_NearNodeEnd->cost < this->m_cost_bestpath) {
                                return this->generatePlan(Plan_NearNodeEnd);
                            }
                        }
                    }
                    
            }            
        }
    }   // End while loop.
    
    if (this->bestpath.empty()) {
        // if not reached yet, no solution has found
        std::cout << "Exceeded max iterations!" << std::endl;
        std::cout << "Error: No solution found" << std::endl;
        this->savePlanToFile({}, "Mfiles//Path_after_MAX_ITER.txt", "Error: No solution found");
        this->savePlanToFile({}, "Mfiles//first_viable_path.txt", "Error: No solution found");
        return {};
    }
    else {
        return RRTSTAR::planFromBestPath(); //after reaching the maximum iteration number retun the path that has the lowest cost.
    }
        
}


std::vector<Point> RRTSTAR::planner() {
    // while iter < MAX_Iterations
    while (this->m_num_itr<this->m_max_iter)
    {
        #ifdef DEBUG_FINDNEAREST
        std::cout<<"Iteration: "<<this->m_num_itr<<std::endl;
        #endif
        

        this->m_num_itr++;
        // Node plan_n_rand = this->getRandomNode(); //Pick a random node
        Node plan_n_rand = this->RandomNode_Epsilon();
        if (plan_n_rand.position.m_x!=0 && plan_n_rand.position.m_y!=0) {
            
            Node* plan_n_nearest = this->findNearest(plan_n_rand.position);  //Find the closest node to the new random node.
            Point plan_p_new = this->steer(plan_n_rand, plan_n_nearest); //Steer from "N_Nearest" towards "N_rand": interpolate if node is too far away.
            if (!this->world->checkObstacle(plan_p_new, plan_n_nearest->position)) { // Check if an obstacle is between new node and nearest nod.
                    Node* plan_n_new = new Node; //create new node to store the position of the steered node.
                    plan_n_new->position = plan_p_new; //create new node from the streered new point
                    
                    /* ------------------ Re selecting parents ----------*/
                    std::vector<Node*> plan_v_n_near; //create a vector for neighbor nodes
                    this->findNearNeighbors(plan_n_new->position, this->m_rrstar_radius, plan_v_n_near); // Find nearest neighbors with a given radius from new node.
                    
                    // ESSENCE of RRT* 1 :=> Re-assign edges (re assign the parent of a node)
                    Node* plan_n_parent=this->findParent(plan_v_n_near,plan_n_nearest,plan_n_new); //Find the parent of the given node (the node that is near and has the lowest path cost)
                    
                    // Re-allocate this edge (from parent->child(new_node))
                    this->insertNode(plan_n_parent, plan_n_new);//Add N_new to node list.
                    
                    // ESSENCE of RRT* 2 :=> Re-wire the RRT Tree!
                    this->reWire(plan_n_new, plan_v_n_near); //rewire the tree

                    if (this->reached() && this->bestpath.empty()) { //find the first viable path
                        return this->generatePlan(this->lastnode);
                    }

                    if (!this->bestpath.empty()) { //find more optimal paths
                        if (this->reached()) {
                            // If we get a better path (lower cost)!
                            if (this->lastnode->cost < this->m_cost_bestpath) {
                                return this->generatePlan(this->lastnode);
                            }
                        }
                        else {
                            // Havent reach the goal, ??? Why here.
                            // return the nearest point to the destination(goal)
                            Node* Plan_NearNodeEnd = this->findNearest(this->destination);
                            if (Plan_NearNodeEnd->cost < this->m_cost_bestpath) {
                                return this->generatePlan(Plan_NearNodeEnd);
                            }
                        }
                    }
                    
            }            
        }
    }
    
    if (this->bestpath.empty()) {
        // if not reached yet, no solution has found
        std::cout << "Exceeded max iterations!" << std::endl;
        std::cout << "Error: No solution found" << std::endl;
        this->savePlanToFile({}, "Mfiles//Path_after_MAX_ITER.txt", "Error: No solution found");
        this->savePlanToFile({}, "Mfiles//first_viable_path.txt", "Error: No solution found");
        return {};
    }
    else {
        return RRTSTAR::planFromBestPath(); //after reaching the maximum iteration number retun the path that has the lowest cost.
    }
        
}


Node RRTSTAR::RandomNode_Epsilon() {

    float p = (float) rand()/RAND_MAX;

    std::random_device rand_rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 rand_gen(rand_rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> rand_unif(0, 1.0);  // initialize a uniform distribution between 0 and 1
    
    if(p>EPSILON){
        // return GOAL point.
        Node rand_randomnode;
        rand_randomnode.position = this->destination;
        return rand_randomnode;
    }
    else{
        Point rand_point(rand_unif(rand_gen) * this->world->getWorldWidth(), rand_unif(rand_gen) * this->world->getWorldHeight());//Generate a random point
        if (rand_point.m_x >= 0 && rand_point.m_x <= this->world->getWorldWidth() && rand_point.m_y >= 0 && rand_point.m_y <= this->world->getWorldHeight()) { //check of the generated point is inside the world!
            Node rand_randomnode;
            rand_randomnode.position = rand_point;
            return rand_randomnode;
        }
        return {};
    }


}


Node RRTSTAR::getRandomNode() {

    std::random_device rand_rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 rand_gen(rand_rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> rand_unif(0, 1.0);  // initialize a uniform distribution between 0 and 1
    
    Point rand_point(rand_unif(rand_gen) * this->world->getWorldWidth(), rand_unif(rand_gen) * this->world->getWorldHeight());//Generate a random point
    if (rand_point.m_x >= 0 && rand_point.m_x <= this->world->getWorldWidth() && rand_point.m_y >= 0 && rand_point.m_y <= this->world->getWorldHeight()) { //check of the generated point is inside the world!
        Node rand_randomnode;
        rand_randomnode.position = rand_point;
        return rand_randomnode;
    }
    return {};
}

// Return the nearest node (ONLY ONE)

// Make a map to map 
/*
    min distance 's index: => 
    {
        Node*

    }

*/

// #pragma omp declare reduction(mindisNode : Node* :              \
// omp_out = omp_in->cur_dist > omp_out->cur_dist ? omp_out : omp_in) 
//     // initializer (omp_priv=NULL)
    // #pragma omp parallel for default(shared) reduction(min:fn_minDist)  
// #pragma omp declare reduction (merge : std::vector<int> \
//   : omp_out.insert(omp_out.end(), omp_in.begin(), omp_in.end()))

Node* RRTSTAR::findNearest(const Point point) {
    float fn_minDist = FLT_MAX;//set the minimum distance to the maximum number possible
    Node* fn_closest = NULL;
    #ifdef DEBUG_FINDNEAREST
    printf("There are %d nodes in the tree.\n", this->nodes.size());
    std::cout<<"New node: ("<<point.m_x<<", "<<point.m_y<<")\n";
    #endif
    int min_index;

    #ifdef PARALLEL
     #pragma omp parallel for // reduction(mindisNode : fn_closest)  

    #endif
    for (size_t i=0; i < this->nodes.size(); i++) { //iterate through all nodes of the tree to find the closest to the new node
        
        #ifdef DEBUG_FINDNEAREST
        // #if (defined DEBUG_FINDNEAREST && defined PARALLEL) 
            // int omp_id = omp_get_thread_num();
            // std::cout<<"I'm t"<<omp_id<<"\n";
            // if(omp_id==0)printf("Total: %d of threads\n",omp_get_num_threads());
        #endif
        float fn_dist = this->distance(point, this->nodes[i]->position);
        
        if (fn_dist < fn_minDist) {
            fn_minDist = fn_dist;
            fn_closest = this->nodes[i];
            fn_closest->cur_dist = fn_dist;
            #if (defined DEBUG_FINDNEAREST && defined PARALLEL)
            // printf("t%d pick index: %d, closest_point(%f,%f). Orig_cost:%f, fn_dist:%f\n",omp_id, i, this->nodes[i]->position.m_x, this->nodes[i]->position.m_y, fn_minDist, fn_dist);
            #endif
            min_index = i;
        }
    }
    #ifdef DEBUG_FINDNEAREST
    // printf("There are %d nodes in the tree.\n", this->nodes.size());
    printf("nearest neighbor: at (%f,%f)\n",fn_closest->position.m_x, fn_closest->position.m_y);
    printf("min_index: %d\n\n",min_index);
    #endif
    // TODO: should modify here since dependencies: 有4個threads的話，要比較4個中最近的neighbor!
    return fn_closest;
}




// Return several neighbors within the radius. (Multiple Nodes)
// Write ALL near neighbors of "point"(<radius) to vector (neighbor_nodes) shared(neighbor_nodes)
void RRTSTAR::findNearNeighbors(const Point point, const float radius, std::vector<Node*>& neighbor_nodes) { // Find neighbor nodes of the given node within the defined radius
    #ifdef PARALLEL
        // #pragma omp parallel for  KEY!!! Error here!!
    #endif
    for (size_t i = 0; i < this->nodes.size(); i++) { //iterate through all nodes to see which ones fall inside the circle with the given radius.
        if (this->distance(point, this->nodes[i]->position) < radius) {
            neighbor_nodes.push_back(this->nodes[i]);
        }
    }
}

float RRTSTAR::distance(const Point p, const Point q) { //Find the distance between two points.
    Point dist_v = p - q;
    return sqrt(powf(dist_v.m_x, 2) + powf(dist_v.m_y, 2));
}

float RRTSTAR::getCost(const Node* N) { //get the cost current node (traveling from the given node to the root)
    return N->cost;
}


float RRTSTAR::pathCost(const Node* Np, const Node* Nq) { //Compute the distance between the position of two nodes
    return this->distance(Nq->position, Np->position);
}

// From n_nearest steer to n_rand directoin. if TOO FAR (>step_size):just step_size.
Point RRTSTAR::steer(const Node n_rand, const Node* n_nearest) { // Steer from new node towards the nearest neighbor and interpolate if the new node is too far away from its neighbor

    if (this->distance(n_rand.position, n_nearest->position) >this->m_step_size) { //check if the distance between two nodes is larger than the maximum travel step size
        Point steer_p = n_rand.position - n_nearest->position;
        double steer_norm = this->distance(n_rand.position, n_nearest->position);
        steer_p = steer_p / steer_norm; //normalize the vector, make ||steer|| = 1, leave unit vector in steer direction.
        return (n_nearest->position + this->m_step_size * steer_p); //travel in the direction of line between the new node and the near node
    }
    else {
        return  (n_rand.position);
    }

}
//(dynamic, 2)


// Answer's parent is not the root. why?
Node* RRTSTAR::findParent(std::vector<Node*> v_n_near,Node* n_nearest, Node* n_new) {
    Node* fp_n_parent = n_nearest; //create new note to find the parent(New Parent)
    float fp_cmin = this->getCost(n_nearest) + this->pathCost(n_nearest, n_new); // Update cost of reaching "N_new" from "N_Nearest"
    #ifdef PARALLEL
    // #pragma omp parallel for reduction(min:fp_cmin)
    #endif    
    for (size_t j = 0; j < v_n_near.size(); j++) { //In all members of "N_near", check if "N_new" can be reached from a different parent node with cost lower than Cmin, and without colliding with the obstacle.
        Node* fp_n_near = v_n_near[j];
        if (!this->world->checkObstacle(fp_n_near->position, n_new->position) &&
            (this->getCost(fp_n_near) + this->pathCost(fp_n_near, n_new)) < fp_cmin) {
            fp_n_parent = fp_n_near; // a near node with minimun cost of path
            fp_cmin = this->getCost(fp_n_near) + this->pathCost(fp_n_near, n_new); //update the cost of path
        }
    }
    return fp_n_parent;
}


std::vector<Point> RRTSTAR::get_available_points(){
    return this->Available_Points;
}


void RRTSTAR::insertNode(Node* n_parent, Node* n_new) { //Append the new node to the tree.
    n_new->parent = n_parent; //update the parent of new node
    n_new->cost = n_parent->cost + this->pathCost(n_parent, n_new);//update the cost of new node
    n_parent->children.push_back(n_new); //update the children of the nearest node to the new node
    this->nodes.push_back(n_new);//add the new node to the tree
    // this->Available_Points.push_back(n_new->position); //Add one more availble point! 

    this->lastnode = n_new;//inform the tree which node is just added
}

void RRTSTAR::reWire(Node* n_new, std::vector<Node*>& neighbor_nodes) { // Rewire the tree to decrease the cost of the path. 
    for (size_t j = 0; j < neighbor_nodes.size(); j++) {  // Search through nodes in "N_near" and see if changing their parent to "N_new" lowers the cost of the path. Also check the obstacles
        Node* rw_n_near = neighbor_nodes[j];
        if (!this->world->checkObstacle(n_new->position, rw_n_near->position) &&
            (this->getCost(n_new) + this->pathCost(n_new, rw_n_near)) < this->getCost(rw_n_near)) {
            Node* rw_n_parent = rw_n_near->parent;
            float rw_costdifference = this->getCost(rw_n_near) - (this->getCost(n_new) + this->pathCost(n_new, rw_n_near)); //calculate the cost  by which the cost of all children of the near node must decrease
            // Remove branch between N_Parent and N_Near
            rw_n_parent->children.erase(std::remove(rw_n_parent->children.begin(), rw_n_parent->children.end(), rw_n_near), rw_n_parent->children.end());
            // Add branch between N_New and N_Near
            rw_n_near->cost = this->getCost(n_new) + this->pathCost(n_new, rw_n_near);
            rw_n_near->parent = n_new;
            n_new->children.push_back(rw_n_near);
            // this->Available_Points.push_back(n_new->position);
            this->updateChildrenCost(rw_n_near, rw_costdifference);// Update the cost of all children of the near node 
        }
    }
}

// Here can I parallelize TOO.
void RRTSTAR::updateChildrenCost(Node* n, const float costdifference) {//Update the cost of all children of a node after rewiring 
    
    for (size_t i = 0; i < n->children.size(); i++)
    {
        n->children[i]->cost = n->children[i]->cost - costdifference;
        this->updateChildrenCost(n->children[i], costdifference); //recursive function. call it self to go through all children of the given node.
    }

}


bool RRTSTAR::reached() { //check if the last node in the tree is close to the end position.
    if (this->distance(this->lastnode->position, this->destination) < m_destination_threshhold) {
        return true;
    }
    return false;
}

void RRTSTAR::setStepSize(const float step) { // set the step size (the maximum distance between two nodes) for the RRT* algorithm
    this->m_step_size = step;
}

float RRTSTAR::getStepSize() { // get the step size (the maximum distance between two nodes) of the RRT* algorithm
    return this->m_step_size;
}

void RRTSTAR::setMaxIterations(const int iter) { // set the maximum number of iteration for the RRT* algorithm
    this->m_max_iter = iter;
}

int RRTSTAR::getMaxIterations() { //get the maximum number of iteration of the RRT* algorithm
    return this->m_max_iter;
}

int RRTSTAR::getCurrentIterations() {
    return this->m_num_itr;
}


void RRTSTAR::savePlanToFile(const std::vector<Point> path,const std::string filename, const std::string fileHeader) { //Save the the plan (vector of points) to file.
    
    std::ofstream spf_pathfile;
    spf_pathfile.open(filename);
    if (!fileHeader.empty()){
        spf_pathfile << fileHeader << std::endl;
       // std::cout << fileHeader << std::endl;
        spf_pathfile << "X" << '\t' << "Y" << std::endl;
       // std::cout << "X" << '\t' << "Y" << std::endl;
    }
    for (size_t s = 0; s < path.size(); s++) {
        spf_pathfile << (path[s].m_x) << '\t' << (path[s].m_y) << std::endl;
       // std::cout << (path[s].m_x) << '\t' << (path[s].m_y) << std::endl;
    }
    spf_pathfile.close();
}


std::vector<Point> RRTSTAR::generatePlan(Node* n) {// generate shortest path to destination.
    // CAN I parallelize here? But should first flatten, then.
    while (n != NULL) { // It goes from the last_node to the root
        this->path.push_back(n);
        n = n->parent;
    }

    // std::cout<<"Update bestpath!\n";
    this->bestpath.clear();// best path: {last_node, n-1, n-2, ... , root (starting point)}
    this->bestpath = this->path;    //store the current plan as the best plan so far

    // Every time we update bestpath, clear the 'path' container. For next path
    this->path.clear(); //clear the path as we have stored it in the Bestpath variable.
    this->m_cost_bestpath = this->bestpath[0]->cost; //store the cost of the generated path
    return this->planFromBestPath();
}

std::vector<Point> RRTSTAR::planFromBestPath() { // Generate plan (vector of points) from the best plan so far.
    std::vector<Point> pfb_generated_plan;
    // Accelerate I/O here. 
    for (size_t i = 0; i < this->bestpath.size(); i++) { // It goes from a node near destination to the root
        pfb_generated_plan.push_back(this->bestpath[i]->position);
    }
    return pfb_generated_plan;
}


void RRTSTAR::deleteNodes(Node* root){ //Free up memory when RRTSTAR destructor is called.

    for (auto& i : root->children) {
        deleteNodes(i);
    }
    delete root;
}