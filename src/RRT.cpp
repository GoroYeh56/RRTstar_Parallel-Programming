/**
 *  This file contains classes and methods to implement RRT* algorithm
 */


#include"RRTstar.h"


#include "omp.h"

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

----------------------- */

std::vector<Point> RRTSTAR::RRT_Explore(int K) {
    // while iter < MAX_Iterations

    // 1. K iterations (sample K points)
    for (int iter=0; iter < K; iter++){
        this->m_num_itr++;
        // 2. random sample a Point.
        Node plan_n_rand = this->getRandomNode();
        while(plan_n_rand.position.m_x == 0 || plan_n_rand.position.m_y==0){
            Node plan_n_rand = this->getRandomNode();
        } 
        // 3. Find the nearest neighbor
        Node* plan_n_nearest = this->findNearest(plan_n_rand.position);  //Find the closest node to the new random node.
        // 4. If distance from this neighbor to goal is
        //    within threshold :=> STOP (reach Goal)
        if(this->distance(plan_n_nearest->position, this->destination) < this->m_destination_threshhold){
            return this->Available_Points;
        }
        // 5. Else: Steer from this n_neighbor to the n_rand     
        Point plan_p_new = this->steer(plan_n_rand, plan_n_nearest); //Steer from "N_Nearest" towards "N_rand": interpolate if node is too far away.
        
        // 6. If the new node didn't hit obstacles, append to the graph.
        //    Note: Check Obstacle here!   
        if (!this->world->checkObstacle(plan_p_new, plan_n_nearest->position)){            
            Node* plan_n_new = new Node; //create new node to store the position of the steered node.
            plan_n_new->position = plan_p_new; //create new node from the streered new point
            std::vector<Node*> plan_v_n_near; //create a vector for neighbor nodes
            this->findNearNeighbors(plan_n_new->position, this->m_rrstar_radius, plan_v_n_near); // Find nearest neighbors with a given radius from new node.
            Node* plan_n_parent=this->findParent(plan_v_n_near,plan_n_nearest,plan_n_new); //Find the parent of the given node (the node that is near and has the lowest path cost)
            this->insertNode(plan_n_parent, plan_n_new);//Add N_new to node list.
            this->Available_Points.push_back(plan_p_new);
        }
    }
    // 7. return the graph (A vector of Points)
    return this->Available_Points;
        
}

std::vector<Point> RRTSTAR::get_nodes_points(){
    std::vector<Point> nodes_points;
    // Accelerate I/O here. 
    for (size_t i = 0; i < this->nodes.size(); i++) { // It goes from a node near destination to the root
        nodes_points.push_back(this->nodes[i]->position);
    }
    return nodes_points;
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
Node* RRTSTAR::findNearest(const Point point) {
    float fn_minDist = FLT_MAX;//set the minimum distance to the maximum number possible
    Node* fn_closest = NULL;

    #pragma omp parallel for    
    for (size_t i = 0; i < this->nodes.size(); i++) { //iterate through all nodes of the tree to find the closest to the new node
        int omp_id = omp_get_thread_num();
        if(omp_id == 0 && this->getCurrentIterations()%500==0 ){
            printf("iter: %d, thread %d, Total %d of threads\n",this->getCurrentIterations(), omp_id, omp_get_num_threads());
            printf("Number of nodes: %d\n", this->nodes.size());
        }
        float fn_dist = this->distance(point, this->nodes[i]->position);
        if (fn_dist < fn_minDist) {
            fn_minDist = fn_dist;
            fn_closest = this->nodes[i];
        }
    }

    // TODO: should modify here since dependencies: 有4個threads的話，要比較4個中最近的neighbor!
    return fn_closest;
}

// Return several neighbors within the radius. (Multiple Nodes)
void RRTSTAR::findNearNeighbors(const Point point, const float radius, std::vector<Node*>& neighbor_nodes) { // Find neighbor nodes of the given node within the defined radius
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
        steer_p = steer_p / steer_norm; //normalize the vector
        return (n_nearest->position + this->m_step_size * steer_p); //travel in the direction of line between the new node and the near node
    }
    else {
        return  (n_rand.position);
    }


}

Node* RRTSTAR::findParent(std::vector<Node*> v_n_near,Node* n_nearest, Node* n_new) {
    Node* fp_n_parent = n_nearest; //create new note to find the parent
    float fp_cmin = this->getCost(n_nearest) + this->pathCost(n_nearest, n_new); // Update cost of reaching "N_new" from "N_Nearest"
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
    while (n != NULL) { // It goes from the given node to the root
        this->path.push_back(n);
        n = n->parent;
    }
    this->bestpath.clear();
    this->bestpath = this->path;    //store the current plan as the best plan so far

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