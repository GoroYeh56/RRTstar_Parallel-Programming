/**
 *  This file contains classes and methods to construct the world for the robot.
 *  It contains classes to store points, lines, world width and height, and obstacles.
 */


#include"World.h"


//Point class Constructors
Point::Point()
    :m_x(0.0f), m_y(0.0f) {}
Point::Point(float X, float Y)
    : m_x(X), m_y(Y) {}




//Line class Constructors
Line::Line() {};
Line::Line(float x1pos, float y1pos, float x2pos, float y2pos)
{
    this->m_p1.m_x = x1pos;
    this->m_p1.m_y = y1pos;
    this->m_p2.m_x = x2pos;
    this->m_p2.m_y = y2pos;
}

//Line class Methods


bool Line::LineIntersection(const Line& OtherLine) { //Return if two lines has an intersection
    // Line "ThisLine" represented as a1x + b1y = c1 
    float lis_a1 = this->m_p2.m_y - this->m_p1.m_y;
    float lis_b1 = this->m_p1.m_x - this->m_p2.m_x;
    float lis_c1 = lis_a1 * (this->m_p1.m_x) + lis_b1 * (this->m_p1.m_y);

    // Line "OtherLine" represented as a2x + b2y = c2 
    float lis_a2 = OtherLine.m_p2.m_y - OtherLine.m_p1.m_y;
    float lis_b2 = OtherLine.m_p1.m_x - OtherLine.m_p2.m_x;
    float lis_c2 = lis_a2 * (OtherLine.m_p1.m_x) + lis_b2 * (OtherLine.m_p1.m_y);

    float determinant = lis_a1 * lis_b2 - lis_a2 * lis_b1; //Calculate the determinant

    if (determinant == 0) // The lines are parallel.
    {
        return 0;
    }
    else
    {
        //finding the intersection point
        float x_intersect = (lis_b2 * lis_c1 - lis_b1 * lis_c2) / determinant;
        float y_intersect = (lis_a1 * lis_c2 - lis_a2 * lis_c1) / determinant;
        // Check if the intersecting point is within both line segments
        if (((x_intersect > std::min(this->m_p1.m_x, this->m_p2.m_x)) && (x_intersect < std::max(this->m_p1.m_x, this->m_p2.m_x))) &&
            ((x_intersect > std::min(OtherLine.m_p1.m_x, OtherLine.m_p2.m_x)) && (x_intersect < std::max(OtherLine.m_p1.m_x, OtherLine.m_p2.m_x))))
            return 1;
        if (((y_intersect > std::min(this->m_p1.m_y, this->m_p2.m_y)) && (y_intersect < std::max(this->m_p1.m_y, this->m_p2.m_y))) &&
            ((y_intersect > std::min(OtherLine.m_p1.m_y, OtherLine.m_p2.m_y)) && (y_intersect < std::max(OtherLine.m_p1.m_y, OtherLine.m_p2.m_y))))
            return 1;
    }
    return 0;
}


//Obstacles class Constructors


World::World()
    : world_width(500), world_height(500) {}


World::World(float w, float h)
    : world_width(w), world_height(h) {}

//Obstacles class Methods

void World::addObstacle(Point& topLeft, Point& bottomRight) {
    // Get topLeft and bottomRight points from the given points, and check if the provided points are in the correct order.
    if (topLeft.m_x > bottomRight.m_x && topLeft.m_y > bottomRight.m_y) {
        Point ao_tmp = topLeft;
        topLeft = bottomRight;
        bottomRight = ao_tmp;
    }
    //add the obstacle.
    m_obstacles.push_back(std::make_pair(topLeft, bottomRight));

}

bool World::checkObstacle(Point& p1, Point& p2) {
    Line co_line(p1.m_x, p1.m_y, p2.m_x, p2.m_y); //creat line between two nodes
    for (int i = 0; i < (int)m_obstacles.size(); i++) { //iterate through all obstacles
        //create four lines for each side of the rectangle.
        Line co_l1(m_obstacles[i].first.m_x, m_obstacles[i].first.m_y, m_obstacles[i].second.m_x, m_obstacles[i].first.m_y);
        Line co_l2(m_obstacles[i].first.m_x, m_obstacles[i].first.m_y, m_obstacles[i].first.m_x, m_obstacles[i].second.m_y);
        Line co_l3(m_obstacles[i].second.m_x, m_obstacles[i].second.m_y, m_obstacles[i].second.m_x, m_obstacles[i].first.m_y);
        Line co_l4(m_obstacles[i].second.m_x, m_obstacles[i].second.m_y, m_obstacles[i].first.m_x, m_obstacles[i].second.m_y);
        // check for the intersection
        if (co_line.LineIntersection(co_l1) || co_line.LineIntersection(co_l2) || co_line.LineIntersection(co_l3) || co_line.LineIntersection(co_l4))
            return true;
    }
    return false;
}

void World::saveObsToFile(const std::string filename) {
    std::ofstream sof_pathfile;
    sof_pathfile.open(filename);
    sof_pathfile << "Obstacles. Obstacles are stored as rectangles. Rectangle is denoted by two points; top left and bottom right." << std::endl;
    sof_pathfile << "Top Left X" << '\t' << "Top Left X Y" << '\t' << "Bottom Right X" << '\t' << "Bottom Right Y" << std::endl;
    for (size_t s = 0; s < this->m_obstacles.size(); s++) { //iterate through all obstacles
        sof_pathfile << this->m_obstacles[s].first.m_x << '\t' << this->m_obstacles[s].first.m_y << '\t' << this->m_obstacles[s].second.m_x << '\t' << this->m_obstacles[s].second.m_y << std::endl;
    }
    sof_pathfile.close();
}


void World::setWorldWidth(float w) {
    this->world_width = w;
}



void World::setWorldHeight(float h) {
    this->world_height = h;
}



float World::getWorldWidth() {
    return this->world_width;
}



float World::getWorldHeight() {
    return this->world_height;
}

