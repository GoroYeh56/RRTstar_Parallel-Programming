/**
 *  This file contains classes and methods to construct the world for the robot.
 *  It contains classes to store points, lines, world width and height, and obstacles.
 */


#pragma once


#include<iostream>
#include <vector>
#include <algorithm>
#include <fstream>
#include <float.h>


/**
* @brief Class for storing position of a point in the space
*/
class Point {
public:
    //Data member
    float m_x;
    float m_y;
    //Constructors
    Point();
    Point(float X, float Y);


};

//Operators overloading for Point class
template <typename T>
inline Point operator *(const Point& left, T right)
{
    return Point(left.m_x * right, left.m_y * right);
}

template <typename T>
inline Point operator *(T left, const Point& right)
{
    return Point(right.m_x * left, right.m_y * left);
}

inline Point operator +(const Point& left, const Point& right)
{
    return Point(left.m_x + right.m_x, left.m_y + right.m_y);
}


inline Point operator -(const Point& left, const Point& right)
{
    return Point(left.m_x - right.m_x, left.m_y - right.m_y);
}

template <typename T>
inline Point operator /(const Point& left, T right)
{
    return Point(left.m_x / right, left.m_y / right);
}


/**
* @brief Class for creating a line segment from two Points. 
*/
class Line {
public:
    //Constructors
    Line();
    Line(float x1pos, float y1pos, float x2pos, float y2pos);
    //Methods

    /**
    * @brief Return if "this" line has an intersection with "other" line.
    * @param Line OtherLine
    * @return bool
    */
    bool LineIntersection(const Line& OtherLine);
private:
    // Data Members 
    Point m_p1;
    Point m_p2;
};


/**
* @brief class for storing obstacles and world dimension. Obstacles are stored as rectangles. Rectangle is denoted by two points; top left and bottom right.
* if the world width and world height do not set using setWorldWidth and setWorldHeight methods, the default values which are world_width=500 and world_height=500
* will be set.
*/
class World
{
private:
    // Data Members 
    std::vector<std::pair<Point, Point> > m_obstacles;
    float world_width;
    float world_height;
public:
    //Constructors
    World();
    World(float w, float h);
    //class Methods
    
    /**
    * @brief Create obstacles. Obstacles are stored as rectangles. Rectangle is denoted by two points; top left and bottom right.
    * @param Point topLeft (i.e., position of the top left point)
    * @param Point bottomRight (i.e., position of the bottom right point)
    * @return bool
     */
    void addObstacle(Point& topLeft, Point& bottomRight);

    /**
   * @brief Check if there is any obstacle between the 2 nodes.
   * @param Point p1 (i.e., the position of the first node)
   * @param Point p2 (i.e., the position of the second node)
   * @return bool value of whether obstacle exists between nodes
   */
    bool checkObstacle(Point& p1, Point& p2);


    /**
    * @brief Save obstacles to file
    * @param std::string filename
    * @param std::string fileHeader.
    * @return void
     */
    void saveObsToFile(const std::string filename);

    /**
    * @brief set the width of the world
    * @param float w
    * @return void
    */
    void setWorldWidth(float w);

    /**
    * @brief set the height of the world
    * @param float h
    * @return void
    */
    void setWorldHeight(float h);

    /**
    * @brief get the width of the world
    * @param void
    * @return float
    */
    float getWorldWidth();

    /**
    * @brief get the height of the world
    * @param void
    * @return float
    */
    float getWorldHeight();

};
