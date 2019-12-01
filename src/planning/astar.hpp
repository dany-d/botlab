#ifndef PLANNING_ASTAR_HPP
#define PLANNING_ASTAR_HPP

#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <common/point.hpp>

#include <limits>

class ObstacleDistanceGrid;


class Node
{
public:
    Point<int> n;
    mutable Node *p;  
    mutable float f_cost;
    mutable float g_cost;

    Node(Point<int> n, Node *p, float g_cost) : n(Point<int>(n.x, n.y)), p(p), f_cost(std::numeric_limits<float>::infinity()), g_cost(g_cost){};
};

inline bool operator==(const Node &lhs, const Node &rhs)
{
    return (lhs.n.x == rhs.n.x) && (lhs.n.y == rhs.n.y);
}

inline bool operator<(const Node &lhs, const Node &rhs)
{
    // Go from bottom left to top right
    return lhs.f_cost < rhs.f_cost;
}

struct Compare
{
    inline bool operator()(const Node *lhs, const Node *rhs) const
    {
        return lhs->f_cost > rhs->f_cost;
    }
};

struct equal
{
    inline bool operator()(Node *const lhs, Node *const rhs) const
    {
        return lhs->n.x == rhs->n.x && lhs->n.y == rhs->n.y;
    }
};

struct hash
{
    size_t operator()(Node *const x) const
    {
        return (
            (51 + std::hash<int>()(x->n.x)) * 51 + std::hash<int>()(x->n.y));
    }
};



/**
* SearchParams defines the parameters to use when searching for a path. See associated comments for details
*/
struct SearchParams
{
    double minDistanceToObstacle;   ///< The minimum distance a robot can be from an obstacle before
                                    ///< a collision occurs
                                    
    double maxDistanceWithCost;     ///< The maximum distance from an obstacle that has an associated cost. The planned
                                    ///< path will attempt to stay at least this distance from obstacles unless it must
                                    ///< travel closer to actually find a path
                                    
    double distanceCostExponent;    ///< The exponent to apply to the distance cost, whose function is:
                                    ///<   pow(maxDistanceWithCost - cellDistance, distanceCostExponent)
                                    ///< for cellDistance > minDistanceToObstacle && cellDistance < maxDistanceWithCost
};


/**
* search_for_path uses an A* search to find a path from the start to goal poses. The search assumes a circular robot
* 
* \param    start           Starting pose of the robot
* \param    goal            Desired goal pose of the robot
* \param    distances       Distance to the nearest obstacle for each cell in the grid
* \param    params          Parameters specifying the behavior of the A* search
* \return   The path found to the goal, if one exists. If the goal is unreachable, then a path with just the initial
*   pose is returned, per the robot_path_t specification.
*/
robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params);

#endif // PLANNING_ASTAR_HPP
