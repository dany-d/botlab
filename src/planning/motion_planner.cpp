#include <planning/motion_planner.hpp>
#include <planning/astar.hpp>
#include <common/grid_utils.hpp>
#include <common/timestamp.h>
#include <lcmtypes/robot_path_t.hpp>
#include <cmath>


MotionPlanner::MotionPlanner(const MotionPlannerParams& params)
: params_(params)
{
    setParams(params);
}


MotionPlanner::MotionPlanner(const MotionPlannerParams& params, const SearchParams& searchParams)
: params_(params)
, searchParams_(searchParams)
{
}


robot_path_t MotionPlanner::planPath(const pose_xyt_t& start,
                                     const pose_xyt_t& goal,
                                     const SearchParams& searchParams) const
{
    // If the goal isn't valid, then no path can actually exist
    if(!isValidGoal(goal))
    {
        robot_path_t failedPath;
        failedPath.utime = utime_now();
        failedPath.path_length = 1;
        failedPath.path.push_back(start);

        std::cout << "INFO: path rejected due to invalid goal\n";

        return failedPath;
    }

    // Otherwise, use A* to find the path
    return search_for_path(start, goal, distances_, searchParams);
}


robot_path_t MotionPlanner::planPath(const pose_xyt_t& start, const pose_xyt_t& goal) const
{
    return planPath(start, goal, searchParams_);
}


bool MotionPlanner::isValidGoal(const pose_xyt_t& goal) const
{
    float dx = goal.x - prev_goal.x, dy = goal.y - prev_goal.y;
    float distanceFromPrev = std::sqrt(dx * dx + dy * dy);
    //std::cout<<"Goal cell: "<<goal.x<<" "<<goal.y<<std::endl;
    //std::cout<<"Distance from prev: "<<distanceFromPrev<<std::endl;
    //std::cout<<"Number of frontiers: "<<num_frontiers<<std::endl;
    //if there's more than 1 frontier, don't go to a target that is within a robot diameter of the current pose
    //if(num_frontiers >= 1 && distanceFromPrev < searchParams_.minDistanceToObstacle) return false;
    //std::cout<<distances_.widthInCells()<<" "<<distances_.heightInCells()<<std::endl;
    auto goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances_);
    // A valid goal is in the grid
    if(distances_.isCellInGrid(goalCell.x, goalCell.y))
    {
        //std::cout<<"Distance: "<< goalCell.x <<" "<<goalCell.y<<" "<<distances_(goalCell.x, goalCell.y)<<"\n";
        // And is far enough from obstacles that the robot can physically occupy the space
        // Add an extra cell to account for discretization error and make motion a little safer by not trying to
        // completely snuggle up against the walls in the motion plan
        std::cout<<"is cell valid: "<<(distances_(goalCell.x, goalCell.y) > params_.robotRadius)<<std::endl;
        return distances_(goalCell.x, goalCell.y) > params_.robotRadius;
    }
    //std::cout<<"not in grid\n";

    // A goal must be in the map for the robot to reach it
    return false;
}


bool MotionPlanner::isPathSafe(const robot_path_t& path) const
{

    ///////////// TODO: Implement your test for a safe path here //////////////////
    if(path.path_length < 1)
    {
        return false;
    }

    // Look at each position in the path, along with any intermediate points between the positions to make sure they are
    // far enough from walls in the occupancy grid to be safe
    for(auto p : path.path)
    {
        Point<int> cell = distances_.poseToCell(p.x, p.y);
        if(distances_(cell.x, cell.y) < params_.robotRadius - 0.05)
        {
            std::cout<<"Path not safe\n";
            return false;
        }
    }
    return true;
}


void MotionPlanner::setMap(const OccupancyGrid& map)
{   //std::cout<<"Setup map\n";
    distances_.setDistances(map);
}


void MotionPlanner::setParams(const MotionPlannerParams& params)
{
    searchParams_.minDistanceToObstacle = params_.robotRadius;
    searchParams_.maxDistanceWithCost = 5.0 * searchParams_.minDistanceToObstacle;
    searchParams_.distanceCostExponent = 2.0;
}
