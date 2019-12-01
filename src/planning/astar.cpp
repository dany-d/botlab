#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <algorithm>
#include <iostream>
#include <vector>
#include <unordered_set>
#include <queue>


double obs_cost(const SearchParams &params,double obsDistance)
{

    double maxDistanceWithCost = params.maxDistanceWithCost;
    double minDistanceToObstacle = params.minDistanceToObstacle;
    double distanceCostExponent = params.distanceCostExponent;
    if (obsDistance > minDistanceToObstacle && obsDistance < maxDistanceWithCost){
        return pow(maxDistanceWithCost - obsDistance, distanceCostExponent);
    }
    else{
        return 0;
    }
}


// bool isGoal(Node *currentNode, Node *goal)
// {
//     if (currentNode == goal)
//         return (true);
//     else
//         return (false);
// }

double dist(Node *currentNode, Node *goal)
{
    // Return using the distance formula
    return ((double)sqrt((currentNode->n.x - goal->n.x) * (currentNode->n.x - goal->n.x) + 
                        (currentNode->n.y - goal->n.y) * (currentNode->n.y - goal->n.y)));
}

bool isValid(Node *currentNode, const SearchParams &params, const ObstacleDistanceGrid &distances){

    if (distances.operator()(currentNode->n.x, currentNode->n.y)< params.minDistanceToObstacle
    && distances.isCellInGrid(currentNode->n.x, currentNode->n.y)){
        return true;
    }
    else return false;
}

double cost_to_go(int newX,int newY,double step){
    if (abs(newX) == 1&& abs(newY ==1)){
        return step;
    }
    else return (1.414*step);
}

robot_path_t construct_path(Node *currentNode, robot_path_t path)
{
    pose_xyt_t setpoints;
    // Node *trackNode = currentNode;
    while(currentNode->p == nullptr){
        setpoints.x = currentNode->n.x;
        setpoints.y = currentNode->n.y;

        ///// TODO: check for theta
        setpoints.theta = 0; 
        path.path.push_back(setpoints);
        currentNode = currentNode->p;
    }

    return path;
} 



robot_path_t search_for_path(pose_xyt_t start,
                pose_xyt_t goal,
                const ObstacleDistanceGrid &distances,
                const SearchParams &params)
{
    robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start);
    path.path_length = path.path.size();

    // initialize start node
    std::priority_queue<Node *, std::vector<Node *>, Compare> pq;
    std::unordered_set<Node *, hash, equal> open_set;
    
    std::vector<Node> open_list;
    std::vector<Node> closed_list;
    Point<int> start_point(start.x,start.y);
    Point<int> goal_point(goal.x,goal.y);

    Node start_node(start_point, nullptr, 0);
    Node goal_node(goal_point,nullptr,0);
    start_node.f_cost = dist(&start_node,&goal_node) 
    + obs_cost(params, distances.operator()(start_node.n.x, start_node.n.y));

    pq.push(&start_node);
    open_set.insert(&start_node);

    double step = 0.05;

    while (open_list.size() > 0)
    {
        // select node
        Node *currentNode = pq.top();
        pq.pop();
        open_set.erase(currentNode);

        // check destination
        if (currentNode == &goal_node){
            path = construct_path(currentNode,path);
            break;
        }

        // init neighbors
        Point<int> neighbor_pt(0, 0);
        Node neighbor(neighbor_pt, nullptr, 0);
        double temp_g_cost;


        // 8 -connected
        for (int newX = -1; newX <= 1; newX++)
        {
            for (int newY = -1; newY <= 1; newY++)
            {
                neighbor.n.y = currentNode->n.x + step * newX;
                neighbor.n.y = currentNode->n.y + step * newY;

                // check for valid neighbors (obstacle and valid cell) 
                if (isValid(&neighbor, params, distances))
                {
                    temp_g_cost = currentNode->g_cost + cost_to_go(newX, newY, step);
                    if (temp_g_cost < neighbor.g_cost)
                    {
                        neighbor.p = currentNode;
                        neighbor.g_cost = temp_g_cost;
                        neighbor.f_cost = neighbor.g_cost + dist(currentNode, &goal_node) 
                        + obs_cost(params, distances.operator()(neighbor.n.x, neighbor.n.y));
                        if (open_set.find(&neighbor) != open_set.end())
                        {
                            pq.push(&neighbor);
                            open_set.insert(&neighbor);
                        }
                    }
                }
            }
        }
    }
    return path;
}
