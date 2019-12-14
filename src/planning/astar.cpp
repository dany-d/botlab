#include "queue"
#include "unordered_set"
#include "stack"
#include "algorithm"

#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <algorithm>
#include <iostream>
#include <vector>
#include <unordered_set>
#include <queue>
#include <common/grid_utils.hpp>
#include <memory>

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

double dist(Node *currentNode, Node *goal)
{
    // Return using the eucl dist formula
    return ((double)sqrt((currentNode->n.x - goal->n.x) * (currentNode->n.x - goal->n.x) + 
                        (currentNode->n.y - goal->n.y) * (currentNode->n.y - goal->n.y)));
}

float roundit(float val)
{
    if (val < 0){
        return ceil(val*100 - 0.5)/100;}
    return floor(val*100 + 0.5)/100;
}

bool isValid(Node *currentNode, const SearchParams &params, const ObstacleDistanceGrid &distances){
    

    // if (distances.isCellInGrid(grid_position_to_global_position(currentNode->n, distances).x, grid_position_to_global_position(currentNode->n, distances).y)){
    if (distances.isCellInGrid(currentNode->n.x,currentNode->n.y)){
        // if ((distances(grid_position_to_global_position(currentNode->n, distances).x, grid_position_to_global_position(currentNode->n, distances).y) < params.minDistanceToObstacle)){
        if ((distances(currentNode->n.x,currentNode->n.y) <= params.minDistanceToObstacle)){
            return false;
        }
        return true;
    }
    return false;
}

double cost_to_go(int newX,int newY,double step){
    if (abs(newX) == 1&& abs(newY) ==1){
        return (1.414 * step);
    }
    else return step;
}

robot_path_t construct_path(Node *currentNode, robot_path_t path, const ObstacleDistanceGrid &distances)
{
    pose_xyt_t setpoints;
    int itr=0;
    int itr2=0;
    float theta_prev= 0;
    float x_prev = 0;
    float y_prev = 0;

    // Node *trackNode = currentNode;
    while(currentNode->p != nullptr){
        // std::cin.get();
        itr++;
        itr2++;
        setpoints.x = roundit(grid_position_to_global_position(currentNode->n, distances).x);
        setpoints.y = roundit(grid_position_to_global_position(currentNode->n, distances).y);
        setpoints.theta = atan2(y_prev - setpoints.y, x_prev - setpoints.x);

        if((itr ==7) || ((fabs(theta_prev-setpoints.theta) > 0.18) && (itr2==2))){
            path.path.push_back(setpoints);
            itr =0;
            itr2 = 0;

        }
        // std::cout << "added setpoint" << currentNode->n << "fcost:  " << currentNode->f_cost<< std::endl;
        
        x_prev = setpoints.x;
        y_prev = setpoints.y;
        theta_prev = setpoints.theta;

        currentNode = currentNode->p;
    }
    std::reverse(std::begin(path.path), std::end(path.path));

    return path;
} 

void printNode(Node *currentNode){
    std::cout << "x,y - " << currentNode->n << std::endl;
    std::cout << "f_cost,g_cost: " << currentNode->f_cost << "," << currentNode->g_cost << std::endl;
}

Node * print_queue(std::priority_queue<Node *, std::vector<Node *>, Compare> &pq)
{
    while (!pq.empty())
    {
        Node* a = pq.top();
        std::cout << "pq: " <<a->n << "parent: " << a->p->n << "," << a->f_cost   <<std::endl;
        pq.pop();
    }
}

void delete_set(std::vector <Node*> del_vector)
{
    for(int i=0; i < del_vector.size(); ++i)
    {
        delete del_vector[i];
        // del_vector[i]->f_cost = NULL;
        // delete &(del_vector[i]->g_cost);
        // delete[] del_vector[i];
        // std::cout << "ptr: " << del_vector[i] << " parent addr: " << del_vector[i]->p << " fcost: " << &del_vector[i]->f_cost << " ptr_gcost: " << &del_vector[i]->g_cost << std::endl;
    }
    
}

robot_path_t search_for_path(pose_xyt_t start,
                pose_xyt_t goal,
                const ObstacleDistanceGrid &distances,
                const SearchParams &params)
{
    robot_path_t path;
    
    // path.path.push_back(start);

    // initialize start node
    std::priority_queue<Node *, std::vector<Node *>, Compare> pq;
    std::unordered_set<Node *, hash, equal> open_set;
    std::vector<Node*> del_set;
    
    Point<double> global_start_pt(start.x,start.y);
    Point<int> start_point(global_position_to_grid_cell(global_start_pt,distances).x,global_position_to_grid_cell(global_start_pt,distances).y);
    Point<double> global_goal_pt(goal.x,goal.y);
    Point<int> goal_point(global_position_to_grid_cell(global_goal_pt, distances).x, global_position_to_grid_cell(global_goal_pt, distances).y);

    Node start_node = Node(start_point, nullptr, 0);
    Node goal_node = Node(goal_point, nullptr, 0);


    Node *start_ptr = &start_node;
    Node *goal_ptr = &goal_node;

   
    start_ptr->g_cost = 0;
    start_ptr->f_cost = dist(start_ptr, goal_ptr) + obs_cost(params, distances(start_ptr->n.x, start_ptr->n.y));

    // printNode(goal_ptr);
    pq.push(start_ptr);
    open_set.insert(start_ptr);


    double step = 1;
    int itr=0;

    // if(isValid(goal_ptr, params, distances)){
    //     std::cout << "Goal position invalid" << std::endl;
    // }

        while (open_set.size() > 0)
    {
        // select node
        Node *currentNode = pq.top();
        pq.pop();
        open_set.erase(currentNode);
        // std::cout << "currentNode: " << currentNode->n << std::endl;
        // printNode(currentNode);


        itr++;
        // std::cout << "itr - " << itr << std::endl;

        if (itr ==100000){
            std::cout << "iterated: 100k times... somethings up " << std::endl;
            // print_queue(pq);           // break;
            std::cin.get();
        }


        // check destination
        if (*currentNode == *goal_ptr)
        {
            // std::cout << "Goal found: " << std::endl;
            
            path = construct_path(currentNode,path,distances);
            path.path.insert(path.path.begin(),start);
            path.path.push_back(goal);

            // PRINT setpoints
            for(int i;i<path.path.size();++i){
                std::cout << path.path[i].x << "," << path.path[i].y << std::endl;
            }
            delete_set(del_set);
            path.utime = start.utime;
            path.path_length = path.path.size();
            // print_queue(pq);
            // std::cin.get();
            return path;
        }
        // init neighbors
        double temp_g_cost;

        // 8 -connected
        for (int newX = -1; newX <= 1; newX++)
        {
            for (int newY = -1; newY <= 1; newY++)
            {
                if (newX == 0 && newY ==0){continue;} // ignoring currentNode

                // Node *n_ptr;
                Point<int> neighbor_pt(currentNode->n.x + step * newX, currentNode->n.y + step * newY);
                // std::cout << "nei - " << (neighbor_pt) << std::endl;
                // Node neighbor(neighbor_pt, nullptr, 0);


                Node* n_ptr = new Node(neighbor_pt,nullptr,0);
                // n_ptr->n = neighbor_pt;

                // std::unique_ptr <Node[]> n_ptr2(new Node(neighbor_pt,nullptr[8]);
                // std::unique_ptr <Node> n_ptr = std::make_unique(n_ptr);

    
                // std::cout << "auto ptr - " << &n_pt2 << std::endl;

                // std::unique_ptr<Node> v1 = std::make_unique<Node>();
                // std::unique_ptr<double[]> p = std::make_unique<double[]>(42);


                // n_ptr->n.x = currentNode->n.x + step * newX;
                // n_ptr->n.y = currentNode->n.y + step * newY;
                // printNode(n_ptr);
                // std::cout << "curr - " << (currentNode->n) << std::endl;

                // check for valid neighbors (obstacle and valid cell) 
                if (isValid(n_ptr, params, distances))
                {
                    // std::cout << "neighbor valid" << std::endl;
                    temp_g_cost = currentNode->g_cost + cost_to_go(newX, newY, step);
                    if (temp_g_cost < n_ptr->g_cost)
                    {
                        n_ptr->p = currentNode;
                        // std::cout << n_ptr->p->n << currentNode->n << std::endl;
                        n_ptr->g_cost = temp_g_cost;
                        n_ptr->f_cost = n_ptr->g_cost + dist(currentNode,goal_ptr) + obs_cost(params, distances(n_ptr->n.x,n_ptr->n.y));
                        // std::cout << "obs_cost: " << pow(params.maxDistanceWithCost - distances(start_ptr->n.x, start_ptr->n.y),params.distanceCostExponent) << std::endl;
                        // + obs_cost(params, distances(grid_position_to_global_position(n_ptr->n,distances).x,grid_position_to_global_position(n_ptr->n,distances).y));
                        // std::cout << "can find" << (open_set.find(&n_ptr-> == open_set.end())<< std::endl;
                        if (open_set.find(n_ptr) == open_set.end())
                        {
                            pq.push(std::move(n_ptr));
                            open_set.insert(std::move(n_ptr));
                            del_set.push_back(n_ptr);
                            // std::cout << "Node added- " << n_ptr << std::endl;
                        }
                    }
                }
            }
        }
    }
}
