#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <common/angle_functions.hpp>
#include <cfloat>
#include <common/grid_utils.hpp>
using namespace std;

struct Node
{
    int y;
    int x;
    int parentX;
    int parentY;
    float gCost;
    float hCost; 
    float fCost;
};

static bool isValid(int x, int y, const ObstacleDistanceGrid& distances, const SearchParams& params) { //If our Node is an obstacle it is not valid
	if (distances.isCellInGrid(x, y)){
    	if (distances(x,y) < params.minDistanceToObstacle)
    	{
    		return false;
    	}
    	return true;
    }
    return false;
}

static bool isDestination(int x, int y, Point<int>& goalCell) {
    if (x == goalCell.x && y == goalCell.y) {
        return true;
    }
    return false;
}

static vector<Node> makePath(std::vector<Node>& map, Point<int>& goalCell, const ObstacleDistanceGrid& distances) {
    int x = goalCell.x;
    int y = goalCell.y;
    std::stack<Node> path;
    std::vector<Node> usablePath;
    int id = y*distances.widthInCells() + x;
    while (!(map[id].parentX == x && map[id].parentY == y) && map[id].x != -1 && map[id].y != -1) 
    {
        path.push(map[id]);
        int tempX = map[id].parentX;
        int tempY = map[id].parentY;
        x = tempX;
        y = tempY;
        id = y*distances.widthInCells() + x;

    }
    path.push(map[id]);

    while (!path.empty()) {
        Node top = path.top();
        path.pop();
        usablePath.emplace_back(top);
    }
    return usablePath;
}

robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start);  

    auto goalCell = global_position_to_grid_cell(Point<float>(goal.x, goal.y), distances);

    if (isValid(goalCell.x, goalCell.y, distances, params)==false) 
    {
		path.path_length = path.path.size();
		return path;
	}

    bool closedList[distances.widthInCells()][distances.heightInCells()];

    unsigned int grid_size = distances.heightInCells()*distances.widthInCells();

    //Initialize whole map
    std::vector<Node> allMap(distances.widthInCells()*distances.heightInCells());
    for (int x = 0; x < distances.widthInCells(); x++) {
        for (int y = 0; y < distances.heightInCells(); y++) {
        	int id = y * distances.widthInCells() + x;

            allMap[id].fCost = FLT_MAX;
            allMap[id].gCost = FLT_MAX;
            allMap[id].hCost = FLT_MAX;
            allMap[id].parentX = -1;
            allMap[id].parentY = -1;
            allMap[id].x = x;
            allMap[id].y = y;

            closedList[x][y] = false;
        }
    }

    //Initialize our starting list
    auto startCell = global_position_to_grid_cell(Point<float>(start.x, start.y), distances);
    int x = startCell.x;
    int y = startCell.y;
    int id = y*distances.widthInCells() + x;
    allMap[id].fCost = 0.0;
    allMap[id].gCost = 0.0;
    allMap[id].hCost = 0.0;
    allMap[id].parentX = x;
    allMap[id].parentY = y;

    if (isValid(startCell.x, startCell.y, distances, params)==false) 
    {
		path.path_length = path.path.size();
		return path;
	}

    std::vector<Node> openList;  
    openList.emplace_back(allMap[id]);
    bool destinationFound = false;
    // std::cout<<"grid size"<<grid_size<<std::endl;

    while (!openList.empty()&&openList.size()<grid_size) 
    {
        Node node;
        do {
            float temp = FLT_MAX;
            int itNodeID = 0;
            for (unsigned int i_num = 0; i_num < openList.size(); i_num++)
            {
            	Node n = openList[i_num];
            	if(n.fCost < temp)
            	{
            		temp = n.fCost;
            		itNodeID = i_num;
            	}
            }
            node = openList[itNodeID];
            openList.erase(openList.begin()+itNodeID);
        } while (isValid(node.x, node.y, distances, params) == false);

        x = node.x;
        y = node.y;
        closedList[x][y] = true;
        std::vector<Node> usablePath;
        pose_xyt_t pose_temp;
        
        //For each neighbour starting from North-West to South-East
        for (int newX = -1; newX <= 1; newX++) {
            for (int newY = -1; newY <= 1; newY++) {
                double gNew, hNew, fNew;
                if (isValid(x + newX, y + newY, distances, params)) {
                	int id = (y + newY)*distances.widthInCells() + x + newX;
                    if (isDestination(x + newX, y + newY, goalCell))
                    {
                        //Destination found - make path
                        allMap[id].parentX = x;
                        allMap[id].parentY = y;
                        usablePath = makePath(allMap, goalCell, distances);
                        for (unsigned int i = 0; i < usablePath.size();i++){
                        	auto globalpos = grid_position_to_global_position(Point<float>(usablePath[i].x, usablePath[i].y), distances);
                        	pose_temp.x = globalpos.x;
                        	pose_temp.y = globalpos.y;
                        	pose_temp.theta = wrap_to_pi(atan2(usablePath[i].y - usablePath[i].parentY, usablePath[i].x - usablePath[i].parentX));
                        	// std::cout<<"path: "<<pose_temp.x<<" "<<pose_temp.y<<" "<<" "<<pose_temp.theta<<std::endl;
                        	path.path.push_back(pose_temp);
                        }
                        path.path.push_back(goal);
    					path.path_length = path.path.size();
    					destinationFound = true;
    					return path;
                    }
                    // else if (!(abs(newX)==abs(newY))&&(closedList[x + newX][y + newY] == false))
                    else if (closedList[x + newX][y + newY] == false)
                    {
                    	if (!(abs(newX)==abs(newY))){
                    		gNew = node.gCost + 1.0;
                    	}
                    	else {
                    		gNew = node.gCost + 1.4;
                    	}
                    	
                        hNew = sqrt((x + newX - goal.x)*(x + newX - goal.x) + (y + newY - goal.y)*(y + newY - goal.y));
                        if ((distances(x + newX, y + newY) > params.minDistanceToObstacle)&&(distances(x + newX, y + newY) < params.maxDistanceWithCost))
                        {
                        	hNew +=  0.5*pow(distances.cellsPerMeter() *(params.maxDistanceWithCost - distances(x + newX, y + newY)), params.distanceCostExponent);
                        }
                        fNew = gNew + hNew;
                        // Check if this path is better than the one already present
                        if (allMap[id].fCost == FLT_MAX || allMap[id].fCost > fNew)
                        {
                            // Update the details of this neighbour node
                            allMap[id].fCost = fNew;
                            allMap[id].gCost = gNew;
                            allMap[id].hCost = hNew;
                            allMap[id].parentX = x;
                            allMap[id].parentY = y;
                            openList.emplace_back(allMap[id]);
                        }
                    }
                }
            }
        }
    }
    if (destinationFound == false) 
    {
    cout << "Destination not found" << endl;
	}
	path.path_length = path.path.size();
	return path;
}
