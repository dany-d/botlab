#include <planning/frontiers.hpp>
#include <planning/motion_planner.hpp>
#include <common/grid_utils.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/robot_path_t.hpp>
#include <queue>
#include <set>
#include <cassert>

bool sortcondition(frontier_t i1, frontier_t i2)
{
  return (i1.dist < i2.dist);
}


bool is_frontier_cell(int x, int y, const OccupancyGrid& map);
frontier_t grow_frontier(Point<int> cell, const OccupancyGrid& map, std::set<Point<int>>& visitedFrontiers);
robot_path_t path_to_frontier( frontier_t& frontier,
                              const pose_xyt_t& pose,
                              const OccupancyGrid& map,
                              const MotionPlanner& planner);
pose_xyt_t nearest_navigable_cell(pose_xyt_t pose,
                                  Point<float> desiredPosition,
                                  const OccupancyGrid& map,
                                  const MotionPlanner& planner);
pose_xyt_t search_to_nearest_free_space(Point<float> position, const OccupancyGrid& map, const MotionPlanner& planner);
double path_length(const robot_path_t& path);


std::vector<frontier_t> find_map_frontiers(const OccupancyGrid& map,
                                           const pose_xyt_t& robotPose,
                                           double minFrontierLength)
{
    /*
    * To find frontiers, we use a connected components search in the occupancy grid. Each connected components consists
    * only of cells where is_frontier_cell returns true. We scan the grid until an unvisited frontier cell is
    * encountered, then we grow that frontier until all connected cells are found. We then continue scanning through the
    * grid. This algorithm can also perform very fast blob detection if you change is_frontier_cell to some other check
    * based on pixel color or another condition amongst pixels.
    */
    std::vector<frontier_t> frontiers;
    std::set<Point<int>> visitedCells;

    Point<int> robotCell = global_position_to_grid_cell(Point<float>(robotPose.x, robotPose.y), map);
    std::queue<Point<int>> cellQueue;
    cellQueue.push(robotCell);
    visitedCells.insert(robotCell);

    // Use a 4-way connected check for expanding through free space.
    const int kNumNeighbors = 4;
    const int xDeltas[] = { -1, 1, 0, 0 };
    const int yDeltas[] = { 0, 0, 1, -1 };

    // Do a simple BFS to find all connected free space cells and thus avoid unreachable frontiers
    while(!cellQueue.empty())
    {
        Point<int> nextCell = cellQueue.front();
        cellQueue.pop();

        // Check each neighbor to see if it is also a frontier
        for(int n = 0; n < kNumNeighbors; ++n)
        {
            Point<int> neighbor(nextCell.x + xDeltas[n], nextCell.y + yDeltas[n]);

            // If the cell has been visited or isn't in the map, then skip it
            if(visitedCells.find(neighbor) != visitedCells.end() || !map.isCellInGrid(neighbor.x, neighbor.y))
            {
                continue;
            }
            // If it is a frontier cell, then grow that frontier
            else if(is_frontier_cell(neighbor.x, neighbor.y, map))
            {
                frontier_t f = grow_frontier(neighbor, map, visitedCells);

                // If the frontier is large enough, then add it to the collection of map frontiers
                if(f.cells.size() * map.metersPerCell() >= minFrontierLength)
                {
                    frontiers.push_back(f);
                }
            }
            // If it is a free space cell, then keep growing the frontiers
            else if(map(neighbor.x, neighbor.y) < 0)
            {
                visitedCells.insert(neighbor);
                cellQueue.push(neighbor);
            }
        }
    }

    return frontiers;
}


robot_path_t plan_path_to_home(const pose_xyt_t &homePose,
                                   const pose_xyt_t& robotPose,
                                   const OccupancyGrid& map,
                                   const MotionPlanner& planner){
   robot_path_t emptyPath;
   emptyPath.utime = robotPose.utime;
   emptyPath.path_length = 0;
   std::cout<<"Map info (w,h): "<<map.widthInMeters()<<" "<<map.heightInMeters()<<" "<<map.heightInCells()<<" "<<map.widthInCells()<<std::endl;

   pose_xyt_t goalPose2;
   goalPose2.x  = homePose.x;
   goalPose2.y  = homePose.y;

   int L = 5;
   for (int l=0; l<L; ++l){
     for (int m=0; m<l; ++m){
       for (int n=0; n<l; ++n){
         goalPose2.x = homePose.x + (m-l/2)*map.metersPerCell();
         goalPose2.y = homePose.y + (n-l/2)*map.metersPerCell();
         std::cout<<"Home 2: "<<goalPose2.x<<" "<<goalPose2.y<<std::endl;
         if(planner.isValidGoal(goalPose2)){
           robot_path_t path;
           path = planner.planPath(robotPose, goalPose2);
           if(planner.isPathSafe(path) && path.path_length!=0){
             std::cout<<"A path to home is returned.\n";
             return path;
           }
         }
       }
     }
   }
   std::cout<<"Empty Path to home returned.\n";
   return emptyPath;

}


robot_path_t plan_path_to_frontier(std::vector<frontier_t>& frontiers,
                                   const pose_xyt_t& robotPose,
                                   const OccupancyGrid& map,
                                   const MotionPlanner& planner)
{
    ///////////// TODO: Implement your strategy to select the next frontier to explore here //////////////////
    /*
    * NOTES:
    *   - If there's multiple frontiers, you'll need to decide which to drive to.
    *   - A frontier is a collection of cells, you'll need to decide which one to attempt to drive to.
    *   - The cells along the frontier might not be in the configuration space of the robot, so you won't necessarily
    *       be able to drive straight to a frontier cell, but will need to drive somewhere close.
    */

    //TODO: maybe need reorgnized frontier according to current pose. Right now it only return a valid path.
    robot_path_t emptyPath;
    emptyPath.utime = robotPose.utime;
    emptyPath.path_length = 0;

    float min=100000; // Created to find the frontier closest to us
    int min_index=0; // To store the index of the closest frontier

    //std::cout<<"Map info (w,h): "<<map.widthInMeters()<<" "<<map.heightInMeters()<<" "<<map.heightInCells()<<" "<<map.widthInCells()<<std::endl;
    int L = 10;

    pose_xyt_t desiredPose;

    // Finding the frontier closest to us
    for(unsigned int i=0; i<frontiers.size(); i++){
        pose_xyt_t goalPose;

        goalPose.x = 0;
        goalPose.y = 0;

        // Iterating through the cells inside one frontier and calculating median
        for(unsigned int j=0; j<frontiers[i].cells.size(); j++){
          goalPose.x += frontiers[i].cells[j].x;
          goalPose.y += frontiers[i].cells[j].y;
        }
        goalPose.x/= frontiers[i].cells.size();
        goalPose.y/= frontiers[i].cells.size();
        frontiers[i].mean.x = goalPose.x;
        frontiers[i].mean.y = goalPose.y;
        frontiers[i].dist = sqrt(pow(goalPose.x -robotPose.x,2)+pow(goalPose.y-robotPose.y,2));

        // if (min > sqrt(pow(goalPose.x -robotPose.x,2)+pow(goalPose.y-robotPose.y,2)))
        // {
        //   min = sqrt(pow(goalPose.x - robotPose.x, 2) + pow(goalPose.y - robotPose.y, 2));
        //   desiredPose.x=goalPose.x;
        //   desiredPose.y=goalPose.y;
        //   
        // }

        // std::sort(frontiers.begin(), frontiers.end(), sortcondition);
      }

      while(min_index<frontiers.size())
      {
        sort(frontiers.begin(), frontiers.end(), sortcondition);
        desiredPose.x = frontiers[min_index].mean.x;
        desiredPose.y = frontiers[min_index].mean.y;

        std::cout << "Closest frontier x and y coordinate are: " << desiredPose.x << " " << desiredPose.y << std::endl;

        // Calculating the equation of line perpendicular to current set of points

        int num_of_cells = frontiers[min_index].cells.size() - 1;
        // Slope of the line passing through the closest frontier
        float m = (frontiers[min_index].cells[num_of_cells].x - frontiers[min_index].cells[0].x) / (frontiers[min_index].cells[num_of_cells].y - frontiers[min_index].cells[0].y);
        // Find the slope of line perpendicular to original line
        m = -1 / m;

        // This line must pass through the centroid of the contour
        // Intercept of this line is
        float c = desiredPose.y - desiredPose.x * m;
        pose_xyt_t goalPose2;

        // First moving in postive x direction

        for (unsigned int x = 0; x < 10; x += 2)
        {
          goalPose2.x = desiredPose.x + x * map.metersPerCell();
          goalPose2.y = m * goalPose2.x + c;
          std::cout << "Goal2: " << goalPose2.x << " " << goalPose2.y << std::endl;

          if (planner.isValidGoal(goalPose2))
          {
            //std::cout<<"Valid goal.\n";
            robot_path_t path;
            path = planner.planPath(robotPose, goalPose2);
            //std::cout<<planner.isPathSafe(path) <<" "<<path.path_length<<std::endl;
            if (planner.isPathSafe(path) && path.path_length != 0)
            {
              std::cout << "A path to frontier is returned.\n";
              return path;
            }
          }
            } 
          


          // Second moving in negative x direction

          for (int x=-1; x>-11; x-=2){
            goalPose2.x = desiredPose.x+x*map.metersPerCell();
            goalPose2.y = m*goalPose2.x+c;
            std::cout<<"Goal2: "<<goalPose2.x<<" "<<goalPose2.y<<std::endl;

            if(planner.isValidGoal(goalPose2)){
              //std::cout<<"Valid goal.\n";
              robot_path_t path;
              path = planner.planPath(robotPose, goalPose2);
              //std::cout<<planner.isPathSafe(path) <<" "<<path.path_length<<std::endl;
              if(planner.isPathSafe(path) && path.path_length!=0){
                std::cout<<"A path to frontier is returned.\n";
                return path;
              }
            }
          }
          min_index++;
          std::cout<<"This frontier is not cool, checking next one .... " << std::endl;
      }
        // if closest not feasible, check next - 






    //     goalPose2.x  = desiredPose.x;
    //     goalPose2.y  = desiredPose.y;
    //   for (int l=0; l<L; ++l){
    //     for (int m=0; m<l; ++m){
    //       for (int n=0; n<l; ++n){
    //         goalPose2.x = goalPose.x + (m-l/2)*map.metersPerCell();
    //         goalPose2.y = goalPose.y + (n-l/2)*map.metersPerCell();
    //         //std::cout<<"Goal2: "<<goalPose2.x<<" "<<goalPose2.y<<std::endl;
    //         if(planner.isValidGoal(goalPose2)){
    //           //std::cout<<"Valid goal.\n";
    //           robot_path_t path;
    //           path = planner.planPath(robotPose, goalPose2);
    //           //std::cout<<planner.isPathSafe(path) <<" "<<path.path_length<<std::endl;
    //           if(planner.isPathSafe(path) && path.path_length!=0){
    //             std::cout<<"A path to frontier is returned.\n";
    //             return path;
    //           }
    //         }
    //       }
    //     }
    //   }
    // }
    std::cout<<"No path to any frontiers found.... this is not good\n";
    emptyPath.path.push_back(robotPose);
    return emptyPath;
}


bool is_frontier_cell(int x, int y, const OccupancyGrid& map)
{
    // A cell if a frontier if it has log-odds 0 and a neighbor has log-odds < 0

    // A cell must be in the grid and must have log-odds 0 to even be considered as a frontier
    if(!map.isCellInGrid(x, y) || (map(x, y) != 0))
    {
        return false;
    }

    const int kNumNeighbors = 4;
    const int xDeltas[] = { -1, 1, 0, 0 };
    const int yDeltas[] = { 0, 0, 1, -1 };

    for(int n = 0; n < kNumNeighbors; ++n)
    {
        // If any of the neighbors are free, then it's a frontier
        // Note that logOdds returns 0 for out-of-map cells, so no explicit check is needed.
        if(map.logOdds(x + xDeltas[n], y + yDeltas[n]) < 0)
        {
            return true;
        }
    }

    return false;
}


frontier_t grow_frontier(Point<int> cell, const OccupancyGrid& map, std::set<Point<int>>& visitedFrontiers)
{
    // Every cell in cellQueue is assumed to be in visitedFrontiers as well
    std::queue<Point<int>> cellQueue;
    cellQueue.push(cell);
    visitedFrontiers.insert(cell);

    // Use an 8-way connected search for growing a frontier
    const int kNumNeighbors = 8;
    const int xDeltas[] = { -1, -1, -1, 1, 1, 1, 0, 0 };
    const int yDeltas[] = {  0,  1, -1, 0, 1,-1, 1,-1 };

    frontier_t frontier;

    // Do a simple BFS to find all connected frontier cells to the starting cell
    while(!cellQueue.empty())
    {
        Point<int> nextCell = cellQueue.front();
        cellQueue.pop();

        // The frontier stores the global coordinate of the cells, so convert it first
        frontier.cells.push_back(grid_position_to_global_position(nextCell, map));

        // Check each neighbor to see if it is also a frontier
        for(int n = 0; n < kNumNeighbors; ++n)
        {
            Point<int> neighbor(nextCell.x + xDeltas[n], nextCell.y + yDeltas[n]);
            if((visitedFrontiers.find(neighbor) == visitedFrontiers.end())
                && (is_frontier_cell(neighbor.x, neighbor.y, map)))
            {
                visitedFrontiers.insert(neighbor);
                cellQueue.push(neighbor);
            }
        }
    }

    return frontier;
}
