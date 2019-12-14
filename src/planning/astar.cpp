#include "queue"
#include "unordered_set"
#include "stack"
#include "algorithm"

#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <common/point.hpp>
#include <planning/Node.hpp>






robot_path_t search_for_path(pose_xyt_t start,
                             pose_xyt_t goal,
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    //std::cout<<"Astar start -- search start (global): "<<start.x<<" "<<start.y<<" goal: "<<goal.x<<" "<<goal.y<<std::endl;
    //std::cout<<"Map INFO (w/h): "<<distances.widthInCells()<<" "<<distances.heightInCells()<<" "<<distances.widthInMeters()<<" "<<distances.heightInMeters()<<std::endl;
    //std::cout<<"Origin: "<<distances.originInGlobalFrame().x<<" "<<distances.originInGlobalFrame().y<<std::endl;
    robot_path_t path;
    path.path_length = 0;
    path.utime = start.utime;
    //First node in path
    int width = distances.widthInCells();
    int height = distances.heightInCells();
    float metersPerCell = distances.metersPerCell();

    //Initialize search priority queue and visted list (using unordered map)
    std::priority_queue<Node *, std::vector<Node *>, Compare> pq;
    std::unordered_set<Node *, hash, equal> open_set;

    //Initialize start point in cell coordinates
    Point<int> end_point = distances.poseToCell(goal.x, goal.y);
    Node dest_node = Node(end_point, nullptr,0);
    Point<int> start_point = distances.poseToCell(start.x, start.y);
    Node* start_node = new Node(start_point, nullptr,0);
    start_node->g_score = obstacleCost(*start_node, distances, params);
    start_node->f_score =  calculateHscore(*start_node, dest_node, distances, params) + start_node->g_score;
    open_set.insert(start_node);
    std::cout<<"search start (Cell): "<<start_node->n.x<<" "<<start_node->n.y<<" goal: "<<dest_node.n.x<<" "<<dest_node.n.y<<std::endl;

    pq.push(start_node);
    //visited_list.insert(start_node.n);
    const Node* cur = start_node;
    //For debug
    //int count = 0;
    while(!pq.empty() && !isDestinationReached(*cur, dest_node)){
      //std::cout<<"pq size: "<<pq.size()<<" open size: "<<open_set.size()<<std::endl;
      cur = pq.top();
      //std::cout<<"current: "<<cur->n.x<<" "<<cur->n.y<<" "<<cur->f_score<<std::endl;//" parent: "<<cur->p->n.x<<" "<<cur->p->n.y<<std::endl;

      //Using 4-neighbour connection here -- can be changed to 8-neighbour
      std::vector<Point<int>> neighbours;
      neighbours.resize(8);
      neighbours[0] = Point<int>(std::max(0, cur->n.x - 1), cur->n.y);
      neighbours[3] = Point<int>(cur->n.x, std::max(0, cur->n.y - 1));
      neighbours[2] = Point<int>(std::min(cur->n.x + 1, width), cur->n.y);
      neighbours[1] = Point<int>(cur->n.x, std::min(cur->n.y + 1, height));
      neighbours[7] = Point<int>(std::max(0, cur->n.x - 1), std::max(0, cur->n.y - 1));
      neighbours[6] = Point<int>(std::max(0, cur->n.x - 1), std::min(cur->n.y + 1, height));
      neighbours[5] = Point<int>(std::min(cur->n.x + 1, width), std::max(0, cur->n.y - 1));
      neighbours[4] = Point<int>(std::min(cur->n.x + 1, width), std::min(cur->n.y + 1, height));


      for(unsigned int i=0; i<neighbours.size(); i++){
        if (distances(neighbours[i].x, neighbours[i].y)>params.minDistanceToObstacle){
          //std::cout<<"Neighbour #"<<i<<": "<<neighbours[i].x<<" "<<neighbours[i].y<<std::endl;
          Node* n = new Node(neighbours[i], cur, cur->g_score+metersPerCell);
          n->g_score -= obstacleCost(*cur, distances, params);
          //std::cout<<n->n.x<<" "<<n->n.y<<" "<<distances(n->n.x, n->n.y)<<" Scores (from parent, h): "<<n->g_score<<" "<<calculateHscore(*n, dest_node, distances, params)<<std::endl;

          n->g_score += obstacleCost(*n, distances, params);
          if(i>=4){
            n->g_score += (sqrt(2) - 1)*metersPerCell;
          }
          auto iter = open_set.find(n);
          if(iter==open_set.end()){
            n->f_score = n->g_score + calculateHscore(*n, dest_node, distances, params);
            open_set.insert(n);
            pq.push(n);
            //std::cout<<"Top: "<<pq.top()->n.x<<" "<<pq.top()->n.y<<" "<<pq.top()->f_score<<std::endl;
            //std::cout<<n->n.x<<" "<<n->n.y<<" "<<distances(n->n.x, n->n.y)<<" Scores (g h): "<<n->g_score<<" "<<calculateHscore(*n, dest_node, distances, params)<<std::endl;
          }
          else{
            if(n->g_score < (*iter)->g_score){
              (*iter)->p = cur;
              (*iter)->f_score = n->g_score + calculateHscore(*(*iter), dest_node, distances, params);
              (*iter)->g_score = n->g_score;
              //std::cout<<"Updated Scores (f g h): "<<(*iter)->f_score<<" "<<(*iter)->g_score<<" "<<calculateHscore(*n,dest_node, distances, params)<<std::endl;
            }
         }
        }
      }
      //count++;
      pq.pop();
    }

    pose_xyt_t first;
    first.x = goal.x;
    first.y = goal.y;
    int cnt=0;
    if(isDestinationReached(*cur, dest_node)){
      while(cur!=nullptr){
          //if(i==sparser){
            pose_xyt_t second;
            Point<float> p = distances.cellToPose(cur->n.x, cur->n.y);
            second.x = p.x;
            second.y = p.y;
            //cur_pose.theta = 0;
            //std::cout<<"First Pose: "<<first.x<<" "<<first.y<<std::endl;
            //std::cout<<"Second Pose: "<<second.x<<" "<<second.y<<std::endl;
            cnt++;
            if(!IsPathFree(first, second, distances, params) || cnt==7){
              path.path.push_back(second);
              first.x = second.x;
              first.y = second.y;
              cnt=0;
            }
        cur = cur->p;
      }
      path.path.push_back(start);
      //path.path.push_back(start);
      std::reverse(path.path.begin(), path.path.end());
      path.path.push_back(goal);
      path.path_length = path.path.size();
    }
    else{
      std::cout<<"No valid path.\n";
    }

    //destory all new nodes
    while(!open_set.empty()){
      auto it = open_set.begin();
      open_set.erase(it);
      delete *it;
    }

    std::cout<<"Path returned. Length: "<<path.path_length<<std::endl;
    for(unsigned int i=0; i<path.path.size(); ++i){
      std::cout<<path.path[i].x<<" ";
    }
    std::cout<<std::endl;
    //std::cout<<(*iter)->n.x<<" "<<(*iter)->n.y<<" "<<(*iter)->f_score<<std::endl;
    return path;
}


bool IsPathFree(pose_xyt_t first, pose_xyt_t second,
                const ObstacleDistanceGrid& distances, const SearchParams& params){
    float x0 = first.x;
    float y0 = first.y;
    float x1 = second.x;
    float y1 = second.y;
    float minDist =  params.minDistanceToObstacle + distances.metersPerCell();
    float dx = fabs(x1-x0);
    float dy = fabs(y1-y0);
    float sx = (x0<x1)? distances.metersPerCell() : -distances.metersPerCell();
    float sy = (y0<y1)? distances.metersPerCell() : -distances.metersPerCell();
    float err = dx-dy;
    float x = x0;
    float y = y0;
    while(fabs(x-x1) > distances.metersPerCell()/2.0 || fabs(y-y1) > distances.metersPerCell()/2.0 ){
      Point<int> p;
      p = distances.poseToCell(x,y);
      if (distances(p.x, p.y) < minDist){
        //std::cout<<p.x<<" "<<p.y<<" "<<distances(p.x, p.y)<<std::endl;
        return false;
      }
      // Computing the next x,y cells through which laser passes using Breshenham's Algorithm
      float e2 = 2*err;
      if (e2 >= -dy)
          {
          err -= dy;
          x += sx;
        }

      if (e2 <= dx)
          {
            err += dx;
          y += sy;
        }
    }
    return true;
}


bool isDestinationReached(const Node &n, const Node &dest){
  return n.n == dest.n;
}


float calculateHscore(const Node &n, const Node &dest,
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params){
  float h = distance_between_points(n.n, dest.n);
  h *= distances.metersPerCell();
  //h*=0.8;
  return h;
}

float obstacleCost(const Node &n, const ObstacleDistanceGrid& distances, const SearchParams& params){
   float cellDistance = distances(n.n.x, n.n.y);
   // /std::cout<<cellDistance<<" "<<n.n.x<<" "<< n.n.y<<std::endl;
   if(cellDistance < params.maxDistanceWithCost){
      //std::cout<<n.g_score<<" "<<n.f_score<<" "<<pow(params.maxDistanceWithCost - cellDistance, params.distanceCostExponent)<<std::endl;
      return pow(params.maxDistanceWithCost - cellDistance, params.distanceCostExponent);
      }
   else{
     return 0;
   }
}
