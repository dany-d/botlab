#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>


ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
}


void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    resetGrid(map);

    ///////////// TODO: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map.
    for(int i=0; i<width_; i++){
      for(int j=0; j<height_; j++){
        //std::cout<<i<<" "<<j<<" "<<findDistance(i,j, map) * metersPerCell_ <<std::endl;
        distance(i,j) = findDistance(i,j, map) * metersPerCell_;
      }
    }
}


float ObstacleDistanceGrid::findDistance(int x, int y, const OccupancyGrid& map){
  // this can probably be heavily optimized by keeping a set of occupied cells
  //Note: the distance is in grid scale (not meters)
  //std::cout<<"x, y: "<<x<<" "<<y<<std::endl;
  if (map.logOdds(x,y) == 0) return 0; //Cell unknown

  float min_distance = std::numeric_limits<float>::max();
  for (int i = 0; i < map.widthInCells(); ++i) {
    for (int j = 0; j < map.heightInCells(); ++j) {
      //std::cout<<"i, j: "<<i<<" "<<j<<std::endl;
      if (map.logOdds(i, j) >-5) { //occupied
        float temp_distance = sqrt(pow(x - i, 2) + pow(y - j, 2));
        if (temp_distance < min_distance)
        {
          min_distance = temp_distance;
        }
      }
    }
  }
  //std::cout<<x<<" "<<y<<" "<<min_distance * metersPerCell_<<std::endl;
  return min_distance;
}


bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0) && (x < width_) && (y >= 0) && (y < height_);
}


void ObstacleDistanceGrid::resetGrid(const OccupancyGrid& map)
{
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();

    // If the grid is already the correct size, nothing needs to be done
    if((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    {
        return;
    }

    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();

    cells_.resize(width_ * height_);
}
