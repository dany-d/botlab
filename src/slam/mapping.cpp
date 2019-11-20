#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>
#include <cmath> // Importing library for all the math function such as cos, atan2
#include <algorithm> // Importing library for using the min fucntion

using namespace std;

// Intialization constructor for setting the range of the laser and the amount by which log odds needs to be updated
Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance), kHitOdds_(hitOdds), kMissOdds_(missOdds)
{
}

int metersToCellX(float x, OccupancyGrid& map)
{
    return std::floor(x * map.cellsPerMeter()) + (map.widthInCells() / 2);
}

int metersToCellY(float x, OccupancyGrid& map)
{
    return std::floor(x * map.cellsPerMeter()) + (map.heightInCells() / 2);
}

void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////

  // Intialization
  // If there is no last pose, then we need to start at next timestamp	
  if(!started_){
    last_pose_ = pose;
    map.reset();
    started_ = true;
    return;
  }
  // last_pose_ = pose;
  // Moving laser scan to interpolate current robot pose
  MovingLaserScan ml_scan(scan, last_pose_, pose, rayStride_);
  int new_logOdd = 0;
	// This algorith is supposed to update the log odds associated with each cell //

	// Need to determine the cells through which the laser passes through for each ray of laser
	// Creating for loop to iterate through each laser array
	for (unsigned int i=0; i<ml_scan.size(); i++){
    adjusted_ray_t ad_ray = ml_scan.at(i);
  	// Current robot pose in world frame
  	int x0 = metersToCellX(ad_ray.origin.x, map);
  	int y0 = metersToCellY(ad_ray.origin.y, map);

  	float range = ad_ray.range;
  	float theta = ad_ray.theta; // This is the global frame theta!
  	//cout<<i<<" "<<x0<<" "<<y0<<" "<<range<<" "<<theta<<endl;
    if(range > kMaxLaserDistance_){
         continue;
      }
		// End of laser scan in world frame
    int x1 = metersToCellX(ad_ray.origin.x + range*cos(theta), map);
   	int y1 = metersToCellY(ad_ray.origin.y + range*sin(theta), map);
		int dx = abs(x1-x0);
		int dy = abs(y1-y0);
		int sx = (x0<x1)? 1 : -1;
		int sy = (y0<y1)? 1 : -1;
		int err = dx-dy;
		int x = x0;
		int y = y0;
		while(x != x1 || y != y1){

		  // Updating the odds for each cell through which laser array passes
			new_logOdd = map.logOdds(x,y)  - kMissOdds_;
      new_logOdd = new_logOdd < -127 ? -127: new_logOdd;
			map.setLogOdds(x,y, new_logOdd);

			// Computing the next x,y cells through which laser passes using Breshenham's Algorithm
	    int e2 = 2*err;
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
	 	new_logOdd = map.logOdds(x1,y1) +  kHitOdds_;
	 	new_logOdd = new_logOdd > 127 ? 127: new_logOdd;
    map.setLogOdds(x1,y1, new_logOdd);
    //cout<<"Hit: "<<x<<" "<<y<<" "<< int(new_logOdd)<<endl;
	}
  //cout<< int(map.logOdds(124,123)) <<endl;
  last_pose_ = pose;
}
