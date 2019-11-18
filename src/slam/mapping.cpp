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

int metersToCellX(float x, OccupancyGrid &map)
{
    return std::floor(x * map.cellsPerMeter()) + (map.widthInCells() / 2);
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

  // Moving laser scan to interpolate current robot pose
  MovingLaserScan ml_scan(scan, last_pose_, pose, rayStride_);

	// This algorith is supposed to update the log odds associated with each cell //

	// Need to determine the cells through which the laser passes through for each ray of laser
	// Creating for loop to iterate through each laser array
	for (unsigned int i=0; i<ml_scan.size(); i++){
    adjusted_ray_t ad_ray = ml_scan.at(i);
    // Current robot pose in world frame
    float x0 = ad_ray.origin.x;
    float y0 = ad_ray.origin.y;

    float range = ad_ray.range;
    float theta = ad_ray.theta; // This is the global frame theta!
    cout<<i<<" "<<x0<<" "<<y0<<" "<<range<<" "<<theta<<endl;

	  // End of laser scan in world frame
    float x1 = x0 + range*cos(theta);
    float y1 = y0 + range*sin(theta);

		float dx = abs(x1-x0);
		float dy = abs(y1-y0);
		float sx = (x0<x1)? 1 : -1;
		float sy = (y0<y1)? 1 : -1;
		float err = dx-dy;
		float x = x0;
		float y = y0;

    //TODO: inifite loop here.
		while(x != x1 || y != y1){

		  // Updating the odds for each cell through which laser array passes
			CellOdds new_logOdd;
      //TODO: need to transform between global (meter) to occupancy grid (int) frame
      //TODO: clamp logOdds
			new_logOdd = map.logOdds(x, y)  - kMissOdds_;
			map.setLogOdds(x, y, new_logOdd);

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
    CellOdds new_logOdd;
    new_logOdd = map.logOdds(x1, y1)  +  kHitOdds_;
    map.setLogOdds(x1, y1, new_logOdd);
	}
  last_pose_ = pose;
}

// CellOdds inverse_sensor_model(int x, int y, ){
//
// 	float r = sqrt(dx^2 + dy^2);
// 	float phi = atan2(dy,dx)-pose.theta;
//
// 	int j=0; // Creating index for iterating through all the rays of laser
// 	float min=0.0;
// 	int min_index=0; // Index corresponding to k in inverse sensor model
//
// 	// function for finding index of laser ray
// 	while(j<scan.num_ranges)
// 	{
// 		float difference = abs(phi-scan.thetas[j]);
// 		if (difference<min){
// 			min=difference;
// 			min_index=j;
// 			j++;
// 	}
//
// 	// kMaxLaserDistance_ is private member of Mapping class
// 	if (r>std::min(kMaxLaserDistance_, zkt + alpha/2) || abs(phi-scan.thetas[k])>(2*M_PI/2)){
// 		return l0;
// 	}
//
// 	if (zkt < kMaxLaserDistance_ && abs(r - zkt)<alpha/2){
// 		return kHitOdds_;
// 	}
//
// 	if (r<=zkt)
// 		return kMissOdds_;
// }
//
//

// ITEMS UNKNOWN are
// 	zkt
// 	alpha
// 	lo
