#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>
#include <cmath> // Importing library for all the math function such as cos, atan2
#include <algorithm> // Importing library for using the min fucntion



// Intialization constructor for setting the range of the laser and the amount by which log odds needs to be updated
Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance), kHitOdds_(hitOdds), kMissOdds_(missOdds)
{
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////


	// This algorith is supposed to update the log odds associated with each cell //

	// The lidar scan and pose data is taken as const since we wouldn't want this data to be modified

	// Defining the variable to store the end location of laser ray
  	int x1=0;
	int y1=0;

	// Current origin of the robot with reference to global frame
	int x0=pose.x;
	int y0=pose.y;

	// Need to determine the cells through which the laser passes through for each ray of laser
	// Creating for loop to iterate through each laser array
	for (int i=0; i<scan.num_ranges; i++){

		// Determing x1 and y1 position in the robot's reference frame
		x1=scan.ranges[i]*cos(scan.thetas[i]);
		y1=scan.ranges[i]*sin(scan.thetas[i]);
		int dx = abs(x1-x0);
		int dy = abs(y1-y0);
		int sx = (x0<x1)? 1 : -1;
		int sy = (y0<y1)? 1 : -1;
		int err = dx-dy;
		int x = x0;
		int y = y0;

		while(x != x1 || y != y1){
		    
		    // Updating the odds for each cell through which laser array passes
			CellOdds new_logOdd;
			new_logOdd = map.logOdds(x, y) + inverse_sensor_model(dx, dy, scan.thetas[i] ) - l0;
			map.setLogOdds(x, y, new_logOdd);

			// Computing the next x,y cells through which laser passes using Breshenham's Algorithm
		    e2 = 2*err;
		    if (e2 >= -dy)
			    {
					err -= dy;
					x += sx; 
				}

		    if (e2 <= dx)
			    {
			    	err += dx
					y += sy 
				}	
	 	}
		 	
	}

}

CellOdds inverse_sensor_model(int x, int y, ){

	float r = sqrt(dx^2 + dy^2);
	float phi = atan2(dy,dx)-pose.theta;

	int j=0; // Creating index for iterating through all the rays of laser
	float min=0.0;
	int min_index=0; // Index corresponding to k in inverse sensor model

	// function for finding index of laser ray
	while(j<scan.num_ranges)
	{
		float difference = abs(phi-scan.thetas[j]);
		if (difference<min){
			min=difference;
			min_index=j;
			j++;
	}

	// kMaxLaserDistance_ is private member of Mapping class
	if (r>std::min(kMaxLaserDistance_, zkt + alpha/2) || abs(phi-scan.thetas[k])>(2*M_PI/2)){
		return l0;
	}

	if (zkt < kMaxLaserDistance_ && abs(r - zkt)<alpha/2){
		return kHitOdds_;
	}

	if (r<=zkt)
		return kMissOdds_;
}



// ITEMS UNKNOWN are
// 	zkt
// 	alpha
// 	lo




