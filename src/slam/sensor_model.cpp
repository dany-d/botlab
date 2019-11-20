#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>


SensorModel::SensorModel(void)
{
    ///////// TODO: Tune this f_ and sensor_thres
    f_ = 0.8;
    rayStride_ = 1;
    sensor_thres = 50;
}


double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    MovingLaserScan ml_scan(scan, sample.parent_pose, sample.pose, rayStride_);
    int likelihood = 0;
    for (unsigned int i=0; i<ml_scan.size(); i++){
      adjusted_ray_t ad_ray = ml_scan.at(i);
      float range = ad_ray.range;
      float theta = ad_ray.theta; // This is the global frame theta!
      //cout<<i<<" "<<x0<<" "<<y0<<" "<<range<<" "<<theta<<endl;

      // End of laser scan in world frame
      int x1 = map.metersToCellX(ad_ray.origin.x + range*cos(theta));
      int y1 = map.metersToCellY(ad_ray.origin.y + range*sin(theta));
      int endLogOdd = map.logOdds(x1,y1);
      
      if (endLogOdd > sensor_thres){
        likelihood += endLogOdd;
      }
      else{
        int x2 = map.metersToCellX(ad_ray.origin.x + (range+map.metersPerCell()) *cos(theta));
        int y2 = map.metersToCellY(ad_ray.origin.y + (range+map.metersPerCell()) *sin(theta));

        int x3= map.metersToCellX(ad_ray.origin.x + (range-map.metersPerCell()) *cos(theta));
        int y3 = map.metersToCellY(ad_ray.origin.y + (range-map.metersPerCell()) *sin(theta));
        likelihood += f_*(map.logOdds(x2,y2) + map.logOdds(x3,y3));
      }
    }
    return likelihood;
}
