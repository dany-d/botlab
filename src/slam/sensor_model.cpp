#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>


SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
    f_ = 0.9;
    rayStride_ = 7;
    obs_thres = 50;
}


double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    MovingLaserScan ml_scan(scan, sample.parent_pose, sample.pose, rayStride_);
    double likelihood = 0.0;
    for (unsigned int i=0; i<ml_scan.size(); i++){
      adjusted_ray_t ad_ray = ml_scan.at(i);
      float range = ad_ray.range;
      float theta = ad_ray.theta; // This is the global frame theta!
      //cout<<i<<" "<<x0<<" "<<y0<<" "<<range<<" "<<theta<<endl;

      // End of laser scan in world frame
      auto Cell1 = global_position_to_grid_cell(Point<float>(ad_ray.origin.x + range*cos(theta), ad_ray.origin.y + range*sin(theta)), map);
      int x1 = Cell1.x;
      int y1 = Cell1.y;
      int endLogOdd = map.logOdds(x1,y1);
      if (endLogOdd > obs_thres){
        likelihood += double((endLogOdd+127)/254);
        //std::cout<<"hit likelihood: "<<likelihood<<std::endl;
      }
      else{
        int x2 = x1 + floor(1.8*cos(theta));
        int y2 = y1 + floor(1.8*sin(theta));

        int x3 = x1 - floor(1.8*cos(theta));
        int y3 = y1 - floor(1.8*sin(theta));

        likelihood += f_*double((map.logOdds(x2,y2) +127 + map.logOdds(x3,y3)+127)/2/254);
        // std::cout<<"x1: "<<x1<<" y1: "<<y1<<std::endl;
        // std::cout<<"x2: "<<x2<<" y2: "<<y2<<std::endl;
        // std::cout<<"x3: "<<x3<<" y3: "<<y3<<std::endl;
        //std::cout<<"Miss likelihood: "<<likelihood<<std::endl;
      }
    }
    return likelihood;
}
