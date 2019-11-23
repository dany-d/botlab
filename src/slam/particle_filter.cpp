#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <cassert>
using namespace std;


ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{
    for (int i_num = 0; i_num < kNumParticles_; i_num++){
        posterior_[i_num].pose.x = rand()%5*0.01 + pose.x;
        posterior_[i_num].pose.y = rand()%5*0.01 + pose.y;
        posterior_[i_num].pose.theta = rand()%10*0.0154 + pose.theta;
        posterior_[i_num].parent_pose = pose;
        posterior_[i_num].weight = 1.0 /kNumParticles_;
    }
    posteriorPose_ = estimatePosteriorPose(posterior_);
    std::cout<<"pose x: "<<posteriorPose_.x<<" y: "<<posteriorPose_.y<<std::endl;
    std::cout<<"Hit: "<<1<<" "<<std::endl;
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
}


pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    if(hasRobotMoved)
    {
        std::cout<<"start!!!!"<<std::endl;
        auto prior = resamplePosteriorDistribution();
        std::cout<<"prior x:"<<prior[50].pose.x<<" y:"<<prior[50].pose.y<<std::endl;
        std::cout<<"prior x:"<<prior[150].pose.x<<" y:"<<prior[150].pose.y<<std::endl;
        std::cout<<"second"<<std::endl;
        auto proposal = computeProposalDistribution(prior);
        std::cout<<"proposal x:"<<proposal[50].pose.x<<" y:"<<proposal[50].pose.y<<std::endl;
        std::cout<<"proposal x:"<<proposal[150].pose.x<<" y:"<<proposal[150].pose.y<<std::endl;
        std::cout<<"third"<<std::endl;
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        std::cout<<"posterior x:"<<posterior_[50].pose.x<<" y:"<<posterior_[50].pose.y<<std::endl;
        std::cout<<"posterior x:"<<posterior_[150].pose.x<<" y:"<<posterior_[150].pose.y<<std::endl;
        std::cout<<"fourth"<<std::endl;
        // std::cout<<"posterior x: "<<posterior_[5].pose.x<<" y: "<<posterior_[5].pose.y<<" weight: "<<posterior_[5].weight<<std::endl;
        posteriorPose_ = estimatePosteriorPose(posterior_);
        std::cout<<"shout!!!!"<<std::endl;
    }
    // posteriorPose_.x += odometry.x;
    // posteriorPose_.y += odometry.y;
    // posteriorPose_.theta += odometry.theta;
    posteriorPose_.utime = odometry.utime;
    std::cout<<"pose x: "<<posteriorPose_.x<<" y: "<<posteriorPose_.y<<std::endl;
    // std::cout<<"Hit: "<<2<<" "<<std::endl;
    
    return posteriorPose_;
}


pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    // std::cout<<"Hit: "<<3<<" "<<std::endl;
    return posteriorPose_;
}


particles_t ParticleFilter::particles(void) const
{
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    // std::cout<<"Hit: "<<4<<" "<<std::endl;
    return particles;
}


std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    std::vector<particle_t> prior;
    double r = rand()%1000/(1000.0 * kNumParticles_);
    // std::cout<<"random: "<<r<<std::endl;
    double c = posterior_[0].weight;
    int i = 0;
    double U = 0.0;

    for (int m = 0; m < kNumParticles_; m++)
    {
        U = r + m / kNumParticles_;
        while (U > c)
        {
            i += 1;
            c += posterior_[i].weight;
        }
        prior.push_back(posterior_[i]);
    }
    // std::cout<<"Hit: "<<5<<" "<<std::endl;
    return prior;
}


std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    std::vector<particle_t> proposal;
    for (int i_num = 0; i_num < kNumParticles_; i_num++)
    {
        proposal.push_back(actionModel_.applyAction(prior[i_num]));
    }
    // std::cout<<"Hit: "<<6<<" "<<std::endl;
       
    return proposal;
}


std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid&   map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the 
    ///////////       particles in the proposal distribution
    std::vector<particle_t> posterior;
    double alpha = 0.0;
    for (int i_num = 0; i_num < kNumParticles_; i_num++)
    {
        posterior.push_back(proposal[i_num]);
        posterior[i_num].weight = sensorModel_.likelihood(proposal[i_num], laser, map);
        alpha += posterior[i_num].weight;
        // std::cout<<"likeli: "<<alpha<<std::endl;
    }

    for (int i_num = 0; i_num < kNumParticles_; i_num++)
    {
        posterior[i_num].weight /= alpha;
    }
    std::cout<<"likelihood: "<<alpha<<std::endl;
    // std::cout<<"Hit: "<<7<<" "<<std::endl;
    return posterior;
}


pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    pose_xyt_t pose;
    float x = 0.0;
    float y = 0.0;
    float theta_x = 0.0;
    float theta_y = 0.0;
    for (int i_num = 0; i_num < kNumParticles_; i_num++)
    {
        x += posterior[i_num].pose.x * posterior[i_num].weight;
        y += posterior[i_num].pose.y * posterior[i_num].weight;
        theta_x += std::cos(posterior[i_num].pose.theta) * posterior[i_num].weight;
        theta_y += std::sin(posterior[i_num].pose.theta) * posterior[i_num].weight;
    }
    pose.x = x;
    pose.y = y;
    pose.theta = std::atan2(theta_y,theta_x);
    // std::cout<<"pose x: "<<pose.x<<" y: "<<pose.y<<std::endl;
    // std::cout<<"Hit: "<<8<<" "<<std::endl;
    return pose;
}
