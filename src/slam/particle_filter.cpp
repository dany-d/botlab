#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <cassert>


ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{
    for (int i_num = 0; i < kNumParticles_; ++i){
        posterior_[i_num].pose.x = rand() % 5 * 0.01;
        posterior_[i_num].pose.y = rand() % 5 * 0.01;
        posterior_[i_num].pose.theta = 0 % 100 * 0.0154;
        posterior_[i_num].parent_pose = pose;
    }
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
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        posteriorPose_ = estimatePosteriorPose(posterior_);
    }
    
    posteriorPose_.utime = odometry.utime;
    
    return posteriorPose_;
}


pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


particles_t ParticleFilter::particles(void) const
{
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    std::vector<particle_t> prior;
    double r = std::rand() % 1000 / (1000 * kNumParticles_);
    double c = posterior_[0].weight;
    int i = 0;
    double U = 0;

    for (int m = 0; m < kNumParticles_; ++m)
    {
        U = r + (m - 1) / kNumParticles_;
        while (U > c)
        {
            i += 1;
            c += posterior_[i].weight;
        }
        prior.push_back(posterior_[i]);
    }
    return prior;
}


std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    std::vector<particle_t> proposal;
    for (int i_num = 0; i < kNumParticles_; ++i){
        proposal[i_num] = (actionModel_.applyAction(prior[i_num]));
    }
       
    return proposal;
}


std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid&   map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the 
    ///////////       particles in the proposal distribution
    std::vector<particle_t> posterior;
    double alpha = 0;
    for (auto &iter : proposal)
    {
        particle_t post = iter;
        post.weight = pow(2, sensorModel_.likelihood(iter, laser, map));
        alpha += post.weight;
        posterior.push_back(post);
    }

    for (auto &iter : posterior)
    {
        iter.weight /= alpha;
    }
    return posterior;
}


pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    pose_xyt_t pose;
    float x = 0;
    float y = 0;
    float theta = 0;
    for (auto &iter : posterior)
    {
        x += iter.pose.x * iter.weight;
        y += iter.pose.y * iter.weight;
        theta += iter.pose.theta * iter.weight;
    }
    pose.x = x;
    pose.y = y;
    pose.theta = theta;
    return pose;
}
