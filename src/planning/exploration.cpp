#include <planning/exploration.hpp>
#include <planning/frontiers.hpp>
#include <planning/planning_channels.h>
#include <common/grid_utils.hpp>
#include <common/timestamp.h>
#include <mbot/mbot_channels.h>
#include <slam/slam_channels.h>
#include <fstream>
#include <iostream>
#include <queue>
#include <unistd.h>
#include <cassert>
#include <vector> 

const float kReachedPositionThreshold = 0.05f;  // must get within this distance of a position for it to be explored

// Define an equality operator for poses to allow direct comparison of two paths
bool operator==(const pose_xyt_t& lhs, const pose_xyt_t& rhs)
{
    return (lhs.x == rhs.x) && (lhs.y == rhs.y) && (lhs.theta == rhs.theta);
}

float rounditup(float val)
{
    if (val < 0)
    {
        return ceil(val * 100 - 0.5) / 100;
    }
    return floor(val * 100 + 0.5) / 100;
}

Exploration::Exploration(int32_t teamNumber,
                         lcm::LCM* lcmInstance)
: teamNumber_(teamNumber)
, state_(exploration_status_t::STATE_INITIALIZING)
, haveNewPose_(false)
, haveNewMap_(false)
, haveHomePose_(false)
, lcmInstance_(lcmInstance)
, pathReceived_(true)
, haveNewBlocks_(false)
, new_block_pickup_status(false)
// , currentMap_(10.0f,10.0f, 0.05f) // CHANGED
{
    assert(lcmInstance_);   // confirm a nullptr wasn't passed in
    
    lcmInstance_->subscribe(SLAM_MAP_CHANNEL, &Exploration::handleMap, this);
    lcmInstance_->subscribe(SLAM_POSE_CHANNEL, &Exploration::handlePose, this);
    lcmInstance_->subscribe(MESSAGE_CONFIRMATION_CHANNEL, &Exploration::handleConfirmation, this);
    lcmInstance_->subscribe(MBOT_ARM, &Exploration::handleBlock, this);
    lcmInstance_->subscribe(ARM_GRAB_BLOCK, &Exploration::handleBlockConfirmation, this);
    
    // Send an initial message indicating that the exploration module is initializing. Once the first map and pose are
    // received, then it will change to the exploring map state.
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_INITIALIZING;
    status.status = exploration_status_t::STATUS_IN_PROGRESS;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    // std::cout<<currentMap_.loadFromFile("somethingelse.map")<<"MAP LOADED";
    MotionPlannerParams params;
    params.robotRadius = 0.17;
    planner_.setParams(params);
}


bool Exploration::exploreEnvironment()
{
    while((state_ != exploration_status_t::STATE_COMPLETED_EXPLORATION) 
        && (state_ != exploration_status_t::STATE_FAILED_EXPLORATION))
    {
        // If data is ready, then run an update of the exploration routine
        if(isReadyToUpdate())
        {
            std::cout<<"RUNNING EXPLORATION SALMAN\n";
            runExploration();
        }
        // Otherwise wait a bit for data to arrive
        else
        {
            usleep(10000);
        }
    }
    
    // If the state is completed, then we didn't fail
    return state_ == exploration_status_t::STATE_COMPLETED_EXPLORATION;
}

void Exploration::handleMap(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const occupancy_grid_t* map)
{

    std::lock_guard<std::mutex> autoLock(dataLock_);
    // std::cout<<"New MAP received";
    incomingMap_.fromLCM(*map);
    haveNewMap_ = true;
}


void Exploration::handlePose(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const pose_xyt_t* pose)
{

    std::lock_guard<std::mutex> autoLock(dataLock_);
    incomingPose_ = *pose;
    // std::cout<<"New Pose received";
    haveNewPose_ = true;
}

void Exploration::handleBlock(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const mbot_arm_block_t* blocklist) 
// SAL CHANGE
{

    std::lock_guard<std::mutex> autoLock(dataLock_);
    incomingblocklist_ = *blocklist;
    std::cout<<"INCOMING BLOCK COORDINATE"<<incomingblocklist_.pose.x<<"\n";
    std::cout<<"INCOMING BLOCK COORDINATE"<<incomingblocklist_.pose.y<<"\n";

    haveNewBlocks_ = true;
}

void Exploration::handleConfirmation(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const message_received_t* confirm)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    if(confirm->channel == CONTROLLER_PATH_CHANNEL && confirm->creation_time == most_recent_path_time) pathReceived_ = true;
}

void Exploration::handleBlockConfirmation(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const message_received_t* confirm)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    if(confirm->channel == ARM_GRAB_BLOCK) 
        new_block_pickup_status = true;
}

bool Exploration::isReadyToUpdate(void)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    return (haveNewMap_ && (haveNewPose_ || haveNewBlocks_ || new_block_pickup_status));
}


void Exploration::runExploration(void)
{
    assert(isReadyToUpdate());
    
    copyDataForUpdate();
    executeStateMachine();
}


void Exploration::copyDataForUpdate(void)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    
    // Only copy the map if a new one has arrived because it is a costly operation
    if(haveNewMap_)
    {
        currentMap_ = incomingMap_;
        haveNewMap_ = false;
    }

    if(new_block_pickup_status)
    {
        block_pickup_status = new_block_pickup_status;
        new_block_pickup_status = false;
    }
    
    // Always copy the pose because it is a cheap copy
    currentPose_ = incomingPose_;
    haveNewPose_ = false;

    if(haveNewBlocks_)
    {
        currentblocklist_ = incomingblocklist_;
        haveNewBlocks_ = false;
    }

    // The first pose received is considered to be the home pose
    if(!haveHomePose_)
    {
        homePose_ = incomingPose_;
        haveHomePose_ = true;
        std::cout << "INFO: Exploration: Set home pose:" << homePose_.x << ',' << homePose_.y << ',' 
            << homePose_.theta << '\n';
    }
}



void Exploration::executeStateMachine(void)
{
    bool stateChanged = false;
    int8_t nextState = state_;
    
    // Save the path from the previous iteration to determine if a new path was created and needs to be published
    robot_path_t previousPath = currentPath_;
    
    // Run the state machine until the state remains the same after an iteration of the loop
    do
    {
        switch(state_)
        {
            case exploration_status_t::STATE_INITIALIZING:
                nextState = executeInitializing();
                break;
            case exploration_status_t::STATE_EXPLORING_MAP:
                nextState = executeExploringMap(stateChanged);
                break;

            case exploration_status_t::STATE_DETECT_BLOCKS:
                nextState = executeBlockDetection(stateChanged);
                break;

            case exploration_status_t::STATE_PLAN_TO_GRAB_LOC:
                nextState = executeGrabPlanner(stateChanged);
                break;

            case exploration_status_t::STATE_GRAB_BLOCK:
                nextState = executeGrabBlock(stateChanged);
                break;

            case exploration_status_t::STATE_DROP_BLOCK:
                nextState = executeDropBlock(stateChanged);
                break;

            // Add State for if the block was not detected
                
            case exploration_status_t::STATE_RETURNING_HOME:
                nextState = executeReturningHome(stateChanged);
                break;

            case exploration_status_t::STATE_COMPLETED_EXPLORATION:
                nextState = executeCompleted(stateChanged);
                break;
                
            case exploration_status_t::STATE_FAILED_EXPLORATION:
                nextState = executeFailed(stateChanged);
                break;
        }
        
        stateChanged = nextState != state_;
        state_ = nextState;
        
    } while(stateChanged);

    //if path confirmation was not received, resend path
    if(!pathReceived_)
    {
        std::cout << "the current path was not received by motion_controller, attempting to send again:\n";

        std::cout << "timestamp: " << currentPath_.utime << "\n";

        for(auto pose : currentPath_.path){
            std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
        }std::cout << "\n";

        lcmInstance_->publish(CONTROLLER_PATH_CHANNEL, &currentPath_);
    }


    // TODO: Add confirmation for block position

    //if path changed, send current path
    if(previousPath.path != currentPath_.path)
    { 

        std::cout << "INFO: Exploration: A new path was created on this iteration. Sending to Mbot:\n";

        std::cout << "path timestamp: " << currentPath_.utime << "\npath: ";

        for(auto pose : currentPath_.path){
            std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
        }std::cout << "\n";

        lcmInstance_->publish(CONTROLLER_PATH_CHANNEL, &currentPath_);

        pathReceived_ = false;
        most_recent_path_time = currentPath_.utime;
    }

}


int8_t Exploration::executeInitializing(void)
{
    /////////////////////////   Create the status message    //////////////////////////
    // Immediately transition to exploring once the first bit of data has arrived
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_INITIALIZING;
    status.status = exploration_status_t::STATUS_COMPLETE;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    return exploration_status_t::STATE_DETECT_BLOCKS;
}


int8_t Exploration::executeExploringMap(bool initialize)
{
        //////////////////////// TODO: Implement your method for exploring the map ///////////////////////////
    /*
    * NOTES:
    *   - At the end of each iteration, then (1) or (2) must hold, otherwise exploration is considered to have failed:
    *       (1) frontiers_.empty() == true      : all frontiers have been explored as determined by find_map_frontiers()
    *       (2) currentPath_.path_length > 1 : currently following a path to the next frontier
    * 
    *   - Use the provided function find_map_frontiers() to find all frontiers in the map.
    *   - You will need to implement logic to select which frontier to explore.
    *   - You will need to implement logic to decide when to select a new frontier to explore. Take into consideration:
    *       -- The map is evolving as you drive, so what previously looked like a safe path might not be once you have
    *           explored more of the map.
    *       -- You will likely be able to see the frontier before actually reaching the end of the path leading to it.
    */

    int numBlock = 4;
    int block_itr = 0;
    std::vector<pose_xyt_t> desiredposes;
    desiredposes[0].x = 0;
    desiredposes[0].y = 0;
    desiredposes[0].theta = 0;

    desiredposes[1].x = 0;
    desiredposes[1].y = 0;
    desiredposes[1].theta = 0;

    desiredposes[2].x = 0;
    desiredposes[2].y = 0;
    desiredposes[2].theta = 0;

    desiredposes[3].x = 0;
    desiredposes[3].y = 0;
    desiredposes[3].theta = 0;


    planner_.setMap(currentMap_);
    frontiers_ = find_map_frontiers(currentMap_, currentPose_);
    if (block_itr < 4)
    {
        std::cout << "Block num: " << block_itr << std::endl;
        if (!planner_.isPathSafe(currentPath_))
        {
            // currentPath_ = plan_path_to_frontier(frontiers_, currentPose_, currentMap_, planner_);
            currentPath_ = planner_.planPath(currentPose_, desiredposes[block_itr]);
            // currentTarget_ = currentPath_.path[currentPath_.path_length - 1];
        }
        ++block_itr;
    }

    most_recent_path_time = currentPath_.utime;
                                
    /////////////////////////////// End student code ///////////////////////////////
    
    /////////////////////////   Create the status message    //////////////////////////
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_EXPLORING_MAP;
    
    // If no frontiers remain, then exploration is complete
    if(block_itr==4)
    {
        status.status = exploration_status_t::STATUS_COMPLETE;
    }
    // Else if there's a path to follow, then we're still in the process of exploring
    else if(currentPath_.path.size() > 1)
    {
        status.status = exploration_status_t::STATUS_IN_PROGRESS;
    }
    // Otherwise, there are frontiers, but no valid path exists, so exploration has failed
    else
    {
        status.status = exploration_status_t::STATUS_FAILED;
    }
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    ////////////////////////////   Determine the next state    ////////////////////////
    switch(status.status)
    {
        // Don't change states if we're still a work-in-progress
        case exploration_status_t::STATUS_IN_PROGRESS:
            return exploration_status_t::STATE_EXPLORING_MAP;
            
        // If exploration is completed, then head home
        case exploration_status_t::STATUS_COMPLETE:
            return exploration_status_t::STATE_RETURNING_HOME;
            
        // If something has gone wrong and we can't reach all frontiers, then fail the exploration.
        case exploration_status_t::STATUS_FAILED:
            return exploration_status_t::STATE_FAILED_EXPLORATION;
            
        default:
            std::cerr << "ERROR: Exploration::executeExploringMap: Set an invalid exploration status. Exploration failed!";
            return exploration_status_t::STATE_FAILED_EXPLORATION;
    }

}

int8_t Exploration::executeBlockDetection(bool initialize)
{
    // Publish on channel to camera to detect block
    mbot_arm_cmd_t detectblock;
    detectblock.utime = utime_now();
    detectblock.mbot_cmd = 1;

    std::cout<<"executeBlockDetection: Command sent to camera to detect blocks\n";
    lcmInstance_->publish(COMMAND_ARM, &detectblock);

    // Return type may not be correct
    return exploration_status_t::STATE_PLAN_TO_GRAB_LOC;
}

int8_t Exploration::executeGrabPlanner(bool initialize)
{
    // mbot_arm_block_t block_to_pick=currentblocklist_;
    // std::cout<<"\n Block to pick up from location: "<<block_to_pick.pose.x<<"  "<<block_to_pick.pose.y<<"\n";

    planner_.setMap(currentMap_);
    // frontiers_ = find_map_frontiers(currentMap_, currentPose_);

    mbot_arm_block_t block_to_pick = incomingblocklist_;

    // Only execute below commands if number of blocks is > 0
    float transform[3][3];
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            transform[i][j] = 0;
        }
    }
    std::cout << "currentpose= " << currentPose_.theta << std::endl;

    float rot_angle= -currentPose_.theta;

    // if ((currentPose_.theta <= M_PI / 2) && (currentPose_.theta >=0)){
    //     rot_angle = M_PI / 2 - currentPose_.theta;
    // }
    // if(currentPose_.theta>M_PI/2){
    //     rot_angle=currentPose_.theta-M_PI/2;
    // }
    // else if (currentPose_.theta<0){
    //     rot_angle = M_PI/2 + fabs(currentPose_.theta);
    // }
    


    transform[0][0] = cos(rot_angle);
    transform[0][1] = -sin(rot_angle);
    transform[0][2] = currentPose_.x;
    transform[1][0] = sin(rot_angle);
    transform[1][1] = cos(rot_angle);
    transform[1][2] = currentPose_.y;
    transform[2][2] = 1;

    std::cout << "\n"
              << "Transform Matrix: "
              << "\n";
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
        {
            std::cout << " " << transform[i][j];
            if (j == 3 - 1)
                std::cout << "\n";
        }

    std::cout << "\n"
              << "Block Matrix: "
              << "\n";
    float block_coords[3][1] = {{-block_to_pick.pose.y},
                                {block_to_pick.pose.x},
                                {1}};

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 1; ++j)
        {
            std::cout << " " << block_coords[i][j];
            if (j == 1 - 1)
                std::cout << "\n";
        }

    // float block_coords[3][1]={{1},
    //                             {1}, //0.41199721
    //                             {1}};

    // Multiplying the two matrices together

    float multi[3][1];

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 1; j++)
        {
            multi[i][j] = 0;
        }
    }

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 1; ++j)
        {
            for (int k = 0; k < 3; ++k)
            {
                multi[i][j] += transform[i][k] * block_coords[k][j];
            }
            // if(multi[i][j]>1e3 || multi[i][j]<1e-3 )
            //     multi[i][j]=0;
        }
    // Displaying the multiplication of two matrix.
    std::cout << "\n"
              << "Output Matrix: "
              << "\n";
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 1; ++j)
        {
            std::cout << " " << multi[i][j];
            if (j == 1 - 1)
                std::cout << "\n";
        }
    pose_xyt_t desiredpose;
    desiredpose.x = rounditup(multi[0][0]);
    desiredpose.y = rounditup(multi[1][0]);
    desiredpose.theta = 0;

    std::cout << "desiredPose.x" << desiredpose.x << ","
              << "desiredpose.y" << desiredpose.y << std::endl;

    std::cout << "1\n";
    currentPath_ = planner_.planPath(currentPose_, desiredpose);
    //currentTarget_ = currentPath_.path[currentPath_.path_length - 1];
    std::cout << "Printing the path to grab the block\n";
    for (int i; i < path.path.size(); ++i)
    {
        std::cout << path.path[i].x << "," << path.path[i].y << std::endl;
    }
    currentPath_.path.pop_back();

    std::cout << "2\n";

    most_recent_path_time = currentPath_.utime;

    // if (frontiers_.size() > 0)
    // {
    //     std::cout << "Number of frontiers: " << frontiers_.size() << std::endl;
    //     if (!planner_.isPathSafe(currentPath_) || currentPath_.path_length == 0 ||
    //         fabs(currentPose_.x - currentTarget_.x) + fabs(currentPose_.y - currentTarget_.y) < kReachedPositionThreshold)
    //     { // frontiers_.size()!=prev_frontier_size){
    //         prev_frontier_size = frontiers_.size();
    //         currentPath_ = plan_path_to_frontier(frontiers_, currentPose_, currentMap_, planner_);
    //         currentTarget_ = currentPath_.path[currentPath_.path_length - 1];
    //     }
    // }


    /////////////////////////////// End student code ///////////////////////////////

    /////////////////////////   Create the status message    //////////////////////////
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_EXPLORING_MAP;

    // If no frontiers remain, then exploration is complete
    if (frontiers_.empty())
    {
        status.status = exploration_status_t::STATUS_COMPLETE;
    }
    // Else if there's a path to follow, then we're still in the process of exploring
    else if (currentPath_.path.size() > 1)
    {
        status.status = exploration_status_t::STATUS_IN_PROGRESS;
    }
    // Otherwise, there are frontiers, but no valid path exists, so exploration has failed
    else
    {
        status.status = exploration_status_t::STATUS_FAILED;
    }

    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);

    ////////////////////////////   Determine the next state    ////////////////////////
    switch (status.status)
    {
    // Don't change states if we're still a work-in-progress
    case exploration_status_t::STATUS_IN_PROGRESS:
        return exploration_status_t::STATE_EXPLORING_MAP;

    // If exploration is completed, then head home
    case exploration_status_t::STATUS_COMPLETE:
        return exploration_status_t::STATE_RETURNING_HOME;

    // If something has gone wrong and we can't reach all frontiers, then fail the exploration.
    case exploration_status_t::STATUS_FAILED:
        return exploration_status_t::STATE_FAILED_EXPLORATION;

    default:
        std::cerr << "ERROR: Exploration::executeExploringMap: Set an invalid exploration status. Exploration failed!";
        return exploration_status_t::STATE_FAILED_EXPLORATION;
    }
}


// int8_t Exploration::executeGrabPlanner(bool initialize)
// {
    
//     mbot_arm_block_t block_to_pick=currentblocklist_;
//     // currentblocklist_.utime=utime_now();
//     // currentblocklist_.num_blocks

//     // Only execute below commands if number of blocks is > 0
//     float transform[3][3];
//     for (int i=0; i<3; i++){
//         for (int j=0; j<3; j++){
//             transform[i][j]=0;
//         }
//     }

//     transform[0][0]=cos(currentPose_.theta);
//     transform[0][1]=sin(currentPose_.theta);
//     transform[0][2]=currentPose_.x;
//     transform[1][0]=-sin(currentPose_.theta);
//     transform[1][1]=cos(currentPose_.theta);
//     transform[1][2]=currentPose_.y;
//     transform[2][2]=1;

//     // float block_coords[3][1]={{block_to_pick.pose.y},
//     //                           {block_to_pick.pose.x},
//     //                           {1}};

//     float block_coords[3][1]={{1},
//                               {1}, //0.41199721
//                               {1}};

//     // Multiplying the two matrices together

//     float multi[1][3];

//     for(int i = 0; i < 3; ++i)
//       for(int j = 0; j < 1; ++j)
//         for(int k = 0; k < 3; ++k)
//         {
//             multi[i][j] += transform[i][k] * block_coords[k][j];
//         }
//     // Displaying the multiplication of two matrix.
//     std::cout <<"\n"<< "Output Matrix: " <<"\n";
//     for(int i = 0; i < 3; ++i)
//         for(int j = 0; j < 1; ++j)
//         {
//             std::cout << " " << multi[i][j];
//             if(j == 1-1)
//                 std::cout << "\n";
//         }

//     multi[0][0]=1.0;
//     multi[1][0]=0.0;


//     // if (!block_to_pick.empty()){
//         // int8_t n_blocks = detectblock.num_blocks;
//         // Taking the first block in the list
//         // Should probably change to the nearest block in the list
//         std::cout<<"executeGrabPlanner: block available for pick up\n";
//         // blockPose_ = block_to_pick.pose;

//         // std::cout<<"Block to pick is at location"<<blockPose_.x<<"  "<<blockPose_.y<<"\n";
//         // Added by Salman
//         // The block pose is with reference to the robot camera frame
//         // Need to convert that to the global frame for the planner
        
//         pose_xyt_t desiredPose;

//         // Go to the block location (Ideally we want some distance away from block location)
//         // desiredPose.x=multi[0][0];
//         // desiredPose.y=multi[1][0];
//         desiredPose.x=1.0;
//         desiredPose.y=0.0;
//         desiredPose.theta=0;
//         // desiredPose.utime=utime_now();

//         std::cout<<"desiredPose.x"<<desiredPose.x<<"\n"<<"desiredPose.y"<<desiredPose.y<<"\n"<<"desiredPose.theta"<<desiredPose.theta<<"\n";

//         // bool a=currentMap_.loadFromFile("somethingelse.map");

//         // if (a)
//         //     std::cout<<"MAP LOADED FROM LOCALIZATION\n";
//         // TODO: Check if the path is also safe
//         std::cout<<"1\n";
//         planner_.setMap(currentMap_);
//         std::cout<<"2\n";
//         // Point<float> goalpose(desiredPose.x, desiredPose.y);
//         // std::cout<<"3";
//         // ObstacleDistanceGrid distances_ = planner_.obstacleDistances();
//         // std::cout<<"4";
//         // float obsdist = distances_(distances_.poseToCell(goalpose.x, goalpose.y).x, distances_.poseToCell(goalpose.x, goalpose.y).y);
//         //std::cout << "obsdist goal: " << distances_.poseToCell(goalpose.x, goalpose.y).x<<" "<< distances_.poseToCell(goalpose.x, goalpose.y).y<< obsdist << std::endl;
//         currentPath_=planner_.planPath(currentPose_, desiredPose);
//         std::cout<<"3\n";
        
//         // std::cout<<"after the planner";
//         path = currentPath_;
//         std::cout<<"4\n";

//         // int itr = path.path.size();
//         // while (itr != 1)
//         // {
//         //     // int a = path.path[itr].x;
//         //     if(sqrt((path.path[itr].x - blockPose_.x)*(path.path[itr].x - blockPose_.x) + (path.path[itr].y - blockPose_.y)*(path.path[itr].y - blockPose_.y)) < 0.15){
//         //         --itr;
//         //     }
//         // }
//         // path.path.resize(path.path.size()-itr);

//         lcmInstance_->publish(CONTROLLER_PATH_CHANNEL, &path);

//         return exploration_status_t::STATE_GRAB_BLOCK;  
//     // }

//     // std::cout<<"executeGrabPlanner: No block available for pick up\n";
//     // return exploration_status_t::STATE_GRAB_BLOCK;  

// }


int8_t Exploration::executeGrabBlock(bool initialize)
{
    std::cout<<"executeGrabBlock: Attempting to pick up block\n";
    do{
        //status.state = exploration_status_t::STATE_RETURNING_HOME;
        mbot_arm_cmd_t detectblock;
        detectblock.utime = utime_now();
        detectblock.mbot_cmd = 3;

        lcmInstance_->publish(COMMAND_ARM, &detectblock);
    }while(!block_pickup_status);

    // Need to check if the message confirmation was received

    return exploration_status_t::STATE_COMPLETED_EXPLORATION;
    //return 0;
}

int8_t Exploration::executeDropBlock(bool initialize)
{

    return exploration_status_t::STATE_COMPLETED_EXPLORATION;
    //return 0;
}

int8_t Exploration::executeReturningHome(bool initialize)
{
    //////////////////////// TODO: Implement your method for returning to the home pose ///////////////////////////
    /*
    * NOTES:
    *   - At the end of each iteration, then (1) or (2) must hold, otherwise exploration is considered to have failed:
    *       (1) dist(currentPose_, targetPose_) < kReachedPositionThreshold  :  reached the home pose
    *       (2) currentPath_.path_length > 1  :  currently following a path to the home pose
    */

    std::cout<<"executeReturningHome: Now trying to go home\n";
    planner_.setMap(currentMap_);
    planner_.setNumFrontiers(0);
    if (!planner_.isPathSafe(currentPath_) || currentPath_.path_length == 0
      || sqrt((homePose_.x-currentTarget_.x)*(homePose_.x-currentTarget_.x) + (homePose_.y-currentTarget_.y)*(homePose_.y-currentTarget_.y))>kReachedPositionThreshold){
        currentPath_ = plan_path_to_home(homePose_, currentPose_, currentMap_, planner_);
        currentTarget_ =  currentPath_.path[currentPath_.path_length-1];
    }
    std::cout<<"Path length to home: "<<currentPath_.path.size()<<"\n";
    /////////////////////////////// End student code ///////////////////////////////

    /////////////////////////   Create the status message    //////////////////////////
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_RETURNING_HOME;

    double distToHome = distance_between_points(Point<float>(homePose_.x, homePose_.y),
                                                Point<float>(currentPose_.x, currentPose_.y));
    // If we're within the threshold of home, then we're done.
    if (distToHome <= kReachedPositionThreshold)
    {
        status.status = exploration_status_t::STATUS_COMPLETE;
        std::cout << "Returning complete.\n";
    }
    // Otherwise, if there's a path, then keep following it
    else if (currentPath_.path.size() >= 1)
    {
        status.status = exploration_status_t::STATUS_IN_PROGRESS;
    }
    // Else, there's no valid path to follow and we aren't home, so we have failed.
    else
    {
        status.status = exploration_status_t::STATUS_FAILED;
        std::cout << "No vaild path and not reached home.\n";
    }

    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);

    ////////////////////////////   Determine the next state    ////////////////////////
    if (status.status == exploration_status_t::STATUS_IN_PROGRESS)
    {
        return exploration_status_t::STATE_RETURNING_HOME;
    }
    else if (status.status == exploration_status_t::STATUS_FAILED)
    {
        return exploration_status_t::STATE_FAILED_EXPLORATION;
    }
    else
    {
        return exploration_status_t::STATE_COMPLETED_EXPLORATION;
    }
}

int8_t Exploration::executeCompleted(bool initialize)
{
    // Stay in the completed state forever because exploration only explores a single map.
    exploration_status_t msg;
    msg.utime = utime_now();
    msg.team_number = teamNumber_;
    msg.state = exploration_status_t::STATE_COMPLETED_EXPLORATION;
    msg.status = exploration_status_t::STATUS_COMPLETE;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &msg);
    
    return exploration_status_t::STATE_COMPLETED_EXPLORATION;
}



int8_t Exploration::executeFailed(bool initialize)
{
    // Send the execute failed forever. There is no way to recover.
    exploration_status_t msg;
    msg.utime = utime_now();
    msg.team_number = teamNumber_;
    msg.state = exploration_status_t::STATE_FAILED_EXPLORATION;
    msg.status = exploration_status_t::STATUS_FAILED;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &msg);
    
    return exploration_status_t::STATE_FAILED_EXPLORATION;
}