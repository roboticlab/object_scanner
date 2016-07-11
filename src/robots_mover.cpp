#include <object_scanner/robots_mover.h>

RobotsMover::RobotsMover(RobotsMover::RobotMoverParams* params)
{
    ROS_INFO_STREAM("RobotsMover class created");
    
    table_start_pose = params->table_start_pose;
    camera_mover = new RobotMover(params->arm_group_id, params->camera_viewpoint);
    table_mover = new RobotMover(params->table_group_id, params->table_start_pose);
}

RobotsMover::~RobotsMover()
{

}

void RobotsMover::moveToViewpoint()
{
    camera_mover->moveToInitialPose();
}

void RobotsMover::rotateTable(int counter, float angle_step)
{
    std::vector<double> angles;  
    angles.resize(6);
    angles = table_start_pose;
    angles[5] = RAD(180.0) - counter * RAD(angle_step); 
    
    table_mover->moveToPose(angles);
}

void RobotsMover::rotateTableToStartPos()
{
    table_mover->moveToInitialPose();
}


