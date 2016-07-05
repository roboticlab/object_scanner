#include <object_scanner/robots_mover.h>

RobotsMover::RobotsMover()
{
    ROS_INFO_STREAM("RobotsMover class created");
    
    getRosParams();
    camera_mover = new RobotMover(p.arm_group_id, p.camera_viewpoint);
    table_mover = new RobotMover(p.table_group_id, p.table_start_pose);
}

RobotsMover::RobotsMover(RobotsMover::RobotMoverParams p_)
{
    ROS_INFO_STREAM("RobotsMover class created");
    
    p = p_;
    camera_mover = new RobotMover(p.arm_group_id, p.camera_viewpoint);
    table_mover = new RobotMover(p.table_group_id, p.table_start_pose);
}

RobotsMover::~RobotsMover()
{

}

void RobotsMover::getRosParams()
{
    ros::param::param<std::string>("/arm_movegroup_id", p.arm_group_id, "right_arm");
    ros::param::param<std::string>("/table_movegroup_id", p.table_group_id, "left_arm");
    if (!ros::param::get("/camera_viewpoint", p.camera_viewpoint))
    {
        std::vector<double> angles;
        angles.resize(6);
        angles[0] = RAD(-0.068);                                // Kawasaki
        angles[1] = RAD(7.397);
        angles[2] = RAD(-108.914);
        angles[3] = RAD(-69.871);
        angles[4] = RAD(-63.173);
        angles[5] = RAD(102.539);
        p.camera_viewpoint = angles;
    }
    if (!ros::param::get("/table_start_pose", p.table_start_pose))
    {
        std::vector<double> angles;
        angles.resize(6);
        angles[0] = RAD(-32.296);                               // Kawasaki
        angles[1] = RAD(131.096);
        angles[2] = RAD(26.752);
        angles[3] = RAD(1.448);
        angles[4] = RAD(101.565);
        angles[5] = RAD(180.0);
        p.table_start_pose = angles;
    }
}

void RobotsMover::moveToViewpoint()
{
    camera_mover->moveToInitialPose();
}

void RobotsMover::rotateTable(int counter, float angle_step)
{
    std::vector<double> angles;  
    angles.resize(6);
    angles = p.table_start_pose;
    angles[5] = RAD(180.0) - counter * RAD(angle_step); 
    
    table_mover->moveToPose(angles);
}

void RobotsMover::rotateTableToStartPos()
{
    table_mover->moveToInitialPose();
}


