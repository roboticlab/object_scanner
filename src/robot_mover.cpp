#include <object_scanner/robot_mover.h>

RobotMover::RobotMover(std::string move_group_id_, std::vector<double> initial_pose_)
{
    initial_pose = initial_pose_;    
    move_group_ptr = boost::shared_ptr< moveit::planning_interface::MoveGroup > (new moveit::planning_interface::MoveGroup (move_group_id_));

    robot_state::RobotState start_state(*(move_group_ptr->getCurrentState()));
    joint_model_group = start_state.getJointModelGroup(move_group_ptr->getName());    
    move_group_ptr->setStartState(start_state);
    move_group_ptr->allowReplanning(false);
    move_group_ptr->setPlannerId("RRTConnectkConfigDefault");
    
    ROS_INFO("RobotMover %s with created", move_group_id_.c_str());
}
RobotMover::~RobotMover()
{

}

void RobotMover::moveToInitialPose()
{
    moveToPose(initial_pose);
    ROS_INFO("Set %s to initial point", move_group_ptr->getName().c_str()); 
}

void RobotMover::moveToPose(std::vector<double> angles)
{
    robot_state::RobotState start_state(*move_group_ptr->getCurrentState());
    move_group_ptr->setStartState(start_state);
    move_group_ptr->setJointValueTarget(angles);
    
    bool success = move_group_ptr->plan(plan);
    ROS_INFO("Survey %s", success ? "SUCCESS" : "FAILED"); 
}
