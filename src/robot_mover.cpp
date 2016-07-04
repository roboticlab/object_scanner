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
    ROS_INFO_STREAM("Moved " << move_group_ptr->getName() << " to initial point"); 
}

void RobotMover::moveToPose(std::vector<double> angles)
{
    robot_state::RobotState start_state(*move_group_ptr->getCurrentState());
    move_group_ptr->setStartState(start_state);
    move_group_ptr->setJointValueTarget(angles);
    
    bool success = move_group_ptr->plan(plan);
    ROS_INFO("Survey %s", success ? "SUCCESS" : "FAILED"); 
    move_group_ptr->asyncExecute(plan); 
    waiting();
}

void RobotMover::waiting()
{
    ROS_INFO_STREAM("Waiting...");
    robot_state::RobotState prev_state(*move_group_ptr->getCurrentState());
    robot_state::RobotState curr_state(*move_group_ptr->getCurrentState());
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

    bool isMoving = true;    
    ros::Duration(2.0).sleep();

    while (isMoving)
    {
        ros::Duration(1.0).sleep();
        curr_state = *move_group_ptr->getCurrentState();

        for(std::size_t i = 0; i < joint_names.size(); i++)
        {
            if (fabs(*(curr_state.getJointPositions(joint_names[i])) - *(prev_state.getJointPositions(joint_names[i]))) > 0.005)
            {
                isMoving = true;
                break;
            }
            else
                isMoving = false;
        } 
        prev_state = curr_state;
    }
    ROS_INFO_STREAM("Stopped waiting");
}
