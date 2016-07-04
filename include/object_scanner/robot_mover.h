#ifndef __ROBOT_MOVER_
#define __ROBOT_MOVER_

#include <ros/ros.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_state/conversions.h>

class RobotMover
{
public:
    RobotMover();
    ~RobotMover();
private:
    boost::shared_ptr< moveit::planning_interface::MoveGroup > move_group_ptr;
    const robot_state::JointModelGroup* joint_model_group;
    moveit::planning_interface::MoveGroup::Plan plan;
};

#endif