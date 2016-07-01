#ifndef __ROBOTS_MOVER_
#define __ROBOTS_MOVER_

#include <ros/ros.h>
#include <object_scanner/robot_mover.h>

class RobotsMover
{
public:
    RobotsMover();
private:
    RobotMover* camera_mover;
    RobotMover* table_mover;
};

#endif