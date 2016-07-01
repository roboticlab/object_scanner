#include <object_scanner/robots_mover.h>
RobotsMover::RobotsMover()
{
    ROS_INFO_STREAM("RobotsMover class created");
    camera_mover = new RobotMover();
    table_mover = new RobotMover();
}
RobotsMover::~RobotsMover()
{

}
