#ifndef __ROBOTS_MOVER_
#define __ROBOTS_MOVER_

#include <ros/ros.h>
#include <object_scanner/robot_mover.h>

#define RAD(deg)        ((deg)*M_PI/180)

class RobotsMover
{
public:
    
    struct RobotMoverParams
    {
        std::string arm_group_id;
        std::string table_group_id;
        std::vector<double> camera_viewpoint;
        std::vector<double> table_start_pose;
    };
	
    RobotsMover(RobotsMover::RobotMoverParams*);
    ~RobotsMover();
    
    void moveToViewpoint();
    void rotateTableToStartPos();
    void rotateTable(int, float);

private:
    RobotMoverParams* p;
    RobotMover* camera_mover;
    RobotMover* table_mover;
	std::vector<double> table_start_pose;
};

#endif