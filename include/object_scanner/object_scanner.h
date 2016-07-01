#ifndef __OBJECT_SCANNER_
#define __OBJECT_SCANNER_

#include <ros/ros.h>
#include <object_scanner/robots_mover.h>
#include <object_scanner/tsdf.h>
#include <object_scanner/cloud_processor.h>

class ObjectScanner
{
public:
    ObjectScanner();
    ~ObjectScanner();
private:
    RobotsMover* _mover;
    TSDF* _tsdf;
    CloudProcessor* _cloud_processor;
};

#endif