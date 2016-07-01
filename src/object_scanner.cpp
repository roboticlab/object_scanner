#include <object_scanner/object_scanner.h>

ObjectScanner::ObjectScanner()
{
    ROS_INFO_STREAM("Object scanner created");
    _mover = new RobotsMover();
    _tsdf = new TSDF();
    _cloud_processor = new CloudProcessor();
}
