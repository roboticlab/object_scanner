#include <object_scanner/object_scanner.h>
void ObjectScanner::run()
{
    _cloud_processor->processCloud(1);
}
ObjectScanner::ObjectScanner()
{
    ROS_INFO_STREAM("Object scanner created");
    _mover = new RobotsMover();
    _tsdf = new TSDF();
    _cloud_processor = new CloudProcessor();
}
ObjectScanner::~ObjectScanner()
{

}