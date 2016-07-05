#include <object_scanner/object_scanner.h>
void ObjectScanner::run()
{
    acqusitions_num = 5;
    _cloud_processor->readTransform();
    for (int i = 0; i < acqusitions_num; i++)
    {
	ROS_INFO_STREAM("Start processing cloud#" << i);
	if (!_cloud_processor->processCloud())
	{
	    i--;
	}
    }    
    ROS_INFO_STREAM("Done!");
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