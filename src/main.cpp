#include <sstream>
#include <ros/ros.h>

#include <object_scanner/object_scanner.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_scanner");
    ros::NodeHandle n;
    ROS_INFO_STREAM("object_scanner main started");
    
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ObjectScanner* app = new ObjectScanner();
    
    app->test();
    
    ros::waitForShutdown();
    return 0;
}