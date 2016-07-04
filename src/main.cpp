#include <sstream>
#include <ros/ros.h>

#include <object_scanner/object_scanner.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_scanner");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ROS_INFO_STREAM("object_scanner main started");
    ObjectScanner* app = new ObjectScanner();   
    
    ros::Duration(1.0).sleep();
 
    app->run();  
    
    while(ros::ok())
    {
	
    }

    return 0;
}