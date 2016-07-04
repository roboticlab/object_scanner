#include <sstream>
#include <ros/ros.h>

#include <object_scanner/object_scanner.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_scanner");
    ros::NodeHandle n;
    ROS_INFO_STREAM("object_scanner main started");
	
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	float min_weight_ = 2; 
	float xsize = 0.5;
	float ysize = 0.5;
	float zsize = 0.5; 
	int xres = 1000;
	int yres = 1000;
	int zres = 1000; 
	Eigen::Affine3d tsdf_center;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
    ObjectScanner* app = new ObjectScanner(min_weight_, xsize, ysize, zsize, xres, yres, zres, tsdf_center);
    
    while (ros::ok())
    {

    }

    return 0;
}