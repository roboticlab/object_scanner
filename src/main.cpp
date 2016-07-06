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

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	float min_weight_ = 1; 
	float xsize = 2;
	float ysize = 2;
	float zsize = 2; 
	int xres = 100;
	int yres = 100;
	int zres = 100; 
	Eigen::Affine3d tsdf_center(Eigen::Affine3d::Identity());
	double _focal_length_x_ = 525.; 
	double _focal_length_y_ = 525.; 
	double _principal_point_x_ = 323; 
	double _principal_point_y_ = 261; 
	int _image_width_ = 640; 
	int _image_height_ = 480;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
    ObjectScanner* app = new ObjectScanner(min_weight_, xsize, ysize, zsize, xres, yres, zres, tsdf_center, 
					  _focal_length_x_, _focal_length_y_, _principal_point_x_, _principal_point_y_, _image_width_, _image_height_);
    
 
    ros::Duration(1.0).sleep();    
    app->run();
    app->runVisualizer();
    ros::waitForShutdown();

    return 0;
}