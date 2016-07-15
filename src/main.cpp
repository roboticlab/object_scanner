#include <sstream>
#include <ros/ros.h>
#include <object_scanner/object_scanner.h>

#define RAD(deg)        ((deg)*M_PI/180)

void getRosParams(ObjectScanner::ObjectScannerParams& params)
{
	ros::param::get("cloud_processor/parent", params.cloud_processor_params->parent);
	ros::param::get("cloud_processor/child", params.cloud_processor_params->child);
	ros::param::get("cloud_processor/nx", params.cloud_processor_params->nx);
	ros::param::get("cloud_processor/px", params.cloud_processor_params->px);
	ros::param::get("cloud_processor/ny", params.cloud_processor_params->ny);
	ros::param::get("cloud_processor/py", params.cloud_processor_params->py);
	ros::param::get("cloud_processor/nz", params.cloud_processor_params->nz);
	ros::param::get("cloud_processor/pz", params.cloud_processor_params->pz);
	ros::param::get("cloud_processor/sigma_s", params.cloud_processor_params->sigma_s);
	ros::param::get("cloud_processor/sigma_r", params.cloud_processor_params->sigma_r);
	ros::param::get("cloud_processor/max_iterations", params.cloud_processor_params->max_iterations);
	ros::param::get("cloud_processor/max_correspondence_distance", params.cloud_processor_params->max_correspondence_distance);
	ros::param::get("cloud_processor/transformation_epsilon", params.cloud_processor_params->transformation_epsilon);
	
	ros::param::get("robot_mover/arm_group_id", params.robot_mover_params->arm_group_id);
	ros::param::get("robot_mover/table_group_id", params.robot_mover_params->table_group_id);
	ros::param::get("robot_mover/camera_viewpoint", params.robot_mover_params->camera_viewpoint);
	ros::param::get("robot_mover/table_start_pose", params.robot_mover_params->table_start_pose);
	
	ros::param::get("tsdf/min_weight", params.tsdf_params->min_weight);
	ros::param::get("tsdf/xsize", params.tsdf_params->xsize);
	ros::param::get("tsdf/ysize", params.tsdf_params->ysize);
	ros::param::get("tsdf/zsize", params.tsdf_params->zsize);
	ros::param::get("tsdf/xres", params.tsdf_params->xres);
	ros::param::get("tsdf/yres", params.tsdf_params->yres);
	ros::param::get("tsdf/zres", params.tsdf_params->zres);
	ros::param::get("tsdf/tsdf_center_rotation", params.tsdf_params->tsdf_center_rotation);
	ros::param::get("tsdf/tsdf_center_translation", params.tsdf_params->tsdf_center_translation);
	ros::param::get("tsdf/principal_point_x", params.tsdf_params->principal_point_x);
	ros::param::get("tsdf/principal_point_y", params.tsdf_params->principal_point_y);
	ros::param::get("tsdf/image_width", params.tsdf_params->image_width);
	ros::param::get("tsdf/image_height", params.tsdf_params->image_height);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_scanner");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ROS_INFO_STREAM("object_scanner main started");
	
	ObjectScanner::ObjectScannerParams* params = new ObjectScanner::ObjectScannerParams;
	params->cloud_processor_params = new CloudProcessor::CloudProcessorParams;
	params->robot_mover_params = new RobotsMover::RobotMoverParams;
	params->tsdf_params = new TSDF::TSDFParams;
		
	getRosParams(*params);
	
    ObjectScanner* app = new ObjectScanner(params);
	
//     ros::Duration(1.0).sleep();
//     app->run();
    
    ros::waitForShutdown();

    return 0;
}