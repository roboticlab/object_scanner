#include <object_scanner/object_scanner.h>

void ObjectScanner::run()
{
    _mover->moveToViewpoint();
    ros::Duration(2.0).sleep();
    _mover->rotateTableToStartPos();
    ros::Duration(2.0).sleep();
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

ObjectScanner::ObjectScanner(ObjectScannerParams* params)
{
    ROS_INFO_STREAM("Object scanner created");
    _mover = new RobotsMover(params->robot_mover_params);
    _tsdf = new TSDF(params->tsdf_params);	
    _cloud_processor = new CloudProcessor(params->cloud_processor_params);
// 	TSDFtest();
}
ObjectScanner::~ObjectScanner()
{
}
pcl::PointCloud< pcl::PointXYZRGB >::Ptr ObjectScanner::getCloud(Eigen::Affine3d trans)
{
	return _tsdf->getCloud(trans);
}
pcl::PolygonMesh ObjectScanner::getMesh()
{
	return _tsdf->getMesh();
}
void ObjectScanner::TSDFtest()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud0 (new pcl::PointCloud<pcl::PointXYZRGB>);	
	pcl::io::loadPCDFile ("/home/elena/catkin_ws/src/scaner/src/0.pcd", *cloud0);
	integrateCloud(*cloud0, Eigen::Affine3d::Identity());
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud25 (new pcl::PointCloud<pcl::PointXYZRGB>);	
	pcl::io::loadPCDFile ("/home/elena/catkin_ws/src/scaner/src/25.pcd", *cloud25);
	integrateCloud(*cloud25, Eigen::Affine3d(Eigen::Translation3d(Eigen::Vector3d(0.02,0.0,0.0))));
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud50 (new pcl::PointCloud<pcl::PointXYZRGB>);	
	pcl::io::loadPCDFile ("/home/elena/catkin_ws/src/scaner/src/50.pcd", *cloud50);
	integrateCloud(*cloud50, Eigen::Affine3d(Eigen::Translation3d(Eigen::Vector3d(0.04,0.0,0.0))));
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud25_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud (*cloud50, *cloud25_transformed, Eigen::Affine3d(Eigen::Translation3d(Eigen::Vector3d(0.02,0.0,0.0))));
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud50_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud (*cloud50, *cloud50_transformed, Eigen::Affine3d(Eigen::Translation3d(Eigen::Vector3d(0.04,0.0,0.0))));
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ = _tsdf->getCloud(Eigen::Affine3d::Identity());
	
	pcl::visualization::PCLVisualizer visualizer("Visualiser");
	visualizer.addPointCloud(cloud0, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB>(cloud0, 255.0, 0.0, 0.0), "cloud0");
	visualizer.addPointCloud(cloud25_transformed, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB>(cloud25_transformed, 250, 0.0, 250.0), "cloud25_transformed");
	visualizer.addPointCloud(cloud50_transformed, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB>(cloud50_transformed, 0.0, 0.0, 250.0), "cloud50_transformed");
	visualizer.addPointCloud(cloud_, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB>(cloud_, 0.0, 255.0, 0.0), "cloud_");
	visualizer.setBackgroundColor(0.0,0.0,0.0);
	
	pcl::io::savePLYFileBinary ("/home/elena/catkin_ws/src/scaner/src/123.ply", getMesh());

	while(!visualizer.wasStopped())
	{
		visualizer.spinOnce();
		ros::Duration(0.005).sleep();
	}
}

