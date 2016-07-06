#include <object_scanner/object_scanner.h>
#include <pcl/filters/passthrough.h>
void ObjectScanner::run()
{
    _mover->moveToViewpoint();
    ros::Duration(2.0).sleep();
    _mover->rotateTableToStartPos();
    ros::Duration(2.0).sleep();
    
    acqusitions_num = 5;
    
    int camera_points = 1;
    
    float angle = 10;
    int steps = 360 / angle;
    
    for (int point_num = 0; point_num < camera_points; point_num++)
    {
	for (int step = 0; step < steps; step++)
	{
	    _mover->rotateTable(step, angle);
	    ROS_INFO_STREAM("Table rotated; step " << step << " of " << steps << "; angle: " << angle);
	    
	    _cloud_processor->readTransform();	    
	    if (step == 0)
	    {
		_cloud_processor->setAlighnCloud(false);
	    }
	    else
	    {
		_cloud_processor->setAlighnCloud(true);
		_cloud_processor->setIntegratedCloud(_tsdf->getCloud(_cloud_processor->getLastTransform().inverse()));
	    }
	    for (int i = 0; i < acqusitions_num; i++)
	    {
		ROS_INFO_STREAM("Start processing cloud#" << i);
		if (!_cloud_processor->processCloud())
		{
		    i--;
		}
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(_cloud_processor->getAlighnedCloud()); 
		
		_tsdf->integrateCloud<pcl::PointXYZRGB>(*cloud);
	    }        
	    
	}
	
    }
    

//     ROS_INFO_STREAM("Done!");
}

ObjectScanner::ObjectScanner(float min_weight_, float xsize, float ysize, float zsize, int xres, int yres, int zres, Eigen::Affine3d tsdf_center,
							 double _focal_length_x_, double _focal_length_y_, double _principal_point_x_, double _principal_point_y_, int _image_width_, int _image_height_
)

{
    ROS_INFO_STREAM("Object scanner created");
    _mover = new RobotsMover();
    _tsdf = new TSDF(min_weight_, xsize, ysize, zsize, xres, yres, zres, tsdf_center, _focal_length_x_, _focal_length_y_, _principal_point_x_, _principal_point_y_, _image_width_, _image_height_);	
    _cloud_processor = new CloudProcessor();
//     TSDFtest();
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

