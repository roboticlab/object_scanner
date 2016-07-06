#include <object_scanner/object_scanner.h>
#include <pcl/filters/passthrough.h>
void ObjectScanner::run()
{
    std::thread run_thread(&ObjectScanner::runThread, this);
    run_thread.detach();
}
void ObjectScanner::runThread()
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
// 		_cloud_processor->setIntegratedCloud(_tsdf->getCloud(_cloud_processor->getLastTransform().inverse()));
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr integrated_cloud(_tsdf->getCloud(Eigen::Affine3d::Identity())); 
		ROS_INFO_STREAM("integrated_cloud points: " << integrated_cloud->points.size());
		int nans = 0;
		for (size_t i = 0; i < integrated_cloud->points.size(); i++)
		{
		    if (isnan(integrated_cloud->points[i].z))
			nans++;
		}
		ROS_INFO_STREAM("nans: " << nans);
		_cloud_processor->setIntegratedCloud(integrated_cloud);
		visualizer->updatePointCloud(integrated_cloud, ColorHandlerTXYZRGB(integrated_cloud, 0.0, 255.0, 0.0),  "green_cloud");
		ros::Duration(30).sleep();
	    }
	    for (int i = 0; i < acqusitions_num; i++)
	    {
		ROS_INFO_STREAM("Start processing cloud#" << i);
		if (!_cloud_processor->processCloud())
		{
		    i--;
		}
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr alighned_cloud(_cloud_processor->getAlighnedCloud()); 
		visualizer->updatePointCloud(alighned_cloud, ColorHandlerTXYZRGB(alighned_cloud, 0.0, 0.0, 255.0), "blue_cloud");
		_tsdf->integrateCloud<pcl::PointXYZRGB>(*alighned_cloud);
	    }        	    
	}	
    }
    ROS_INFO_STREAM("Done!");
}

ObjectScanner::ObjectScanner(float min_weight_, float xsize, float ysize, float zsize, int xres, int yres, int zres, Eigen::Affine3d tsdf_center,
							 double _focal_length_x_, double _focal_length_y_, double _principal_point_x_, double _principal_point_y_, int _image_width_, int _image_height_
)
{
    ROS_INFO_STREAM("Object scanner created");
    visualizer.reset(new pcl::visualization::PCLVisualizer("vis"));
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
void ObjectScanner::runVisualizer()
{
    double scale = 0.1;
    visualizer->addCoordinateSystem(scale);
    visualizer->setBackgroundColor(0.0,0.0,0.0);   
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>); 
    visualizer->addPointCloud(cloud, ColorHandlerTXYZRGB(cloud, 0.0, 0.0, 255.0), "blue_cloud");    
    visualizer->addPointCloud(cloud, ColorHandlerTXYZRGB(cloud, 0.0, 255.0, 0.0), "green_cloud");
    visualizer->addPointCloud(cloud, ColorHandlerTXYZRGB(cloud, 255.0, 0.0, 0.0), "red_cloud");
    visualizer->addPointCloud(cloud, ColorHandlerTXYZRGB(cloud, 255.0, 255.0, 0.0), "yellow_cloud");
    visualizer->addPointCloud(cloud, ColorHandlerTXYZRGB(cloud, 255.0, 0.0, 255.0), "purple_cloud");
    visualizer->addPointCloud(cloud, ColorHandlerTXYZRGB(cloud, 0.0, 255.0, 255.0), "light_blue_cloud");
    visualizer->spinOnce();
    visualizer->addCoordinateSystem();
    while(!visualizer->wasStopped())
    {
	    visualizer->spinOnce();
	    ros::Duration(0.01).sleep();
    }
}


