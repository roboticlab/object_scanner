#include <object_scanner/cloud_processor.h>


void CloudProcessor::processCloud(int times_num)
{
    ROS_INFO_STREAM("CloudProcessor::processCloud");
    rotateCloud(input_cloud, temp_cloud);
    cutCloud(temp_cloud, temp2_cloud);
    filterCloud(temp2_cloud, temp_cloud);  
    alighnCloud(temp_cloud, output_cloud);
        pcl::visualization::PCLVisualizer visualizer("Visualiser");
    visualizer.addPointCloud(temp2_cloud, ColorHandlerTXYZRGB(temp2_cloud, 0.0, 0.0, 255.0), "temp2_cloud");    
    visualizer.addPointCloud(output_cloud, ColorHandlerTXYZRGB(output_cloud, 0.0, 255.0, 0.0), "output_cloud");
    visualizer.addCoordinateSystem();
    visualizer.setBackgroundColor(0.0,0.0,0.0);    
    while(!visualizer.wasStopped())
    {
	    visualizer.spinOnce();
	    ros::Duration(0.05).sleep();
    }
    ROS_INFO_STREAM("CloudProcessor::processCloud done");
}
void CloudProcessor::rotateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
{
    std::string parent = "camera_rgb_optical_frame";
    std::string child = "rotary_table";
    tf::StampedTransform transform = getTransform(child, parent);
    Eigen::Affine3d transform_eigen;
    tf::transformTFToEigen(transform, transform_eigen);
    transformCloud(input, output, transform_eigen);
}
void CloudProcessor::cutCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
{   
    float nx = -0.3;
    float px = 0.3;
    float ny = -0.3;
    float py = 0.3;
    float nz = -0.1;
    float pz = 0.4;
    
    passThroughFilter(input, output, "x", nx, px, true);
    passThroughFilter(output, output, "y", ny, py, true);
    passThroughFilter(output, output, "z", nz, pz, true);
}
void CloudProcessor::filterCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
{
    float sigma_s = 3.0;
    float sigma_r = 0.01;
    bilateralFilter(input, output, sigma_r, sigma_s);
    removeNaNs(output, output);
}
void CloudProcessor::alighnCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    getIntegratedCloud(target_cloud);
    icpAlighn(input, target_cloud, output);
}
void CloudProcessor::getIntegratedCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
{
    // Waiting for TSDF engine
    *output = *input_cloud;
}
tf::StampedTransform CloudProcessor::getTransform(std::string parent, std::string child)
{
    tf::TransformListener listener;
    tf::StampedTransform out;
    try
    {
	if (listener.waitForTransform(parent, child, ros::Time(0), ros::Duration(5)))
	{
	    listener.lookupTransform(parent, child, ros::Time(0), out); 
	    return out;
	}
	else 
	{
	    ROS_INFO_STREAM("Reading transform " << parent << "->" << child << " failed");
	}
    }
    catch (tf::TransformException ex)
    {
	ROS_ERROR("%s",ex.what());
	ros::Duration(1).sleep();
    }
}

// ---------------- Pure PCL methods
void CloudProcessor::transformCloud(pcl::PointCloud< pcl::PointXYZRGB >::Ptr input, pcl::PointCloud< pcl::PointXYZRGB >::Ptr output, Eigen::Affine3d transform)
{
    pcl::transformPointCloud(*input, *output, transform);
}
void CloudProcessor::passThroughFilter(pcl::PointCloud< pcl::PointXYZRGB >::Ptr input, pcl::PointCloud< pcl::PointXYZRGB >::Ptr output, 
				       std::string field, float nx, float px, bool keep_organized = true)
{
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setKeepOrganized(keep_organized);
    pass.setInputCloud (input);
    pass.setFilterFieldName (field);
    pass.setFilterLimits (nx, px);
    pass.filter (*output);  
}
void CloudProcessor::removeNaNs(pcl::PointCloud< pcl::PointXYZRGB >::Ptr input, pcl::PointCloud< pcl::PointXYZRGB >::Ptr output)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>()); 
    for (auto point : input->points)
    {
	if (!isnan(point.x))
	{
	    out_cloud->points.push_back(point);
	}
    }
    *output = *out_cloud;
}
void CloudProcessor::bilateralFilter(pcl::PointCloud< pcl::PointXYZRGB >::Ptr input, pcl::PointCloud< pcl::PointXYZRGB >::Ptr output, float sigma_r, float sigma_s)
{
    pcl::FastBilateralFilterOMP<pcl::PointXYZRGB> fbFilter;
    fbFilter.setInputCloud(input); 
    fbFilter.setSigmaR(sigma_r);    
    fbFilter.setSigmaS(sigma_s);  
    fbFilter.applyFilter(*output);   
}
void CloudProcessor::icpAlighn(pcl::PointCloud< pcl::PointXYZRGB >::Ptr _source, pcl::PointCloud< pcl::PointXYZRGB >::Ptr _target, pcl::PointCloud< pcl::PointXYZRGB >::Ptr _cloud_out)
{
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> ICPAlighner;
    ICPAlighner.setInputSource(_source); //model
    ICPAlighner.setInputTarget (_target); //scene
    ICPAlighner.setMaximumIterations (500);
    ICPAlighner.setMaxCorrespondenceDistance (0.01);
    ICPAlighner.setTransformationEpsilon (1e-6);
    ICPAlighner.align (*_cloud_out);
}

// ---------------- Constructor, callbacks, common methods
void CloudProcessor::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& pc_msg)
{
    input_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*pc_msg, *this->input_cloud);
}
pcl::PointCloud< pcl::PointXYZRGB >::Ptr CloudProcessor::getAlighnedCloud()
{
    return output_cloud;
}
CloudProcessor::CloudProcessor()
{
    subs_cloud_topic = "/camera/depth_registered/points";    
    cloud_subscriber =  nh_.subscribe(subs_cloud_topic,1,&CloudProcessor::pointCloudCallback, this);    
    input_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    output_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    temp_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    temp2_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
}
CloudProcessor::~CloudProcessor()
{

}


//     pcl::visualization::PCLVisualizer visualizer("Visualiser");
//     visualizer.addPointCloud(temp_cloud, ColorHandlerTXYZRGB(temp_cloud, 0.0, 0.0, 255.0), "temp_cloud");    
//     visualizer.addPointCloud(output_cloud, ColorHandlerTXYZRGB(output_cloud, 0.0, 255.0, 0.0), "output_cloud");
//     visualizer.addCoordinateSystem();
//     visualizer.setBackgroundColor(0.0,0.0,0.0);    
//     while(!visualizer.wasStopped())
//     {
// 	    visualizer.spinOnce();
// 	    ros::Duration(0.05).sleep();
//     }