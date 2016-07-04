#include <object_scanner/cloud_processor.h>


void CloudProcessor::processCloud()
{
    // Rotate
    rotateCloud(input_cloud, temp_cloud);
    // Cut
    cutCloud(temp_cloud, temp2_cloud);
    // Filter
    filterCloud(temp2_cloud, temp_cloud);
    // Alighn
    alighnCloud(temp_cloud, temp2_cloud);
}
void CloudProcessor::rotateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
{
    ROS_INFO_STREAM("rotateCloud");
}
void CloudProcessor::cutCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
{
    ROS_INFO_STREAM("cutCloud");
}
void CloudProcessor::filterCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
{
    ROS_INFO_STREAM("filterCloud");
}
void CloudProcessor::alighnCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
{
    ROS_INFO_STREAM("alighnCloud");
}


// ---------------- Pure PCL functions
void CloudProcessor::tranformCloud(pcl::PointCloud< pcl::PointXYZRGB >::Ptr input, pcl::PointCloud< pcl::PointXYZRGB >::Ptr output, Eigen::Affine3d transform)
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

// ---------------- Constructor, callbacks, common functions
void CloudProcessor::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& pc_msg)
{
    input_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*pc_msg, *this->input_cloud);
}
CloudProcessor::CloudProcessor()
{
    ROS_INFO_STREAM("CloudProcessor class created");
    
    cloud_subscriber =  nh_.subscribe(subs_cloud_topic.c_str(),1,&CloudProcessor::pointCloudCallback, this);
    
    input_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    temp_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    temp2_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
}
CloudProcessor::~CloudProcessor()
{

}
