#ifndef __CLOUD_PROCESSOR_H_
#define __CLOUD_PROCESSOR_H_
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/fast_bilateral_omp.h>

#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl_conversions/pcl_conversions.h>

typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> ColorHandlerTXYZRGB;

// struct IcpParams
// {
// //     int max_iterations = 500;
//     
// };
class CloudProcessor
{
public:
    CloudProcessor();
    ~CloudProcessor();
    bool processCloud();
    void readTransform();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getAlighnedCloud();
private:    
    std::string subs_cloud_topic;    
    ros::NodeHandle nh_;
    ros::Subscriber cloud_subscriber;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sensor_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp2_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotated_cloud;
    
    Eigen::Affine3d transform_eigen;
    
    void getCloudVector(int times_num);
    void rotateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output);
    void cutCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output);
    void filterCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output);
    void alighnCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output);
    void getIntegratedCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr output);
    tf::StampedTransform getTransform(std::string parent, std::string child);
    //PCL methods
    void transformCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output, Eigen::Affine3d transform);
    void passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output, 
			   std::string field, float nx, float px,bool keep_organized);
    void removeNaNs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output);
    void bilateralFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output, float sigma_r, float sigma_s);
    void icpAlighn(pcl::PointCloud< pcl::PointXYZRGB >::Ptr _source, pcl::PointCloud< pcl::PointXYZRGB >::Ptr _target, pcl::PointCloud< pcl::PointXYZRGB >::Ptr _cloud_out);
    
    
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &pc_msg);
    
};

#endif