#ifndef __OBJECT_SCANNER_
#define __OBJECT_SCANNER_

#include <ros/ros.h>
#include <object_scanner/robots_mover.h>
#include <object_scanner/tsdf.h>
#include <object_scanner/cloud_processor.h>
#include <pcl/visualization/pcl_visualizer.h>

class ObjectScanner
{
public:
    ObjectScanner(float, float, float, float, int, int, int, Eigen::Affine3d, double, double, double, double, int, int);
    ~ObjectScanner();
	
	template <typename PointT> 
	void integrateCloud(pcl::PointCloud<PointT> cloud, Eigen::Affine3d trans)
	{
		_tsdf->integrateCloud(cloud, trans);
	};
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud(Eigen::Affine3d);
	pcl::PolygonMesh getMesh();
	void TSDFtest();
	
private:
    RobotsMover* _mover;
    TSDF* _tsdf;
    CloudProcessor* _cloud_processor;
};

#endif