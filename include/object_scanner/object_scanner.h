#ifndef __OBJECT_SCANNER_
#define __OBJECT_SCANNER_

#include <ros/ros.h>
#include <object_scanner/robots_mover.h>
#include <object_scanner/tsdf.h>
#include <object_scanner/cloud_processor.h>

class ObjectScanner
{
public:
    ObjectScanner(float, float, float, float, int, int, int, Eigen::Affine3d);
    ~ObjectScanner();
	template <typename PointT> void IntegrateCloud(pcl::PointCloud<PointT>, Eigen::Affine3d);	
	pcl::PointCloud<pcl::PointNormal>::Ptr GetCloud(Eigen::Affine3d);
	pcl::PolygonMesh GetMesh();
private:
    RobotsMover* _mover;
    TSDF* _tsdf;
    CloudProcessor* _cloud_processor;
};

#endif