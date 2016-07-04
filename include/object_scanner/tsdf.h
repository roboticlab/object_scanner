#ifndef __TSDF_H_
#define __TSDF_H_

#include <ros/ros.h>
#include <object_scanner/libTSDF/cpu_tsdf/tsdf_volume_octree.h>
#include <pcl/PolygonMesh.h>
#include <object_scanner/libTSDF/cpu_tsdf/marching_cubes_tsdf_octree.h>

class TSDF
{
public:
    TSDF(float, float, float, float, int, int, int, Eigen::Affine3d);	
	template <typename PointT> void IntegrateCloud(pcl::PointCloud<PointT>, Eigen::Affine3d);	
	pcl::PointCloud<pcl::PointNormal>::Ptr GetCloud(Eigen::Affine3d);
	pcl::PolygonMesh GetMesh();
	
private:
	cpu_tsdf::TSDFVolumeOctree::Ptr tsdf;    
	float min_weight;
};
#endif