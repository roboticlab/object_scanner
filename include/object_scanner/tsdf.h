#ifndef __TSDF_H_
#define __TSDF_H_

#include <ros/ros.h>
#include <pcl/PolygonMesh.h>
#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/compression/organized_pointcloud_conversion.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include <pcl/gpu/kinfu_large_scale/kinfu.h>
#include <pcl/gpu/kinfu_large_scale/raycaster.h>
#include <pcl/gpu/kinfu_large_scale/tsdf_volume.h>
#include <pcl/gpu/kinfu_large_scale/marching_cubes.h>
#include <pcl/gpu/containers/device_array.h>
#include <pcl/cuda/common/eigen.h>

// #include <pcl/gpu/kinfu/raycaster.h>
// #include <pcl/gpu/kinfu/tsdf_volume.h>
// #include <pcl/gpu/kinfu/marching_cubes.h>
// #include <pcl/cuda/common/eigen.h>
// #include <pcl/gpu/kinfu_large_scale/device.h>
// // #include <pcl/gpu/kinfu_large_scale/kinfu.h>
// #include <pcl/gpu/containers/device_array.h>

typedef pcl::gpu::kinfuLS::RayCaster RayCaster;
typedef pcl::gpu::kinfuLS::TsdfVolume TsdfVolume;
typedef pcl::gpu::kinfuLS::MarchingCubes MarchingCubes;
typedef pcl::gpu::DeviceArray2D<float> MapArr;
typedef pcl::gpu::DeviceArray2D<unsigned short> Depth;


class TSDF
{
public:
	
	struct TSDFParams
    {
		float min_weight;
		float xsize;
		float ysize;
		float zsize;
		int xres;
		int yres;
		int zres;
		std::vector<double> tsdf_center_rotation;
		std::vector<double> tsdf_center_translation;
		double focal_length_x;
		double focal_length_y;
		double principal_point_x;
		double principal_point_y;
		int image_width;
		int image_height;
    };
	
    TSDF(TSDFParams*);
	~TSDF();
	
	void integrateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud(Eigen::Affine3f);
	pcl::PolygonMesh getMesh();
	
private:
	RayCaster::Ptr rayCaster;
	TsdfVolume::Ptr tsdf;
	MarchingCubes::Ptr marchingCube;
	TSDFParams* params;
	boost::shared_ptr<pcl::PolygonMesh> convertToMesh(const pcl::gpu::DeviceArray<pcl::PointXYZ>& triangles);
// 	pcl::PointCloud<pcl::PointXYZRGB>::Ptr depthToCloud(Depth);
// 	Depth cloudToDepth(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> depthToCloud(Depth*);
	Depth* cloudToDepth(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>);
};

#endif