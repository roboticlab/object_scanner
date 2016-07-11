#ifndef __TSDF_H_
#define __TSDF_H_

#include <ros/ros.h>
#include <object_scanner/libTSDF/cpu_tsdf/tsdf_volume_octree.h>
#include <pcl/PolygonMesh.h>

#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include <object_scanner/libTSDF/cpu_tsdf/marching_cubes_tsdf_octree.h>

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
	
	template <typename PointT> 
	void integrateCloud(pcl::PointCloud<PointT> cloud, Eigen::Affine3d trans)
	{
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
		normals->clear();
		tsdf->integrateCloud (cloud, *normals, trans);
	};	
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud(Eigen::Affine3d);
	pcl::PolygonMesh getMesh();
	
private:
	cpu_tsdf::TSDFVolumeOctree::Ptr tsdf;    
	float min_weight;
};

#endif