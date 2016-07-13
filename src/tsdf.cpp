#include <object_scanner/tsdf.h>

TSDF::TSDF(TSDFParams* params_)
{
	params = params_;
	tsdf.reset(new TsdfVolume(Eigen::Vector3i(params->xres, params->yres, params->zres)));
	tsdf->setSize(Eigen::Vector3f(params->xsize, params->ysize, params->zsize));
	tsdf->reset();	
	rayCaster.reset(new RayCaster(params->image_height, params->image_width, params->focal_length_x, params->focal_length_y, params->principal_point_x, params->principal_point_y) );
	marchingCube.reset(new MarchingCubes());
}
TSDF::~TSDF()
{

}
void TSDF::integrateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGB)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ (new pcl::PointCloud<pcl::PointXYZ>); 
	pcl::copyPointCloud(*cloudXYZRGB, *cloudXYZ);	
	std::vector<uint16_t> vec_image; 
	std::vector<uint8_t> dummy;
	pcl::io::OrganizedConversion<pcl::PointXYZ,false>::convert(*cloudXYZ, params->focal_length_x, 0, 1, false, vec_image, dummy);
	int rows = cloudXYZ->height; 
	int cols = cloudXYZ->width;
	Depth depth;
	depth.create(rows, cols);
	int k = 0;
	for (int i = 0; i < rows; i++) 
	{
		for (int j = 0; j < cols; j++) 
		{
			depth[i,j] = vec_image[k];
			k++;
		}
	}
	pcl::device::kinfuLS::Intr intr;
	float3 f;
	pcl::device::kinfuLS::Mat33 Rcurr_inv;

// 	pcl::device::integrateTsdfVolume(depth, pcl::device::Intr(params->focal_length_x, params->focal_length_y, params->principal_point_x, params->principal_point_y), 
// 									 float3(params->xsize, params->ysize, params->zsize), Rcurr_inv, tcurr, tranc_dist, volume);
// 	pcl::device::kinfuLS::integrateTsdfVolume();
}

// void pcl::device::integrateTsdfVolume ( const PtrStepSz< ushort > & depth_raw,
// const Intr & intr,
// const float3 & volume_size,
// const Mat33 & Rcurr_inv,
// const float3 & tcurr,
// float tranc_dist,
// PtrStep< short2 > volume 
// ) 
// Performs Tsfg volume uptation (extra obsolete now)
// 
// Parameters
// [in] depth_raw Kinect depth image
// [in] intr camera intrinsics
// [in] volume_size size of volume in mm
// [in] Rcurr_inv inverse rotation for current camera pose
// [in] tcurr translation for current camera pose
// [in] tranc_dist tsdf truncation distance
// [in] volume tsdf volume to be updated




pcl::PointCloud<pcl::PointXYZRGB>::Ptr TSDF::getCloud(Eigen::Affine3f trans)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ (new pcl::PointCloud<pcl::PointXYZ>); 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
	
	rayCaster->run(*tsdf, trans);
	MapArr vertexMap = rayCaster->getVertexMap();		
	pcl::gpu::kinfuLS::convertMapToOranizedCloud(vertexMap, *cloudXYZ);
	
// 	int rows = vertexMap.rows(); 
// 	int cols = vertexMap.cols();
// 	std::vector<float> vec_image; 
// 	for (int i = 0; i < rows; i++) 
// 	{
// 		for (int j = 0; j < cols; j++) 
// 		{
// 			vec_image.push_back(vertexMap[i,j]);
// 		}
// 	}
// 	std::vector<uint8_t> dummy; 
// 	
// 	pcl::io::OrganizedConversion<pcl::PointXYZ,false>::convert(vec_image, dummy, false, cols, rows, params->focal_length_x, *cloudXYZ);
	
	pcl::copyPointCloud(*cloudXYZ, *cloudXYZRGB);
	return cloudXYZRGB;
}
pcl::PolygonMesh TSDF::getMesh()
{	
	pcl::gpu::DeviceArray< pcl::PointXYZ > triangles_target, triangles_buffer;
	triangles_target = marchingCube->run(*tsdf, triangles_buffer);
	pcl::PolygonMesh::Ptr mesh = convertToMesh(triangles_target);
	return *mesh;
}
boost::shared_ptr<pcl::PolygonMesh> TSDF::convertToMesh(const pcl::gpu::DeviceArray<pcl::PointXYZ>& triangles)
{ 
	if (triangles.empty())
		return boost::shared_ptr<pcl::PolygonMesh>();

	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.width = (int)triangles.size();
	cloud.height = 1;
	triangles.download(cloud.points);

	boost::shared_ptr<pcl::PolygonMesh> mesh_ptr( new pcl::PolygonMesh() ); 
	pcl::toPCLPointCloud2(cloud, mesh_ptr->cloud);

	mesh_ptr->polygons.resize (triangles.size() / 3);
	for (size_t i = 0; i < mesh_ptr->polygons.size (); ++i)
	{
		pcl::Vertices v;
		v.vertices.push_back( i * 3 + 0 );
		v.vertices.push_back( i * 3 + 2 );
		v.vertices.push_back( i * 3 + 1 ); 
		mesh_ptr->polygons[i] = v;
	} 
	return mesh_ptr;
}
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> TSDF::depthToCloud(Depth* depth)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ (new pcl::PointCloud<pcl::PointXYZ>); 
	
	
	
	return cloudXYZ;
}
Depth* TSDF::cloudToDepth(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudXYZ)
{
	Depth* depth  = new Depth; 
	
	return depth;
}