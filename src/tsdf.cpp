#include <object_scanner/tsdf.h>

TSDF::TSDF(TSDFParams* params_)
{
    ROS_INFO_STREAM("1");
	params = params_;
	ROS_INFO_STREAM("2");
	
// 	tsdf = TsdfVolume::Ptr(new TsdfVolume(Eigen::Vector3i(params->xres, params->yres, params->zres)));
	tsdf = TsdfVolume::Ptr(new TsdfVolume(Eigen::Vector3i(128, 128, 128)));
	ROS_INFO_STREAM("3");
	tsdf->setSize( Eigen::Vector3f(params->xsize, params->ysize, params->zsize) );
	ROS_INFO_STREAM("4");
	tsdf->reset();	
	ROS_INFO_STREAM("5");
	rayCaster.reset(new RayCaster(params->image_height, params->image_width, params->focal_length_x, params->focal_length_y, params->principal_point_x, params->principal_point_y) );
	ROS_INFO_STREAM("6");
	marchingCube.reset(new MarchingCubes());
	ROS_INFO_STREAM("7");
}
TSDF::~TSDF()
{

}
void TSDF::integrateCloud(Eigen::Affine3f trans, pcl::PointCloud<pcl::PointXYZRGB> cloudXYZRGB)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ (new pcl::PointCloud<pcl::PointXYZ>); 
	pcl::copyPointCloud(cloudXYZRGB, *cloudXYZ);	
	std::vector<uint16_t> vec_image; 
	std::vector<uint8_t> dummy;
	pcl::io::OrganizedConversion<pcl::PointXYZ,false>::convert(*cloudXYZ, params->focal_length_x, 0, 1, false, vec_image, dummy);
	Depth depth;
	depth.create(cloudXYZ->height, cloudXYZ->width);
	int k = 0;
	for (int i = 0; i < cloudXYZ->height; i++) 
	{
		for (int j = 0; j < cloudXYZ->width; j++) 
		{
			depth[i,j] = vec_image[k];
			k++;
		}
	}	
	pcl::device::kinfuLS::Intr intr (params->focal_length_x, params->focal_length_y, params->principal_point_x, params->principal_point_y);
	float3 volume_size = make_float3 (params->xsize, params->ysize, params->zsize);
	pcl::device::kinfuLS::Mat33 Rcurr_inv;
	Rcurr_inv.data[0] = make_float3(trans.rotation()(0,0), trans.rotation()(0,1), trans.rotation()(0,2));
	Rcurr_inv.data[1] = make_float3(trans.rotation()(1,0), trans.rotation()(1,1), trans.rotation()(1,2));
	Rcurr_inv.data[2] = make_float3(trans.rotation()(2,0), trans.rotation()(2,1), trans.rotation()(2,2));
	float3 tcurr = make_float3 (trans.translation().x(), trans.translation().y(), trans.translation().z());
	pcl::device::kinfuLS::integrateTsdfVolume(depth, intr, volume_size, Rcurr_inv, tcurr, tsdf->getTsdfTruncDist(), tsdf->data());	
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr TSDF::getCloud(Eigen::Affine3f trans)
{
	rayCaster->run(*tsdf, trans, new pcl::gpu::kinfuLS::tsdf_buffer);
	MapArr vertexMap = rayCaster->getVertexMap();
	
	pcl::gpu::DeviceArray2D<pcl::PointXYZ> arrayXYZ;
	pcl::gpu::kinfuLS::convertMapToOranizedCloud(vertexMap, arrayXYZ);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ (new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < arrayXYZ.rows(); i++) 
		for (int j = 0; j < arrayXYZ.cols(); j++) 
			cloudXYZ->points.push_back(arrayXYZ[i,j]);
		
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
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGB (new pcl::PointCloud<pcl::PointXYZRGB>);	
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