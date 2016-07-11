#include <object_scanner/tsdf.h>

TSDF::TSDF(TSDFParams* params)
{
	min_weight = params->min_weight;	// minimum weight -- i.e. if a voxel sees a point less than 2 times, it will not render  a mesh triangle at that location
	tsdf.reset(new cpu_tsdf::TSDFVolumeOctree (params->focal_length_x, params->focal_length_y, params->principal_point_x, params->principal_point_y, params->image_width, params->image_height) );
	tsdf->setGridSize (params->xsize, params->ysize, params->zsize); // m x m x m
	tsdf->setResolution (params->xres, params->yres, params->zres); // Smallest cell size = 10m / 2048 = about half a centimeter

	Eigen::Matrix3d m; m << params->tsdf_center_rotation[0], params->tsdf_center_rotation[1], params->tsdf_center_rotation[2],
							params->tsdf_center_rotation[3], params->tsdf_center_rotation[4], params->tsdf_center_rotation[5],
							params->tsdf_center_rotation[6], params->tsdf_center_rotation[7], params->tsdf_center_rotation[8];
	Eigen::Vector3d v; v << params->tsdf_center_translation[0], params->tsdf_center_translation[1], params->tsdf_center_translation[2];
	
	Eigen::Affine3d rotate = Eigen::Affine3d(Eigen::AngleAxisd(m));
	Eigen::Affine3d translation = Eigen::Affine3d(Eigen::Translation3d(v));
	
	tsdf->setGlobalTransform(rotate * translation);
	tsdf->reset();
}
TSDF::~TSDF()
{

}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr TSDF::getCloud(Eigen::Affine3d trans)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr raytraced = tsdf->renderView(trans);		
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*raytraced, *cloudXYZRGB);
	return cloudXYZRGB;
}
pcl::PolygonMesh TSDF::getMesh()
{
	cpu_tsdf:: MarchingCubesTSDFOctree mc;
	mc.setInputTSDF (tsdf);
	mc.setMinWeight (min_weight); 
	mc.setColorByRGB (false); 
	pcl::PolygonMesh mesh;
	mc.reconstruct (mesh);
	return mesh;
}