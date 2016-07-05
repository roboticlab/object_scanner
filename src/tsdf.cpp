#include <object_scanner/tsdf.h>

TSDF::TSDF(float min_weight_, float xsize, float ysize, float zsize, int xres, int yres, int zres, Eigen::Affine3d tsdf_center,
		   double _focal_length_x_, double _focal_length_y_, double _principal_point_x_, double _principal_point_y_, int _image_width_, int _image_height_)
{
	min_weight = min_weight_;	// minimum weight -- i.e. if a voxel sees a point less than 2 times, it will not render  a mesh triangle at that location
	tsdf.reset(new cpu_tsdf::TSDFVolumeOctree (_focal_length_x_, _focal_length_y_, _principal_point_x_, _principal_point_y_, _image_width_, _image_height_) );
	tsdf->setGridSize (xsize, ysize, zsize); // m x m x m
	tsdf->setResolution (xres, yres, zres); // Smallest cell size = 10m / 2048 = about half a centimeter
	tsdf->setGlobalTransform (tsdf_center);
	tsdf->reset();
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr TSDF::getCloud(Eigen::Affine3d trans)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr raytraced = tsdf->renderView(trans);		
// 	pcl::PointCloud<pcl::Intensity>::Ptr _raytraced = tsdf->getIntensityCloud(trans);	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr _cloudXYZ (new pcl::PointCloud<pcl::PointXYZ>);
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