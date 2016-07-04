#include <object_scanner/tsdf.h>

TSDF::TSDF(float min_weight_, float xsize, float ysize, float zsize, int xres, int yres, int zres, Eigen::Affine3d tsdf_center)
{
	min_weight = min_weight_;	// minimum weight -- i.e. if a voxel sees a point less than 2 times, it will not render  a mesh triangle at that location
	tsdf.reset(new cpu_tsdf::TSDFVolumeOctree);
	tsdf->setGridSize (xsize, ysize, zsize); // m x m x m
	tsdf->setResolution (xres, yres, zres); // Smallest cell size = 10m / 2048 = about half a centimeter
	tsdf->setGlobalTransform (tsdf_center);
	tsdf->reset();
}
template <typename PointT> void TSDF::IntegrateCloud(pcl::PointCloud<PointT> cloud, Eigen::Affine3d trans)
{
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	normals->clear();
	tsdf->integrateCloud (cloud, normals, trans);
}
pcl::PointCloud< pcl::PointNormal >::Ptr TSDF::GetCloud(Eigen::Affine3d trans)
{
	return tsdf->renderView(trans);
}
pcl::PolygonMesh TSDF::GetMesh()
{
	cpu_tsdf:: MarchingCubesTSDFOctree mc;
	mc.setInputTSDF (tsdf);
	mc.setMinWeight (min_weight); 
	mc.setColorByRGB (false); 
	pcl::PolygonMesh mesh;
	mc.reconstruct (mesh);
	return mesh;
}