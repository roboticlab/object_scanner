#include <object_scanner/object_scanner.h>

ObjectScanner::ObjectScanner(float min_weight_, float xsize, float ysize, float zsize, int xres, int yres, int zres, Eigen::Affine3d tsdf_center)
{
    ROS_INFO_STREAM("Object scanner created");
    _mover = new RobotsMover();
    _tsdf = new TSDF(min_weight_, xsize, ysize, zsize, xres, yres, zres, tsdf_center);	
    _cloud_processor = new CloudProcessor();
}
ObjectScanner::~ObjectScanner()
{
}
template <typename PointT> void ObjectScanner::IntegrateCloud(pcl::PointCloud<PointT> cloud, Eigen::Affine3d trans)
{
	_tsdf->IntegrateCloud(cloud, trans);
}
pcl::PointCloud< pcl::PointNormal >::Ptr ObjectScanner::GetCloud(Eigen::Affine3d trans)
{
	return _tsdf->GetCloud(trans);
}
pcl::PolygonMesh ObjectScanner::GetMesh()
{
	return _tsdf->GetMesh();
}
