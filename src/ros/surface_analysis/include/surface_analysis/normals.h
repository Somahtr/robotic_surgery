// Header file for performing normal estimation

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudNormals;

class NormalsBase
{
public:
	virtual void estimateNormals(const PointCloud::ConstPtr&, PointCloudNormals::Ptr) = 0;
};

class NormalsMLS : public NormalsBase
{
public:
	void estimateNormals(const PointCloud::ConstPtr&, PointCloudNormals::Ptr);
}; 

class NormalsBasic : public NormalsBase
{
public:
	void estimateNormals(const PointCloud::ConstPtr&, PointCloudNormals::Ptr);
};
