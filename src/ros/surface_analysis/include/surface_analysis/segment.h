// Header file for classes for segmentation of the point cloud

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class SegmentationBase
{
public:
	virtual void segment(const PointCloud::ConstPtr&, PointCloud::Ptr&) = 0;
};

class SegmentFilter : public SegmentationBase
{
public:
	void segment(const PointCloud::ConstPtr&, PointCloud::Ptr&);
};
