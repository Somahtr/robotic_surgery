#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <pcl_msgs/PolygonMesh.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/conversions.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
typedef pcl::PointCloud<pcl::Normal> NormalCloud;
typedef pcl::PointCloud<pcl::PointNormal> PointNormalCloud;
typedef pcl::PolygonMesh PolygonMesh;

class Visualiser
{
public:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	
	Visualiser(void);
    void visualiseNormals(const PointNormalCloud::ConstPtr&);
    void visualiseMesh(const pcl_msgs::PolygonMesh::ConstPtr&);
private:
    void recolour(PointCloudRGB::Ptr&);
};
