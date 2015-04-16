#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::Normal> NormalCloud;

class Visualiser
{
    PointCloud::ConstPtr cloud;
    NormalCloud::ConstPtr normals;
    bool hasCloud;
    bool hasNormals;
public:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	
	Visualiser(void);
    void loadPointCloud(const PointCloud::ConstPtr&);
    void loadNormals(const NormalCloud::ConstPtr&);
};
