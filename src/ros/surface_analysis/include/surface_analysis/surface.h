#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

#include "pcl_visualiser.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointNormal> PointNormalCloud;

class SurfaceAnalysisNode
{
    ros::NodeHandle node;
    
    // Subscribers
    ros::Subscriber subInputCloud;

    // Publishers
    ros::Publisher pubProcessedCloud;
    ros::Publisher pubNormals;
    
    PointCloud::ConstPtr pointCloud;
        
public:
    SurfaceAnalysisNode(void);
    
    void LoadPointCloud(const PointCloud::ConstPtr&);
    
    PointCloud::Ptr Segment(PointCloud::ConstPtr, int, int);
    PointNormalCloud EstimateNormals(PointCloud::ConstPtr); 
};
