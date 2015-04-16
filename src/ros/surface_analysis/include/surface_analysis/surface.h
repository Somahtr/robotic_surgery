#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


class SurfaceAnalysisNode
{
    ros::NodeHandle node;
    ros::Subscriber pclSub;
    ros::Subscriber pointSub;
    ros::Publisher pclPub;
    
    PointCloud::ConstPtr pointCloud;
        
public:
    SurfaceAnalysisNode(void);
    void LoadPointCloud(const PointCloud::ConstPtr&);
    void Segment(PointCloud::ConstPtr, int, int); 
};
