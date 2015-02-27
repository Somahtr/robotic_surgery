#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


void LoadInfo(const sensor_msgs::CameraInfo);

struct CalibrationMatrix
{
	float fx, fy, cx, cy;
};

class SurfaceAnalyser
{
	CalibrationMatrix calib;
	ros::Publisher pub;
public:
	SurfaceAnalyser(void);
	PointCloud* CreatePointCloud(const sensor_msgs::Image::ConstPtr&);
	PointCloud* Segment(PointCloud*);
	void LoadDepth(const sensor_msgs::Image::ConstPtr&);
};
