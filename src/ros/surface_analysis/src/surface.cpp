#include "surface.h"
using namespace std;

int main(int argc, char *argv[])
{
	// Initialise the surface analysis node
	ros::init(argc,argv,"surface");

	// Create analyser object
	SurfaceAnalyser* surface (new SurfaceAnalyser());

	// Wait for incoming messages
	ros::spin();
	
	cout << "Node shutting down" << endl;
	return 0;
}



// --------------------------------------------------------------------
// ---Surface Analyser Class ------------------------------------------
// --------------------------------------------------------------------

SurfaceAnalyser::SurfaceAnalyser()
{
	// Create publishers and subscribers
	pcPub = node.advertise<PointCloud>("surface/pointCloud", 1);
	depthSub = node.subscribe("depthImage",10, &SurfaceAnalyser::LoadDepth,this);

	// Camera calibration matrix parameters
	calib.fx = 1;
	calib.fy = 1;
	calib.cx = 262;
	calib.cy = 212;
}

	// Create a point cloud from a depth image
PointCloud::Ptr SurfaceAnalyser::CreatePointCloud(const sensor_msgs::Image::ConstPtr& depth)
{
	cout << "Generating point cloud" << endl;

	// Create a new point cloud object
	PointCloud::Ptr pc (new PointCloud);
	pc->header.frame_id = "pc_frame";
	pc->height = depth->height;
	pc->width = depth->width;
 	pc->is_dense = true;
        pc->points.resize(depth->height*depth->width); 
	cout << "Populating point data" << endl;
	cout << "Point cloud w = " << pc->width << " h = " << pc->height << endl;
	int x, y, i, Z;
	// Populate it with points
	for(x=0; x<depth->width; x++)
	{
		for(y=0; y<depth->height; y++)
		{ 
			i = y * depth->width + x;
			Z = depth->data[i]; 
			cout << "x: " << x << " y: "<< y << " z: " << Z << " w = " << pc->width << " h = " << pc->height << endl;
			pc->points[i] = pcl::PointXYZ(x,y,Z);
		}
	}

	return pc;
}

void SurfaceAnalyser::LoadDepth(const sensor_msgs::Image::ConstPtr& msg)
{
	cout << "Depth image recieved" << endl;

	// Create a pointcloud from the depth information
	PointCloud::Ptr pc = CreatePointCloud(msg);
	
	cout << "Point cloud created: width = " << pc->width << " height = " << pc->height << endl;
	
	// Publish to the visualiser
	pc->header.stamp = ros::Time::now().toSec();
	pcPub.publish(pc);
	cout << "Broadcasting point cloud" << endl;
	
}
