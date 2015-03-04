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
	cameraSub = node.subscribe("cameraInfo",10, &SurfaceAnalyser::LoadCameraInfo,this);


	// Camera calibration matrix parameters
	calib.fx = 2000;
	calib.fy = 2000;
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
	
	// Populate it with points
	int u, v, i, X, Y, Z;
	for(u=0; u<depth->width; u++)
	{
		for(v=0; v<depth->height; v++)
		{ 
			i = v * depth->width + u;

			Z = depth->data[i];
			X = Z*(u-calib.cx)/calib.fx;
			Y = Z*(v-calib.cy)/calib.fy; 
			pc->points[i] = pcl::PointXYZ(X,Y,Z);
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

void SurfaceAnalyser::LoadCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
	cout << "Camera parameters recieved" << endl;
	// Load camera parameters from calibration matrix
	//calib.fx = msg->P[0];
	//calib.fy = msg->P[4];
	//calib.cx = msg->P[2];
	//calib.cy = msg->P[5];	
	//cout << "fx = " << calib.fx << " fy = " << calib.fy << " cx = " << calib.cx << " cy = " << calib.cy << endl; 

	cout << "Intrinsic camera matrix K" << endl;
	for(int i=1;i<8;i++)
	{   cout << msg->K[i] << ", ";}
	cout << msg->K[8] << ";" << endl;
	
	cout << "Rectification matrix R" << endl;
	for(int i=1;i<8;i++)
	{   cout << msg->R[i] << ", ";}
	cout << msg->R[8] << ";" << endl;
	
	cout << "Projection matrix P" << endl;
	for(int i=1;i<11;i++)
	{   cout << msg->P[i] << ", ";}
	cout << msg->P[11] << ";" << endl;
}
