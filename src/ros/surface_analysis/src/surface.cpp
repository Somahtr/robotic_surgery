#include "surface.h"
using namespace std;

int main(int argc, char *argv[])
{
	// Initialise the surface analysis node
	ros::init(argc,argv,"surface");
	ros::NodeHandle node;

	// Create analyser object
	SurfaceAnalyser* surface (new SurfaceAnalyser());

	// Subscribe to incoming depth image messages
	ros::Subscriber sub = node.subscribe("depthImage",10, &SurfaceAnalyser::LoadDepth,surface);

	// Wait for incoming messages
	ros::spin();

	return 0;
}



// --------------------------------------------------------------------
// ---Surface Analyser Class ------------------------------------------
// --------------------------------------------------------------------

SurfaceAnalyser::SurfaceAnalyser()
{
	calib.fx = 1;
	calib.fy = 1;
	calib.cx = 262;
	calib.cy = 212;
}

	// Create a point cloud from a depth image
PointCloud* SurfaceAnalyser::CreatePointCloud(const sensor_msgs::Image::ConstPtr& depth)
{
	// Create a new point cloud object
	PointCloud* pc (new PointCloud);

	for(int i=0; i<depth->height*depth->width; i++)
	{
		int x = i % depth->width;
		int y = i / depth->width;

		int Z = depth->data[i];
		cout << "x: " << x << "y: " << "z: " << Z << endl;
	}
	cin.ignore();
}

void SurfaceAnalyser::LoadDepth(const sensor_msgs::Image::ConstPtr& msg)
{
	cout << "Depth image recieved" << endl;
	CreatePointCloud(msg);
}
