#include "surface.h"
using namespace std;

int main(int argc, char *argv[])
{
	// Initialise the surface analysis node
	ros::init(argc,argv,"surface");

	// Create analyser object
	SurfaceAnalysisNode surface;

	// Wait for incoming messages
	ros::spin();
	
	cout << "Node shutting down" << endl;
	return 0;
}


// --------------------------------------------------------------------
// ---Surface Analysis Node Class -------------------------------------
// --------------------------------------------------------------------

/// Constructor
SurfaceAnalysisNode::SurfaceAnalysisNode(void)
{
	// Create publishers and subscribers
	pcSub = node.subscribe("pointCloud",10, &SurfaceAnalysisNode::LoadPointCloud,this);
}



void SurfaceAnalysisNode::LoadPointCloud(const sensor_msgs::PointCloud2::ConstPtr&)
{
    cout << "Point cloud recieved" << endl;
}



