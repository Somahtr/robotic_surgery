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
	pclSub = node.subscribe("inputCloud",10, &SurfaceAnalysisNode::LoadPointCloud,this);
	pclPub = node.advertise<PointCloud>("outputCloud", 1);
}



void SurfaceAnalysisNode::LoadPointCloud(const PointCloud::ConstPtr& msg)
{
    cout << "Point cloud recieved" << endl;
    pointCloud = msg;
    
    // Perform segmentation
    this->Segment(pointCloud,0,1);
    
    pclPub.publish(pointCloud);
}

void SurfaceAnalysisNode::Segment(PointCloud::ConstPtr cloud, int zMin, int zMax)
{
	
}



