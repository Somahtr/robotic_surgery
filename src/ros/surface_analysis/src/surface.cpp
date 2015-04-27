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
    pointCloud = this->Segment(pointCloud,0,1.5);
    
    pclPub.publish(pointCloud);
}

PointCloud::Ptr SurfaceAnalysisNode::Segment(PointCloud::ConstPtr cloud, int zMin, int zMax)
{
	cout << "Segmenting point cloud between " << zMin << " and " << zMax << endl;
	
	PointCloud::Ptr filteredCloud (new PointCloud);
	
	pcl::PassThrough<pcl::PointXYZ> pass;
  	pass.setInputCloud (cloud);
  	pass.setFilterFieldName ("z");
  	pass.setFilterLimits (zMin, zMax);
  	//pass.setFilterLimitsNegative (true);
  	pass.filter(*filteredCloud);
  	
  	return filteredCloud;
}

PointCloud::Ptr SurfaceAnalysisNode::EstimateNormals(PointCloud::ConstPtr cloud)
{
}

