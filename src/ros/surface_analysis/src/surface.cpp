#include "surface_analysis_node.h"
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
	subInputCloud = node.subscribe("Surface/InputCloud",10, &SurfaceAnalysisNode::LoadPointCloud,this);
	
	pubProcessedCloud = node.advertise<PointCloud>("Surface/ProcessedCloud", 1);
	pubNormals = node.advertise<PointNormalCloud>("Surface/Normals", 1);
}



void SurfaceAnalysisNode::LoadPointCloud(const PointCloud::ConstPtr& msg)
{
    cout << "Point cloud recieved" << endl;
    
    // Perform segmentation
    SegmentFilter filter;
    PointCloud::Ptr filteredCloud (new PointCloud);
    filter.segment(msg, filteredCloud);
     
    pubProcessedCloud.publish(*filteredCloud);
    
    // Perform smoothed normal estimation
    NormalsBasic ne;
    PointCloudNormals::Ptr normals (new PointCloudNormals);
    ne.estimateNormals(filteredCloud, normals);
    
    // Visualise point cloud and normals
    pubNormals.publish(*normals);
    ne.outputCurvatures(normals,"curvaturesbasic050515panda.csv");
}

