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
	pubMesh = node.advertise<pcl_msgs::PolygonMesh>("Surface/Mesh", 1);
}



void SurfaceAnalysisNode::LoadPointCloud(const PointCloud::ConstPtr& msg)
{
    cout << "Point cloud recieved" << endl;
    
    // Segmentation
    SegmentFilter filter;
    PointCloud::Ptr filteredCloud (new PointCloud);
    filter.segment(msg, filteredCloud);
    
    // Visualise segmented cloud 
    pubProcessedCloud.publish(*filteredCloud);
    
    // Normal estimation
    NormalsBasic ne;
    PointCloudNormals::Ptr normals (new PointCloudNormals);
    ne.estimateNormals(filteredCloud, normals);
    
    // Visualise point cloud and normals
    pubNormals.publish(*normals);
    //ne.outputCurvatures(normals,"curvaturesbasic050515panda.csv");
    
    // Mesh reconstruction
    GreedyTriangulation recon;
    PolygonMesh::Ptr mesh (new PolygonMesh);
    recon.reconstruct(normals, mesh);
    
    // Visualise mesh
    pcl_msgs::PolygonMesh meshMsg;
    pcl_conversions::fromPCL(*mesh, meshMsg);
    pubMesh.publish(meshMsg);
}

