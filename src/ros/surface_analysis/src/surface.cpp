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
	//: vis(new Visualiser)
{
	// Create publishers and subscribers
	subInputCloud = node.subscribe("Surface/InputCloud",10, &SurfaceAnalysisNode::LoadPointCloud,this);
	
	pubProcessedCloud = node.advertise<PointCloud>("Surface/ProcessedCloud", 1);
	pubNormals = node.advertise<PointNormalCloud>("Surface/Normals", 1);
}



void SurfaceAnalysisNode::LoadPointCloud(const PointCloud::ConstPtr& msg)
{
    cout << "Point cloud recieved" << endl;
    pointCloud = msg;
    
    // Perform segmentation
    pointCloud = this->Segment(pointCloud,0,1.5);  
    pubProcessedCloud.publish(pointCloud);
    
    // Perform smoothed normal estimation
    PointNormalCloud pointNormals = EstimateNormals(pointCloud);
    
    // Visualise point cloud and normals
    pubNormals.publish(pointNormals);
}

PointCloud::Ptr SurfaceAnalysisNode::Segment(PointCloud::ConstPtr cloud, int zMin, int zMax)
{
	cout << "Segmenting point cloud between " << zMin << " and " << zMax << endl;
	
	PointCloud::Ptr filteredCloud (new PointCloud);
	
	pcl::PassThrough<pcl::PointXYZ> pass;
  	pass.setInputCloud (cloud);
  	pass.setFilterFieldName ("z");
  	pass.setFilterLimits (zMin, zMax);
  	pass.filter(*filteredCloud);
  	
  	return filteredCloud;
}

PointNormalCloud SurfaceAnalysisNode::EstimateNormals(PointCloud::ConstPtr cloud)
{
	// Create a KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  
	// Output has the PointNormal type in order to store the normals calculated by MLS
	PointNormalCloud normals;

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
 
	mls.setComputeNormals (true);

	// Set parameters
	mls.setInputCloud (cloud);
	mls.setPolynomialFit (true);
	mls.setSearchMethod (tree);
	mls.setSearchRadius (0.03);

	// Reconstruct
	mls.process (normals);
	
	return normals;
}

