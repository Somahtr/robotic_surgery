#include "reconstruction.h"

void GreedyTriangulation::reconstruct(const PointNormalCloud::ConstPtr& cloudIn, PolygonMesh::Ptr meshOut)
{
	// Create search tree
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
	tree->setInputCloud (cloudIn);

	// Initialize object
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp;

	// Set the maximum distance between connected points (maximum edge length)
	gp.setSearchRadius (0.025);

	// Set typical values for the parameters
	gp.setMu (2.5);
	gp.setMaximumNearestNeighbors (100);
	gp.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp.setMinimumAngle(M_PI/18); // 10 degrees
	gp.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp.setNormalConsistency(false);

	// Get result
	gp.setInputCloud (cloudIn);
	gp.setSearchMethod (tree);
	gp.reconstruct (*meshOut);
}
