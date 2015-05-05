#include "normals.h"

void NormalsMLS::estimateNormals(const PointCloud::ConstPtr& cloudIn, PointCloudNormals::Ptr normalsOut)
{
	// Create a KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  
	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
 
	mls.setComputeNormals (true);

	// Set parameters
	mls.setInputCloud (cloudIn);
	mls.setPolynomialFit (true);
	mls.setSearchMethod (tree);
	mls.setSearchRadius (0.03);

	// Reconstruct
	mls.process (*normalsOut);
}
