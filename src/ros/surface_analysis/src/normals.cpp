#include "normals.h"

/// Moving least squares normal estimation ///
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
	
	// Reverse direction of normals
	for (size_t i = 0; i < normalsOut->points.size (); ++i)
	{
		normalsOut->points[i].normal_x *=-1;
		normalsOut->points[i].normal_y *=-1;
		normalsOut->points[i].normal_z *=-1;
	}
}

/// Basic normal estimation ///
void NormalsBasic::estimateNormals(const PointCloud::ConstPtr& cloudIn, PointCloudNormals::Ptr normalsOut)
{
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	
	// Create KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloudIn);
	
	// Perform normal estimation
	normEst.setInputCloud (cloudIn);
	normEst.setSearchMethod (tree);
	normEst.setKSearch (20);
	normEst.compute (*normals);

	// Concatenate the XYZ and normal fields
	pcl::concatenateFields (*cloudIn, *normals, *normalsOut);
}

// Output surface curvature values to a file for evaluation purposes
void NormalsBase::outputCurvatures(const PointCloudNormals::ConstPtr& cloud, const char* fname)
{
	ofstream file;
	file.open(fname);
	cout << "Writing curvature data to file '" << fname << "'" << endl;
	for (size_t i = 0; i < cloud->points.size (); ++i)
		file << i << ", " << cloud->points[i].curvature << endl;
		
	file.close();
	cout << "Data written successfully" << endl;
}
