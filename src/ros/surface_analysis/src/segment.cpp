#include "segment.h"
using namespace std;

#define ZMIN 0
#define ZMAX 1
 
// Most basic segmentation using a simple passthrough filter
void SegmentFilter::segment(const PointCloud::ConstPtr& cloudIn, PointCloud::Ptr& cloudOut)
{
	cout << "Segmenting point cloud between " << ZMIN << " and " << ZMAX << endl;
		
	pcl::PassThrough<pcl::PointXYZ> pass;
  	pass.setInputCloud (cloudIn);
  	pass.setFilterFieldName ("z");
  	pass.setFilterLimits (ZMIN, ZMAX);
  	pass.filter(*cloudOut);  	
}
