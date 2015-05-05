#include "surface.h"
#include "segment.h"
#include "normals.h"

class SurfaceAnalysisNode
{
    ros::NodeHandle node;
    
    // Subscribers
    ros::Subscriber subInputCloud;

    // Publishers
    ros::Publisher pubProcessedCloud;
    ros::Publisher pubNormals;
    
    PointCloud::ConstPtr pointCloud;
        
public:
    SurfaceAnalysisNode(void);
    
    void LoadPointCloud(const PointCloud::ConstPtr&);
    
    PointCloud::Ptr Segment(PointCloud::ConstPtr, int, int);
    PointNormalCloud EstimateNormals(PointCloud::ConstPtr); 
};
