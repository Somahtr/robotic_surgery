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
    
        
public:
    SurfaceAnalysisNode(void);   
    void LoadPointCloud(const PointCloud::ConstPtr&);
};
