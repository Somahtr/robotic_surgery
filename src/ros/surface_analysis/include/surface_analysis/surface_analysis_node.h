#include "surface.h"

#include <pcl_msgs/PolygonMesh.h>
#include <pcl/conversions.h>

#include "segment.h"
#include "normals.h"
#include "reconstruction.h"

class SurfaceAnalysisNode
{
    ros::NodeHandle node;
    
    // Subscribers
    ros::Subscriber subInputCloud;

    // Publishers
    ros::Publisher pubProcessedCloud;
    ros::Publisher pubNormals;
    ros::Publisher pubMesh;
    
        
public:
    SurfaceAnalysisNode(void);   
    void LoadPointCloud(const PointCloud::ConstPtr&);
};
