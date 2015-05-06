#include "pcl_visualiser.h"
using namespace std;

int main(int argc, char *argv[])
{
    // Initialise the point cloud visualiser node
	ros::init(argc,argv,"pclvis");
	ros::NodeHandle node;
    cout << "Point cloud visualiser node started" << endl;
    
    // Create the visualiser object and subscribe to topics
    Visualiser* vis (new Visualiser);
    ros::Subscriber normalSub = node.subscribe<PointNormalCloud>("Visualiser/Normals",10, &Visualiser::visualiseNormals, vis);
    ros::Subscriber meshSub = node.subscribe<pcl_msgs::PolygonMesh>("Visualiser/Mesh",10, &Visualiser::visualiseMesh, vis);
    
	// Wait for incoming messages
	ros::Rate loop_rate(4);
    while (node.ok() && !vis->viewer->wasStopped ())
    {
    	vis->viewer->spinOnce (100);
        ros::spinOnce ();
        loop_rate.sleep ();
    }
	
	cout << "Node shutting down" << endl;
	
	delete vis;
	return 0;
}

Visualiser::Visualiser()
	: viewer(new pcl::visualization::PCLVisualizer("Robotic Surgery Point Cloud Viewer"))
{
    viewer->setBackgroundColor (0, 0, 0);
       
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
}


void Visualiser::visualiseNormals(const PointNormalCloud::ConstPtr& msg)
{
	PointCloudRGB::Ptr cloud (new PointCloudRGB);
	NormalCloud::Ptr normals (new NormalCloud);
	
	// Split the point-normal cloud into its separate points and normals
	pcl::copyPointCloud(*msg, *cloud);
	pcl::copyPointCloud(*msg, *normals);
	
	// Colour points according to their depth
	this->recolour(cloud);
	
	// Update point cloud 
	if(!viewer->updatePointCloud(cloud, "cloud"))
	{
		// Add a new point cloud if one does not exist
		viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "cloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
	}
	
	// Update normals
	viewer->removePointCloud("normals", 0);
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals, 10, 0.02, "normals");
}

void Visualiser::visualiseMesh(const pcl_msgs::PolygonMesh::ConstPtr& msg)
{
	// Convert from pcl_msgs to pcl mesh
	PolygonMesh mesh;
	pcl_conversions::toPCL(*msg, mesh);
	
	if(!viewer->updatePolygonMesh(mesh, "mesh"))
	{
		// Add a new point cloud if one does not exist
		viewer->addPolygonMesh(mesh, "mesh");
	}
}

void Visualiser::recolour(PointCloudRGB::Ptr& cloud)
{
	// Find the minimum and maximum bounds of the point cloud
	pcl::PointXYZRGB ptMin, ptMax;
	pcl::getMinMax3D(*cloud, ptMin, ptMax);
	float zRange = ptMax.z - ptMin.z;
	
	// Iterate through all points
	for (size_t i = 0; i < cloud->points.size (); ++i)
	{
		cloud->points[i].r = 255*(cloud->points[i].z-ptMin.z)/zRange;
		cloud->points[i].g = 255*(1-(cloud->points[i].z-ptMin.z)/zRange);
	}
}
