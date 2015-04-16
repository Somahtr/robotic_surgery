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
    ros::Subscriber cloudSub = node.subscribe<pcl::PointCloud<pcl::PointXYZ> >("cloud",10, &Visualiser::loadPointCloud, vis);
    ros::Subscriber normalSub = node.subscribe<pcl::PointCloud<pcl::Normal> >("normals",10, &Visualiser::loadNormals, vis);
    
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
	: viewer(new pcl::visualization::PCLVisualizer("Robotic Surgery Point Cloud Viewer")),
	cloud(new pcl::PointCloud<pcl::PointXYZ>),
	normals(new pcl::PointCloud<pcl::Normal>)
{
	// Create a test point cloud
	PointCloud::Ptr pcl (new PointCloud);
	pcl->push_back(pcl::PointXYZ(1,1,0));
	
	// Test normals
	NormalCloud::Ptr nml (new NormalCloud);
	nml->push_back(pcl::Normal(1,0,0));
	
	// Assign the point cloud pointers
	cloud = pcl;
	normals = nml;
	
	hasCloud = false;
	hasNormals = false;
	
    viewer->setBackgroundColor (0, 0, 0);
    
    viewer->addPointCloud<pcl::PointXYZ>(this->cloud);
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
     
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
    
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
}

void Visualiser::loadPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
{
    cout << "pclvis:Point cloud recieved" << endl;
    this->cloud = msg;
    hasCloud = true;
    viewer->updatePointCloud<pcl::PointXYZ>(this->cloud);
    viewer->updateCamera();    
}

void Visualiser::loadNormals(const pcl::PointCloud<pcl::Normal>::ConstPtr& msg)
{
	if(!hasCloud) {
		cout << "Normals cannot be displayed: no point cloud added" << endl;
		return;
	}
    cout << "Surface normals recieved" << endl;
    this->normals = msg;
}
