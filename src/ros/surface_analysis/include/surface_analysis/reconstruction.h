#include "surface.h"
#include <pcl/surface/gp3.h>

class ReconstructionBase
{
public:
	virtual void reconstruct(const PointNormalCloud::ConstPtr&, PolygonMesh::Ptr meshOut) = 0;
};

class GreedyTriangulation : ReconstructionBase
{
public:
	void reconstruct(const PointNormalCloud::ConstPtr&, PolygonMesh::Ptr meshOut);
};
