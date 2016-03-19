#ifndef _SEGMENTPLANE_H__
#define _SEGMENTPLANE_H__

#include "subPlane.h"

class segmentPlane
{
public:
	segmentPlane(){
		sub_w= 64;
		sub_h = 48;
	}
	std::vector<subPlane> planes;
	pcl::PointCloud<PointT>::Ptr cloud;
	int sub_w;
	int sub_h;
	void segAllPlane();
	void setGrids();
	void setCloud(pcl::PointCloud<PointT>::Ptr cloudin);
};

#endif //_SEGMENTPLANE_H__
