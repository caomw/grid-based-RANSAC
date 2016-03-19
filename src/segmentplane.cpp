#include "segmentPlane.h"


void 
segmentPlane::setGrids()
{
	
	for(int i = 0;i<100;i++)
	{
		subPlane temp;
		temp.getCloud(cloud,i);
		planes.push_back(temp);
	}
}


void 
segmentPlane::segAllPlane()
{
	for(int i=0;i<100;i++)
	{
		planes[i].run();
		if(planes[i].isPlane)
		{
			//cout<<"GET A NEW PLANE";
			planes[i].color_plane();
			//planes[i].color_back_to_father(cloud);
		}
		//planes[i].show();
		//std::cout<<"planes[i].id:"<<planes[i].plane_id<<std::endl;
	}
	for(int i=0;i<100;i++)
	{
		//planes[i].run();
		if(planes[i].isPlane)
		{
			//cout<<"GET A NEW PLANE";
			//planes[i].color_plane();
			planes[i].color_back_to_father(cloud);
		}
		//planes[i].show();
		//std::cout<<"planes[i].id:"<<planes[i].plane_id<<std::endl;
	}
}


void 
segmentPlane::setCloud(pcl::PointCloud<PointT>::Ptr cloudin)
{
	cloud = cloudin;
}