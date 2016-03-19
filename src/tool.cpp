#include "tool.h"

void 
polish_cloud(pcl::PointCloud<PointT>::Ptr cloud)
{
	// maybe I need a filter to smooth it 
	int height = cloud->height;
	int width = cloud->width;
	for(int i=0;i<height;i++)
	{
		for(int j=0;j<width;j++)
		{
			int index = i * height +j;
			if(!(cloud->points[index].z >0.7 && cloud->points[index].z<10) || 
				!(cloud->points[index].x <10 && cloud->points[index].x>-10) ||
				!(cloud->points[index].y <10 && cloud->points[index].y>-10))
			{
				std::cout<<"i:"<<i<<"  j:"<<j<<std::endl;
				cloud->points[index].x = 0;
				cloud->points[index].y = 0;
				cloud->points[index].z = 0;
				//cloud->points[index].b = 100;
			//cloud->points[index].g = 0;
			//cloud->points[index].r = 100;
			}
			
		}
	}
}


void show_a_img(pcl::PointCloud<PointT>::Ptr cloud)
{
	std::string ID = "1";

	pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler(cloud);
	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("viewer"));
	viewer->setBackgroundColor(0,50,100);
	viewer->addPointCloud<PointT> (cloud,rgb_handler,ID);

	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,ID);

	while(!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep (boost::posix_time::microseconds(100000));
	}

	//std::cout<<"end show"<<std::endl;
}

/*
pcl::PointCloud<PointT>::Ptr aim_cloud (new pcl::PointCloud<PointT>);
  aim_cloud->width = 640;
  aim_cloud->height = 480;
  aim_cloud->is_dense = true;
  aim_cloud->resize (aim_cloud->width * aim_cloud->height);

  PointT t_p;
  PointT s_p;
  int idx_1;
  int idx_2;
  int num = 0;
  int n   = 0;
  int k   = 0;
  
  float min_x = cloud_plane->points[0].x; float max_x = cloud_plane->points[0].x;
  float min_z = cloud_plane->points[0].z; float max_z = cloud_plane->points[0].z; 
  for (int i = 1; i < cloud_plane->size(); i++)
  { 
		if (min_x > cloud_plane->points[i].x)
			min_x = cloud_plane->points[i].x;
    if (max_x < cloud_plane->points[i].x)
			max_x = cloud_plane->points[i].x;
		if (min_z > cloud_plane->points[i].z)
			min_z = cloud_plane->points[i].z;
		if (max_z < cloud_plane->points[i].z)
			max_x = cloud_plane->points[i].z;
	}

//遍历
  for (int h = 0; h < 480; h++)
  {
		for (int w = 0; w < 640; w++)
		{
			if (cloud->points[h*480+w].x >= min_x && cloud->points[h*480+w].x <= max_x
				&&
					cloud->points[h*480+w].z >= min_z && cloud->points[h*480+w].z <= max_z
				&&
					coefficients_plane->values[0]*cloud->points[h*480+w].x
				 +coefficients_plane->values[1]*cloud->points[h*480+w].y
				 +coefficients_plane->values[2]*cloud->points[h*480+w].z
				 +coefficients_plane->values[3] > 0.01)
			{
				aim_cloud->points[h*480+w].x = cloud->points[h*480+w].x;
				aim_cloud->points[h*480+w].y = cloud->points[h*480+w].y;
				aim_cloud->points[h*480+w].z = cloud->points[h*480+w].z;
				aim_cloud->points[h*480+w].r = cloud->points[h*480+w].r;
				aim_cloud->points[h*480+w].g = cloud->points[h*480+w].g;
				aim_cloud->points[h*480+w].b = cloud->points[h*480+w].b;
			}
			else
			{
         aim_cloud->points[h*480+w].x = cloud->points[h*480+w].x;
         aim_cloud->points[h*480+w].y = cloud->points[h*480+w].y;
         aim_cloud->points[h*480+w].z = cloud->points[h*480+w].z;
         aim_cloud->points[h*480+w].r = 0;
         aim_cloud->points[h*480+w].g = 0;
         aim_cloud->points[h*480+w].b = 0;
			}
		}
  }
  std::cout << "    Cutting objects from scene and saving." << std::endl;

  writer.write (argv[3], *aim_cloud, false);



	// Extract the planar inliers from the input cloud
	  //extract.setInputCloud (cloud);
	  //extract.setIndices (inliers_plane);
	  //extract.setNegative (false);

	// Write the planar inliers to disk
	  //pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
	  //extract.filter (*cloud_plane);
*/