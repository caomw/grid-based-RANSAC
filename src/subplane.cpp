#include <subPlane.h>
#include <exception>
#ifndef NAN
#define NAN (0.0/0.0)
#endif 
void 
subPlane::run()
{	
	//if(plane_id == 79 || plane_id == 80 || plane_id == 83
	//	|| plane_id == 85 || plane_id == 86) return;
	if(plane_id > 78)return ;
	try
	{
	  ne.setSearchMethod (tree);
	  ne.setInputCloud (cloud);
	  ne.setKSearch (50);
	  ne.compute (*cloud_normals);

	// Create the segmentation object for the planar model and set all the parameters
	  seg.setOptimizeCoefficients (true);
	  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
	  seg.setNormalDistanceWeight (0.1);
	  seg.setMethodType (pcl::SAC_RANSAC);
	  seg.setMaxIterations (100);
	  seg.setDistanceThreshold (0.08);
	  seg.setInputCloud (cloud);
	  std::cout<<cloud<<std::endl;
	  seg.setInputNormals (cloud_normals);
	// Obtain the plane inliers and coefficients
	  seg.segment (*inliers_plane, *coefficients_plane);
	  
	  //check plane 
	  if(inliers_plane->indices.size()>2000)
	  	{
	  		isPlane= true;
	  		std::cout<<"is a plane "<< "  ";
	  	}
	  show_test_info();
	}
	catch(std::exception& e)
	{
		std::cout<<endl;
		std::cout<<"nan data accurred!"<<std::endl;
		isPlane=false;
	 // std::cout<<"end run"<<std::endl;
	}
}

void subPlane::show()
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

void 
subPlane::color_plane()
{
	int b,g,r;
	b = (plane_id * 10) % 255;
	g = (plane_id * 20) % 255;
	r = (plane_id * 30) % 255;
	if(isPlane)
		for(int i=0;i<inliers_plane->indices.size();i++)
		{
			cloud->points[inliers_plane->indices[i]].b = b;
			cloud->points[inliers_plane->indices[i]].g = g;
			cloud->points[inliers_plane->indices[i]].r = r;
		}
}

void 
subPlane::show_test_info()
{
	std::cout<<"Plane ID    :"<<plane_id<<std::endl;
	std::cout<<"----plane coefficients----"<<std::endl;
	std::cout<<coefficients_plane->values[0]<<","
			 <<coefficients_plane->values[1]<<","
			 <<coefficients_plane->values[2]<<","
			 <<coefficients_plane->values[3]<<std::endl;
	std::cout<<"Total Points:"<<cloud->size();
	std::cout<<"Inliers     :"<<inliers_plane->indices.size()<<std::endl;
}

void 
subPlane::getCloud(pcl::PointCloud<PointT>::Ptr cloudin,int id)
{
	if(id<0 || id>=100)
	{
		std::cout<<id<<" is outofrange!"<<std::endl;
		return;
	}
	plane_id = id;
	std::cout<<"id:"<<id<<std::endl;

	cloud->width = 64;
	cloud->height = 48;
	cloud->is_dense = true;
	cloud->resize(cloud->width * cloud->height);
	
	int base_index = (id/10)*640*48+(id%10)*64;
	for(int i=0;i<64;i++)
	{
		for(int j=0;j<48;j++)
		{
			int index = j*64 +i;
			int index2 = j*640 +i + base_index;
			cloud->points[index]=(cloudin->points[index2]);			
		}
	}
}

void 
subPlane::color_back_to_father(pcl::PointCloud<PointT>::Ptr& origin)
{
	int base_index = (plane_id/10)*640*48+(plane_id%10)*64;
	for(int i=0;i<64;i++)
	{
		for(int j=0;j<48;j++)
		{
			int index = j*64 +i;
			int index2 = j*640 +i + base_index;
			origin->points[index2].g=cloud->points[index].g;
			origin->points[index2].b=cloud->points[index].b;
			origin->points[index2].r=cloud->points[index].r;	
		}
	}
	std::cout<<"has colored back!";
	//show_a_img(origin);
}


