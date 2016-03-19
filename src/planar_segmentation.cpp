#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <cmath>
#include <ctime>
#include <iostream>

typedef pcl::PointXYZRGB PointT;

void usage(char* prog)
{
	std::cout << " The program is to segment objects from scene.\n Then use cluster to seperate each object. \n";
// 单个场景pcd文件（输入，480×640），场景中的平面pcd（输出），平面上的物体pcd（输出）
	std::cout << prog << " scene.pcd plane.pcd obj.pcd" << std::endl;
}

int main (int argc, char** argv)
{
  if (argc < 4) { usage(argv[0]); exit(1); }

// All the objects needed
  pcl::PCDReader reader;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

// Datasets
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);

// Read in the cloud data
  reader.read (argv[1], *cloud);
	//  std::cerr << "    PointCloud has: " << cloud->points.size () << " data points." << std::endl;

// Estimate point normals
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
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud);
  seg.setInputNormals (cloud_normals);
// Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
	// std::cerr << "    Plane coefficients: " << *coefficients_plane << std::endl;

// Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

// Write the planar inliers to disk
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_plane);
//这是一些无关的输出，如果调试的时候可以用的上
	//  std::cerr << "    PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
	//  writer.write (argv[2], *cloud_plane, false);
	//  std::cerr << "    Cutting plane from scene and saving." << std::endl;
  
  int total_size = cloud->size();
  int plane_size = cloud_plane->size();

//除去平面之外的点的个数
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

  return 0;
}

