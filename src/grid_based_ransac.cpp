
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
#include "subPlane.h"
#include "segmentPlane.h"

void usage(char* prog)
{
	std::cout << " The program is to segment objects from scene.\n Then use cluster to seperate each object. \n";
// 单个场景pcd文件（输入，480×640），场景中的平面pcd（输出），平面上的物体pcd（输出）
	std::cout << prog << " scene.pcd plane.pcd obj.pcd" << std::endl;
}


int main (int argc, char** argv)
{
  if (argc < 2) { usage(argv[0]); exit(1); }

  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PCDReader reader;
  reader.read(argv[1],*cloud);
  polish_cloud(cloud);

  segmentPlane seg;
  seg.setCloud(cloud);
  seg.setGrids();
  seg.segAllPlane();

  cloud = seg.cloud;

  /*subPlane testplane;
  testplane.getCloud(cloud,79);
  testplane.show();
  polish_cloud(testplane.cloud);
  testplane.show();*/


    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler(cloud);
	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("viewer"));
	viewer->setBackgroundColor(0,0,0);
	viewer->addPointCloud<PointT> (cloud,rgb_handler,"image_viewer");

	//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,ID);

	while(!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep (boost::posix_time::microseconds(100000));
	}

  pcl::PCDWriter writer;
  writer.write("test_output.pcd",*cloud,false);
  return 0;
}

