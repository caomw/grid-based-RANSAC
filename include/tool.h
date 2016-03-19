#ifndef _TOOL_H__
#define _TOOL_H__


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
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <cmath>
#include <ctime>
#include <iostream>

typedef pcl::PointXYZRGB PointT;


void polish_cloud(pcl::PointCloud<PointT>::Ptr cloud); // fill nan point with 0
void polish_cloud_ave(pcl::PointCloud<PointT>::Ptr cloud); // fill nan with ave depth
void show_a_img(pcl::PointCloud<PointT>::Ptr cloud);
#endif //_TOOL_H__