#ifndef _SUBPLANE_H__
#define _SUBPLANE_H__

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

#include "tool.h"

class subPlane
{
public:
	// main tool 
	subPlane():plane_id(0){
		tree = pcl::search::KdTree<PointT>::Ptr(new pcl::search::KdTree<PointT>);
		cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
		cloud_normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
		coefficients_plane = pcl::ModelCoefficients::Ptr (new pcl::ModelCoefficients);
		inliers_plane = pcl::PointIndices::Ptr(new pcl::PointIndices);
		isPlane = false;

	}
	subPlane(int a):plane_id(a){
		tree = pcl::search::KdTree<PointT>::Ptr(new pcl::search::KdTree<PointT>);
		cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
		cloud_normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
		coefficients_plane = pcl::ModelCoefficients::Ptr (new pcl::ModelCoefficients);
		inliers_plane = pcl::PointIndices::Ptr(new pcl::PointIndices);
		isPlane = false;
	}
	virtual ~subPlane(){}
	int plane_id;// the id of the object
	bool isPlane; // if this object has plane feature
	int cluster_id;// the id of the cluster it belongs


	pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;

    //pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;

    pcl::search::KdTree<PointT>::Ptr tree; 
    // main data of the sub plane in 64*48
    //USEFUL:coefficients_plane
	pcl::PointCloud<PointT>::Ptr cloud;
	pcl::PointCloud<pcl::Normal>::Ptr  cloud_normals;
	pcl::ModelCoefficients::Ptr coefficients_plane;
	pcl::PointIndices::Ptr inliers_plane;


	//function 
	void getCloud(pcl::PointCloud<PointT>::Ptr cloudin,int id); // get sub plane according to the id 
	void run(); // estimate sub plane 
	void show(); // visualize the sub picture 
	void color_plane(); // give the plane target color 
	void show_test_info(); // output coefficients of the plane 
	void color_back_to_father(pcl::PointCloud<PointT>::Ptr& origin);
};


#endif // _SUBPLANE_H__