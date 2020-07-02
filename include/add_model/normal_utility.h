#ifndef NORMAL_UTILITY_H_
#define NORMAL_UTILITY_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <numeric>
#include "access_model_data.h"

class Normal_Utility
{
	private:
		typedef pcl::PointXYZ PointT;
		typedef pcl::PointNormal Point_N;
		typedef pcl::PointCloud<PointT> PointCloudT;
		typedef pcl::PointCloud<Point_N> PointCloud_N;
		
		/**
		  Computes the normal utilities. A bad normal is considered a normal thats almost perpendicular to the viewing direction.
		  @param views The view point clouds
		  @param viewpoint The viewpoint origin
		  @param bad_normal_thr Threshold determining if a normal is bad. 90 degrees => normal perpendicular to viewpoint origin. if (angle between normal vector and viewpoint vector > 90 + bad_normal_thr) then the normal is considered to be bad. 
		  @return The vector containing all the normal-utilities for all views
		*/
		std::vector<float>
		compute_normal_utilities (std::vector<PointCloud_N::Ptr> views, PointT viewpoint, float bad_normal_thr);

	public:
	
		/**
		  Generates normal-utilities for the given rendered views of a model
		  @param views The view point clouds
		  @param viewpoint The viewpoint origin
		  @param bad_normal_thr Threshold determining if a normal is bad. 90 degrees => normal perpendicular to viewpoint origin. if (angle between normal vector and viewpoint vector > 90 + bad_normal_thr) then the normal is considered to be bad. 		  
		  @return The normal-utility vector
		*/
		std::vector<float>
		generate_normal_utilities (std::vector<PointCloud_N::Ptr> views, PointT viewpoint, float bad_normal_thr);
	
		/**
		  Saves the normal-utilities
		  @param normal_utilities The utility vector
		  @param model_name The name of the model
		*/
		void
		save_normal_utilities (std::string model_name, std::vector<float> normal_utilities);
	
};

#endif
