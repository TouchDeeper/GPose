
#include "add_model/normal_utility.h"

/**
  Computes the normal utilities. A bad normal is considered a normal thats almost perpendicular to the viewing direction.
  @param views The view point clouds
  @param viewpoint The viewpoint origin
  @param bad_normal_thr Threshold determining if a normal is bad. 90 degrees => normal perpendicular to viewpoint origin. if (angle between normal vector and viewpoint vector > 90 + bad_normal_thr) then the normal is considered to be bad. 
  @return The vector containing all the normal-utilities for all views
*/
std::vector<float>
Normal_Utility::compute_normal_utilities (std::vector<PointCloud_N::Ptr> views, PointT viewpoint, float bad_normal_thr)
{
	std::vector<float> normal_utilities;
	
	float pi_const = atan(1)*4;
	for (int i = 0; i < views.size(); i++)
	{
		int nbr_of_bad_normals = 0;
		for (int j = 0; j < views[i]->points.size(); j++)
		{		
			// Compute the angle between the normal and the viewdirection 
			Eigen::Vector3f n;
			n << views[i]->points[j].normal_x, 
		   		 views[i]->points[j].normal_y, 
		   		 views[i]->points[j].normal_z;
			Eigen::Vector3f v_p;
			v_p << views[i]->points[j].x - viewpoint.x,
				   views[i]->points[j].y - viewpoint.y,
				   views[i]->points[j].z - viewpoint.z;
			n.normalize ();
			v_p.normalize ();
			float angle = acos (n.dot(v_p)) * (180 / pi_const);
			if (angle < (90 + bad_normal_thr))
			{
				nbr_of_bad_normals++;
			}
		}
		
		// Compute the fraction of bad normals for the whole point cloud
		float frac_bad_normals = 1 - nbr_of_bad_normals / (float)views[i]->points.size();
		normal_utilities.push_back (frac_bad_normals);
	}
	
	//
	// Normalize utilities
	//
	
	std::vector<float> temp = normal_utilities;
	std::sort(temp.begin(), temp.end());
	
	float max_value = temp.back();
	float min_value = temp.front();
	for (int i = 0; i < normal_utilities.size(); i++)
	{
		normal_utilities[i] = (normal_utilities[i] - min_value) / (max_value - min_value);
	}
	
	return normal_utilities;
}

/**
  Generates normal-utilities for the given rendered views of a model
  @param views The view point clouds
  @param viewpoint The viewpoint origin
  @param bad_normal_thr Threshold determining if a normal is bad. 90 degrees => normal perpendicular to viewpoint origin. if (angle between normal vector and viewpoint vector > 90 + bad_normal_thr) then the normal is considered to be bad. 
  @return The normal-utility vector
*/
std::vector<float>
Normal_Utility::generate_normal_utilities (std::vector<PointCloud_N::Ptr> views, PointT viewpoint, float bad_normal_thr)
{	
	std::vector<float> normal_utilities;
	
	// Compute normal utility for each view according to the number of bad normals for each view
	normal_utilities = compute_normal_utilities (views, viewpoint, bad_normal_thr);
	
	return normal_utilities;
}

/**
  Saves the normal-utilities
  @param normal_utilities The utility vector
  @param model_name The name of the model
*/
void
Normal_Utility::save_normal_utilities (std::string model_name, std::vector<float> normal_utilities)
{
	Access_Model_Data* amd = Access_Model_Data::GetInstance();
	boost::filesystem::path p = amd->path_to_model_in_model_data (model_name);
	p /= "normal_utilities.csv";
	
	std::ofstream utilities_file;
	utilities_file.open (p.string().c_str());
	
	std::vector<std::string> string_vec;
	for (int i = 0; i < normal_utilities.size(); i++)
	{
		std::ostringstream ss;
		ss << i << ",";
		ss << normal_utilities[i] << ",";
		string_vec.push_back (ss.str ());
	}
	
	for (int i = 0; i < string_vec.size(); i++)
	{
		utilities_file << string_vec[i] << "\n";
	}
	
	utilities_file.close();
	
}
















