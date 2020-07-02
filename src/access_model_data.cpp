#include "access_model_data.h"

/**
  Useful commands:

	-path.string () returns a string of the path (e.g. /home/user/...)

	-path.filename () returns the name of the current file (e.g. a path with string "/home/user/pose_estimation" will
	 return "pose_estimation" when calling filename () )
*/

Access_Model_Data* Access_Model_Data::amd = nullptr;
std::mutex Access_Model_Data::lock_ac;
Access_Model_Data::GC Access_Model_Data::gc;
/**
  Returns the path to pose_estimation
*/
boost::filesystem::path 
Access_Model_Data::path_to_root (void)
{
	// Current path
	boost::filesystem::path p(boost::filesystem::current_path());
	
	// Search parent directories for pose_estimation
	while (true) 
	{
		p = p.parent_path();
		
		if (p.filename() == "pose_estimation")
		{
			break;
		}
	}
	
	return p;
}

/**
  Returns the path to Data in /pose_estimation
*/
boost::filesystem::path 
Access_Model_Data::path_to_data (void)
{
	// Add path to pose_estimation
	boost::filesystem::path p = path_to_root();
	
	// Add path to directory named "data"
	p /= "data";
	
	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		// Path does not exist, create directory
		if (!boost::filesystem::create_directory(p))
		{
			std::stringstream ss;
			ss << "ERROR: Could not create directory " << p << "!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
	}
	
	return p;
}

/**
  Returns the path to Model_data in /Data
*/
boost::filesystem::path 
Access_Model_Data::path_to_model_data (void)
{
	// Path to Data
//	boost::filesystem::path p = path_to_data ();
	boost::filesystem::path p(cr->model_data_path);

	// Add path to Model_data
//	p /= "model_data";
	
	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		// Path does not exist, create directory
		if (!boost::filesystem::create_directory(p))
		{
			std::stringstream ss;
			ss << "ERROR: Could not create directory " << p << "!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
	}
	
	return p;
}
boost::filesystem::path
Access_Model_Data::path_to_scene_data (void)
{
    // Path to Data
    boost::filesystem::path p(cr->scene_data_path);

    // Add path to Model_data
    p /= "mask_visib";

    // Check if such path exists
    if (!boost::filesystem::exists(p))
    {
        // Path does not exist, create directory
        if (!boost::filesystem::create_directory(p))
        {
            std::stringstream ss;
            ss << "ERROR: Could not create directory " << p << "!\n\n";
            pcl::console::print_error(ss.str().c_str());
            std::exit (EXIT_FAILURE);
        }
    }

    return p;
}
/**
  Returns the path to model in /Model_data
*/
boost::filesystem::path 
Access_Model_Data::path_to_model_in_model_data (std::string model)
{	
	// Add path to Model_data
//	boost::filesystem::path p = path_to_model_data ();
	boost::filesystem::path p(cr->model_data_path);

	// Add path to model
	p /= model;
	
	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		// Path does not exist, create directory for model
		if (!boost::filesystem::create_directory(p))
		{
			std::stringstream ss;
			ss << "ERROR: Could not create directory for " << model << " in model_data!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
	}
	
	return p;
}

/**
  Returns the path to CAD_models folder
*/
boost::filesystem::path
Access_Model_Data::path_to_cad_models ()
{
	// Add path to pose_estimation
//	boost::filesystem::path p = path_to_root ();
	boost::filesystem::path p(cr->cad_models_path);

	// Add path to CAD_models
//	p /= CAD_MODELS_PATH;

	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		// Path does not exist, create directory
		if (!boost::filesystem::create_directory(p))
		{
			std::stringstream ss;
			ss << "ERROR: Could not create directory " << p << "!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
	}
	
	return p;
}

/**
  Returns the path to model in /CAD_models
*/
std::string 
Access_Model_Data::path_to_model_in_CAD_models (std::string model)
{	
	// Add path to CAD_models
	boost::filesystem::path p = path_to_cad_models ();
	
	// Add path to model
	p /= model;
	
	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		std::stringstream ss;
		ss << "ERROR: Could not find " << model << " in CAD_models!\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE);
	}
	
	return p.string();
}

/**
  Saves the view-clouds 
  @param model_name The model name
  @param views_N Vector containing the view clouds 
  @param views_original_pose Vector containing the view clouds in original CAD-pose
  @param complete_model Complete model obtained by merging all the views in original pose
*/
void
Access_Model_Data::save_view_clouds (	std::string model_name,
					std::vector<PointCloud_N::Ptr> views_N, 
					std::vector<PointCloud_N::Ptr> views_original_pose, 
					PointCloudT::Ptr complete_model )
{
	// Add path to model
	boost::filesystem::path p = path_to_model_in_model_data (model_name);

	// Save point cloud of complete model
	std::stringstream ss;
	ss << p.string () << "/complete_model.pcd";
	pcl::io::savePCDFileBinaryCompressed (ss.str(), *complete_model);

	//
	// Save all views
	//

	// Add path to directory named "views"
	boost::filesystem::path p_views = p;
	p_views /= "views";

	// Check if such path exists
	if (!boost::filesystem::exists(p_views))
	{
		// Path does not exist, create directory for "views"
		if (!boost::filesystem::create_directory(p_views))
		{
			std::stringstream ss;
			ss << "ERROR: Could not create directory views for " << model_name << " in model_data!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
	}

	for (int i = 0; i < views_N.size(); i++)
	{
		ss.str ("");
		ss << "view" << i << ".pcd";
		boost::filesystem::path p_view = p_views;
		p_view /= ss.str ();

		pcl::io::savePCDFileASCII (p_view.string(), *views_N[i]);
	}

	// 
	// Save all views in original pose
	//

	// Add path to directory named "views_original_pose"
	boost::filesystem::path p_views_original_pose = p;
	p_views_original_pose /= "views_original_pose";

	// Check if such path exists
	if (!boost::filesystem::exists(p_views_original_pose))
	{
		// Path does not exist, create directory for "views_original_pose"
		if (!boost::filesystem::create_directory(p_views_original_pose))
		{
			std::stringstream ss;
			ss << "ERROR: Could not create directory views_original_pose for " << model_name << " in model_data!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
	}

	for (int i = 0; i < views_original_pose.size(); i++)
	{
		ss.str ("");
		ss << "view" << i << ".pcd";
		boost::filesystem::path p_view = p_views_original_pose;
		p_view /= ss.str ();
		pcl::io::savePCDFileASCII (p_view.string(), *views_original_pose[i]);
	}
}
/**
  Saves the fusion view-clouds
  @param model_name The model name
  @param views_N Vector containing the view clouds
  @param views_original_pose Vector containing the view clouds in original CAD-pose
  @param complete_model Complete model obtained by merging all the views in original pose
*/
void
Access_Model_Data::save_fusion_view_clouds (	std::string model_name,
                                                std::vector<std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>> &fuse_results)
{
    // Add path to model
    boost::filesystem::path p = path_to_model_in_model_data (model_name);



    // Add path to directory named "fusion_views"
    boost::filesystem::path p_fusion_views = p;
    p_fusion_views /= "fusion_views";

    // Check if such path exists
    if (!boost::filesystem::exists(p_fusion_views))
    {
        // Path does not exist, create directory for "fusion_views"
        if (!boost::filesystem::create_directory(p_fusion_views))
        {
            std::stringstream ss;
            ss << "ERROR: Could not create directory fusion_views for " << model_name << " in model_data!\n\n";
            pcl::console::print_error(ss.str().c_str());
            std::exit (EXIT_FAILURE);
        }
    }

    for (int i = 0; i < fuse_results.size(); i++)
    {
        boost::filesystem::path p_fusion_view_n = p_fusion_views;
        std::string viewi = "view" + std::to_string(i);
        p_fusion_view_n /= viewi;
        // Check if such path exists
        if (!boost::filesystem::exists(p_fusion_view_n))
        {
            // Path does not exist, create directory for "fusion_views"
            if (!boost::filesystem::create_directory(p_fusion_view_n))
            {
                std::stringstream ss;
                ss << "ERROR: Could not create directory viewi for " << model_name << " in model_data!\n\n";
                pcl::console::print_error(ss.str().c_str());
                std::exit (EXIT_FAILURE);
            }
        }
        for (int j = 0; j < fuse_results[i].size(); ++j) {
            std::stringstream ss;
            ss << "level" << j+1 << ".pcd";
            boost::filesystem::path p_view_level = p_fusion_view_n;
            p_view_level /= ss.str ();
            pcl::io::savePCDFileASCII (p_view_level.string(), *fuse_results[i][j]);
        }
    }
}

/**
  Saves the view-utilities for each view. The utility is a measure of the visibility of the object in each view.   
  @param model_name The model name
  @param utilities Vector containing the view-utilities 
*/
void 
Access_Model_Data::save_view_utilities (std::string model_name, std::vector<float> utilities)
{
	// Add path to model
	boost::filesystem::path p = path_to_model_in_model_data (model_name);
	
	std::ofstream utilities_file;
	p /= "view_utilities.csv";
	utilities_file.open (p.string().c_str());
	
	std::vector<std::string> string_vec;
	for (int i = 0; i < utilities.size(); i++)
	{
		std::ostringstream ss;
		ss << i << ",";
		ss << utilities[i] << ",";
		string_vec.push_back (ss.str ());
	}
	
	for (int i = 0; i < string_vec.size(); i++)
	{
		utilities_file << string_vec[i] << "\n";
	}
	
	utilities_file.close();
}

/**
  Saves the global feature for each view.    
  @param model_name The model name
  @param global_features Point cloud containing the global features 
*/
void 
Access_Model_Data::save_global_features (std::string model_name, FeatureCloudG::Ptr global_features)
{
	// Add path to model
	boost::filesystem::path p = path_to_model_in_model_data (model_name);
	
	// Save global feature point cloud
	p /= "global_features.pcd";
	pcl::io::savePCDFileBinaryCompressed (p.string(), *global_features);
}

/**
  Adds the view-clouds in /views to (@param views)
  @param model_name The model name
  @param views Empty vector. The view-clouds will be loaded and stored in views 
  @param indices Vector containing the view indices to be loaded. If empty then all views will be loaded
*/
void 
Access_Model_Data::load_model_views (std::string model_name, std::vector<PointCloud_N::Ptr> &views, std::vector<int> indices)
{
	// Add path to directory named "Model_data"
	boost::filesystem::path p = path_to_model_data ();
	
	// Add path to directory named model_name
	p /= model_name;
	
	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		std::stringstream ss;
		ss << "ERROR: No directory found for " << model_name << " in model_data!\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE);
	}
	
	// Add path to views
	p /= "views";
	
	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		std::stringstream ss;
		ss << "ERROR: No directory found named views for " << model_name << " in model_data!\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE);
	}
	
	if (indices.size() == 0)
	{
		// Load all views
		std::vector<boost::filesystem::path> path_vec;
		copy(boost::filesystem::directory_iterator(p), boost::filesystem::directory_iterator(), back_inserter(path_vec)); // Get number of files in /views
		for (int i = 0; i < path_vec.size(); i++)
		{
			indices.push_back (i);
		}
	}
	
	for (int i = 0; i < indices.size(); i++)
	{
		// Load views
		PointCloud_N::Ptr view (new PointCloud_N);
		std::stringstream ss;
		ss << p.string () << "/view" << indices[i] << ".pcd";
		if ( pcl::io::loadPCDFile <Point_N> (ss.str (), *view) < 0 )
		{
			std::stringstream ss;
			ss << "ERROR: Could not load view" << indices[i] << " in " << model_name << "/views!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
		views.push_back (view);
	}
}
/**
  Adds the fusion view-clouds in /views to (@param views)
  @param model_name The model name
  @param views Empty vector. The view-clouds will be loaded and stored in views
  @param indices Vector containing the view indices to be loaded. If empty then all views will be loaded
  @param level the level need to load
*/
void
Access_Model_Data::load_fusion_model_views (std::string model_name, std::vector<PointCloud_N::Ptr> &views, std::vector<int> indices, int level)
{
    // Add path to directory named "Model_data"
    boost::filesystem::path p = path_to_model_data ();

    // Add path to directory named model_name
    p /= model_name;

    // Check if such path exists
    if (!boost::filesystem::exists(p))
    {
        std::stringstream ss;
        ss << "ERROR: No directory :"<<p.string()<<"\n\n";
        pcl::console::print_error(ss.str().c_str());
        std::exit (EXIT_FAILURE);
    }

    // Add path to views
    p /= "fusion_views";

    // Check if such path exists
    if (!boost::filesystem::exists(p))
    {
        std::stringstream ss;
        ss << "ERROR: No directory found named views for " << model_name << " in model_data!\n\n";
        pcl::console::print_error(ss.str().c_str());
        std::exit (EXIT_FAILURE);
    }

    if (indices.size() == 0)
    {
        // Load all views
        std::vector<boost::filesystem::path> path_vec;
        copy(boost::filesystem::directory_iterator(p), boost::filesystem::directory_iterator(), back_inserter(path_vec)); // Get number of files in /views
        for (int i = 0; i < path_vec.size(); i++)
        {
            indices.push_back (i);
        }
    }

    for (int i = 0; i < indices.size(); i++)
    {
        // Load views
        PointCloud_N::Ptr view (new PointCloud_N);
        std::string viewi = "view" + std::to_string(i);
        boost::filesystem::path p_ = p;
        p_ /= viewi;
        std::stringstream ss;
        ss << p_.string () << "/level" << level << ".pcd";
        if ( pcl::io::loadPCDFile <Point_N> (ss.str (), *view) < 0 )
        {
            std::stringstream ss;
            ss << "ERROR: Could not load fusion view" << indices[i] << " in " << model_name << "/fusion_views!\n\n";
            pcl::console::print_error(ss.str().c_str());
            std::exit (EXIT_FAILURE);
        }
        views.push_back (view);
    }
}
/**
  Adds the view-clouds in /views_original_pose to (@param views)
  @param model_name The model name
  @param views Empty vector. The view-clouds will be loaded and stored in views 
  @param indices Vector containing the view indices to be loaded. If empty then all views will be loaded
*/
void 
Access_Model_Data::load_model_views_original_pose (std::string model_name, std::vector<PointCloud_N::Ptr> &views, std::vector<int> indices)
{
	// Add path to directory named "Model_data"
	boost::filesystem::path p = path_to_model_data ();
	
	// Add path to directory named model_name
	p /= model_name;
	
	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		std::stringstream ss;
        ss << "ERROR: No directory :"<<p.string()<<"\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE);
	}
	
	// Add path to views_original_pose
	p /= "views_original_pose";
	
	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		std::stringstream ss;
		ss << "ERROR: No directory found named views_original_pose for " << model_name << " in model_data!\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE);
	}
	
	if (indices.size() == 0)
	{
		// Load all views
		std::vector<boost::filesystem::path> path_vec;
		copy(boost::filesystem::directory_iterator(p), boost::filesystem::directory_iterator(), back_inserter(path_vec)); // Get number of files in /views
		for (int i = 0; i < path_vec.size(); i++)
		{
			indices.push_back (i);
		}
	}
	
	for (int i = 0; i < indices.size(); i++)
	{
		// Load views
		PointCloud_N::Ptr view (new PointCloud_N);
		std::stringstream ss;
		ss << p.string () << "/view" << indices[i] << ".pcd";
		if ( pcl::io::loadPCDFile <Point_N> (ss.str (), *view) < 0 )
		{
			std::stringstream ss;
			ss << "ERROR: Could not load view" << indices[i] << " in " << model_name << "/views_original_pose!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
		views.push_back (view);
	}
}

/**
  Returns a vector of all models in Model_data
*/
std::vector<std::string> 
Access_Model_Data::get_model_names (void)
{
	// Add path to Model_data
	boost::filesystem::path p = path_to_model_data ();
	
	std::vector<std::string> vec;
    boost::filesystem::directory_iterator end_iter;
	for (boost::filesystem::directory_iterator dir_itr(p); dir_itr != end_iter; ++dir_itr)
	{
		if (boost::filesystem::is_directory(dir_itr->status()))
		{
			vec.push_back (dir_itr->path().filename().string());
		}
	}
	
	return vec;
}

/**
  Loads the global features for a given model. The global features are ESF features!
  @param model The name of the model
  @param models Empty point cloud. Adds the global features in model 
*/
void 
Access_Model_Data::load_global_features (std::string model, FeatureCloudG::Ptr features)
{
	// Add path to model in Model_data
	boost::filesystem::path p = path_to_model_in_model_data (model);
	
	std::stringstream ss;
	ss << p.string () << "/global_features.pcd";
	
	// Load global features for model 
	if ( pcl::io::loadPCDFile <FeatureG> (ss.str (), *features) < 0 )
	{
		std::stringstream ss;
		ss << "ERROR: Could not load global_features for " << model << " in /model_data!\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE);
	}
}

/**
  Loads the complete/merged point cloud model from the rendered views. Note that the complete model has some offset when merging the rendered views   
  @param model The name of the model
  @param models Empty point cloud. Adds the complete point cloud model 
*/
void 
Access_Model_Data::load_complete_model (std::string model, PointCloudT::Ptr cloud)
{
	// Add path to model in Model_data
	boost::filesystem::path p = path_to_model_in_model_data (model);
	
	std::stringstream ss;
	ss << p.string () << "/complete_model.pcd";
	if ( pcl::io::loadPCDFile <PointT> (ss.str (), *cloud) < 0 )
	{
		std::stringstream ss;
		ss << "ERROR: Could not load complete_model for " << model << " in /model_data!\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE);
	}
}

/**
  Saves the similar view data   
  @param source_name The name of the model
  @param target_name The name of the target (matching) model
  @param similar_views Vector containing the matching scores 
*/
void 
Access_Model_Data::save_similar_model_data (std::string source_name, std::string target_name, std::vector<float> similar_views)
{
	// Get path to source in Model_data
	boost::filesystem::path p_source = path_to_model_in_model_data (source_name);
	
	// Add path to distinguish_utilities
	p_source /= "distinguish_utilities";
	
	// Check if such path exists
	if (!boost::filesystem::exists(p_source))
	{
		// Path does not exist, create directory for "distinguish_utilities"
		if (!boost::filesystem::create_directory(p_source))
		{
			std::stringstream ss;
			ss << "ERROR: Could not create directory for distinguish_utilities in " << source_name << "!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
	}
	
	std::stringstream ss;
	ss << p_source.string () << "/" << target_name << "_distinguish_utilities.csv";
	
	std::ofstream similar_views_file;
	similar_views_file.open (ss.str().c_str());
	
	if (!similar_views_file.is_open ())
	{
		std::stringstream ss;
		ss << "ERROR: Could not open file " << ss.str () << "!\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE);
	}

	std::vector<std::string> string_vec;
	for (int i = 0; i < similar_views.size(); i++)
	{
		std::ostringstream ss;
		ss << i << ",";
		ss << similar_views[i] << ",";
		
		string_vec.push_back (ss.str ());
	}

	for (int i = 0; i < string_vec.size(); i++)
	{
		similar_views_file << string_vec[i] << "\n";
	}

	similar_views_file.close();
}
	
/**
  Saves the feature utilities   
  @param utilities The feature utilities 
*/
void
Access_Model_Data::save_feature_utilities (std::string model_name, std::vector<double> utilities)
{
	// Get path to model_name in Model_data
	boost::filesystem::path p = path_to_model_in_model_data (model_name);
	
	// Add path to feature_utilities.csv
	p /= "feature_utilities.csv";
	
	std::ofstream feature_utilities_file;
	feature_utilities_file.open (p.string().c_str());
	
	if (!feature_utilities_file.is_open ())
	{
		std::stringstream ss;
		ss << "ERROR: Could not open file " << p.string () << "!\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE);
	}

	std::vector<std::string> string_vec;
	for (int i = 0; i < utilities.size(); i++)
	{
		std::ostringstream ss;
		ss << i << ",";
		ss << utilities[i] << ",";
		
		string_vec.push_back (ss.str ());
	}

	for (int i = 0; i < string_vec.size(); i++)
	{
		feature_utilities_file << string_vec[i] << "\n";
	}

	feature_utilities_file.close();
}

/**
Loads the view utilities
@param model_name the name of the model
@return a vector containing all view-utilities
*/
std::vector<float>
Access_Model_Data::load_view_utilities (std::string model_name)
{
	// Add path to model in Model_data
	boost::filesystem::path p = path_to_model_in_model_data (model_name);
	
	// Add path to CVS file
	p /= "view_utilities.csv";
	
	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		std::stringstream ss;
		ss << "ERROR: No file found named view_utilities!\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE);
	}
	
	std::ifstream csv_file (p.string().c_str());

	std::string line;
	std::vector<float> utilities;
	while (std::getline (csv_file, line))
	{
		std::stringstream buffer (line);
		std::string value;
		std::vector<std::string> data;

		while(std::getline(buffer, value, ','))
		{
		   data.push_back(value);
		}

		utilities.push_back (std::strtof (data[1].c_str(), NULL));		
	}
	
	return utilities;
}

/**
Loads the feature utilities
@param model_name the name of the model
@return a vector containing all feature-utilities
*/
std::vector<float>
Access_Model_Data::load_feature_utilities (std::string model_name)
{
	// Add path to model in Model_data
	boost::filesystem::path p = path_to_model_in_model_data (model_name);
	
	// Add path to CVS file
	p /= "feature_utilities.csv";
	
	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		std::stringstream ss;
		ss << "ERROR: No file found named feature_utilities!\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE);
	}
	
	std::ifstream csv_file (p.string().c_str());

	std::string line;
	std::vector<float> utilities;
	while (std::getline (csv_file, line))
	{
		std::stringstream buffer (line);
		std::string value;
		std::vector<std::string> data;

		while(std::getline(buffer, value, ','))
		{
		   data.push_back(value);
		}

		utilities.push_back (std::strtof (data[1].c_str(), NULL));		
	}
	
	return utilities;
}

/**
  Loads the distinguish-utility
  @param model_name the name of the model
  @param similar_model_name the name of the similar model
  @return a vector containing the distinguish utilities for all similar models
*/
std::vector<std::vector<float> >
Access_Model_Data::load_distinguish_utilities (std::string model_name, std::vector<std::string> similar_models)
{
	// Add path to model in Model_data
	boost::filesystem::path p = path_to_model_in_model_data (model_name);
	
	// Add path to distinguish_utilities
	p/= "distinguish_utilities";
	
	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		std::stringstream ss;
		ss << "ERROR: No directory found named distinguish_utilities!\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE);
	}
	
	std::vector<std::vector<float> > utilities;
	for (int i = 0; i < similar_models.size(); i++)
	{
		if (similar_models[i] == model_name)
		{
			continue;
		}
		
		// Add path to CVS file
		boost::filesystem::path p_sv = p;
		p_sv /= similar_models[i] + "_distinguish_utilities.csv";
	
		// Check if such path exists
		if (!boost::filesystem::exists(p_sv))
		{
			std::stringstream ss;
			ss << "ERROR: No file found named " << similar_models[i] << "_distinguish_utilities.csv for " << model_name << "!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
	
		std::ifstream csv_file (p_sv.string().c_str());

		std::string line;
		std::vector<float> vec;
		while (std::getline (csv_file, line))
		{
			std::stringstream buffer (line);
			std::string value;
			std::vector<std::string> data;

			while(std::getline(buffer, value, ','))
			{
			   data.push_back(value);
			}

			vec.push_back (std::strtof (data[1].c_str(), NULL));		
		}
		utilities.push_back (vec);
	}
	
	return utilities;
}

/**
Loads the normal-utilities
@param model_name the name of the model
@return a vector containing all normal-utilities
*/
std::vector<float>
Access_Model_Data::load_normal_utilities (std::string model_name)
{
	// Add path to model in Model_data
	boost::filesystem::path p = path_to_model_in_model_data (model_name);
	
	// Add path to CVS file
	p /= "normal_utilities.csv";
	
	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		std::stringstream ss;
		ss << "ERROR: No file found named normal_utilities!\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE);
	}
	
	std::ifstream csv_file (p.string().c_str());

	std::string line;
	std::vector<float> utilities;
	while (std::getline (csv_file, line))
	{
		std::stringstream buffer (line);
		std::string value;
		std::vector<std::string> data;

		while(std::getline(buffer, value, ','))
		{
		   data.push_back(value);
		}

		utilities.push_back (std::strtof (data[1].c_str(), NULL));		
	}
	
	return utilities;
}

/**
  Loads the views in indices and merges the result
  @param model_name The name of the model
  @param indices The indices of the views
  @param merged_cloud The merged cloud
*/
void 
Access_Model_Data::load_merged_views (std::string model_name, std::vector<int> indices, PointCloud_N::Ptr merged_cloud)
{
	// Load views
	std::vector<PointCloud_N::Ptr> views;
	load_model_views_original_pose (model_name, views, indices);

	// Merge views
	for (int i = 0; i < views.size(); i++)
	{
		*merged_cloud += *(views[i]);
	}

	// Downsample
	float leaf_size = 0.003f;
	pcl::VoxelGrid<Point_N> sor;
	sor.setInputCloud (merged_cloud);
	sor.setLeafSize (leaf_size, leaf_size, leaf_size);
	sor.filter (*merged_cloud);
}

/**
  Returns a merged point cloud of all the point clouds in views 
  @param views Vector containing all the point clouds
  @param merged_cloud The merged cloud
*/
void
Access_Model_Data::merge_views (std::vector<PointCloud_N::Ptr> views, PointCloud_N::Ptr merged_cloud)
{
	if (views.size() < 2)
	{
		std::stringstream ss;
		ss << "ERROR: The input vector must contain at least two point clouds!\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE);
	}
	
	// Merge views
	for (int i = 0; i < views.size(); i++)
	{
		*merged_cloud += *(views[i]);
	}

	// Downsample
	float leaf_size = 0.003f;
	pcl::VoxelGrid<Point_N> sor;
	sor.setInputCloud (merged_cloud);
	sor.setLeafSize (leaf_size, leaf_size, leaf_size);
	sor.filter (*merged_cloud);
}

/**
  Saves the local features for all views 
  @param model_name The model name
  @param features The local feature clouds
*/
void 
Access_Model_Data::save_local_features (std::string model_name, std::vector<FeatureCloudL::Ptr> features)
{
	// Add path to model
	boost::filesystem::path p = path_to_model_in_model_data (model_name);

	// Add path to directory named "local_features"
	boost::filesystem::path p_lf = p;
	p_lf /= "local_features";

	// Check if such path exists
	if (!boost::filesystem::exists(p_lf))
	{
		// Path does not exist, create directory for "local_features"
		if (!boost::filesystem::create_directory(p_lf))
		{
			std::stringstream ss;
			ss << "ERROR: Could not create directory local features for " << model_name << " in model_data!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
	}

	std::stringstream ss;
	for (int i = 0; i < features.size(); i++)
	{
		ss.str ("");
		ss << p_lf.string () << "/view" << i << ".pcd";
		pcl::io::savePCDFileBinaryCompressed (ss.str (), *features[i]);
	}
}
/**
  Saves the fusion local features for all views
  @param model_name The model name
  @param features The local feature clouds
*/
void
Access_Model_Data::save_fusion_local_features (std::string model_name, std::vector<std::vector<FeatureCloudL::Ptr>> features)
{
    // Add path to model
    boost::filesystem::path p = path_to_model_in_model_data (model_name);

    // Add path to directory named "fusion_local_features"
    boost::filesystem::path p_lf = p;
    p_lf /= "fusion_local_features";

    // Check if such path exists
    if (!boost::filesystem::exists(p_lf))
    {
        // Path does not exist, create directory for "fusion_local_features"
        if (!boost::filesystem::create_directory(p_lf))
        {
            std::stringstream ss;
            ss << "ERROR: Could not create directory local features for " << model_name << " in model_data!\n\n";
            pcl::console::print_error(ss.str().c_str());
            std::exit (EXIT_FAILURE);
        }
    }

    std::stringstream ss;
    for (int i = 0; i < features.size(); i++)
    {
        std::string feature_directory = "view" + std::to_string(i);
        p_lf /= feature_directory;
        // Check if such path exists
        if (!boost::filesystem::exists(p_lf))
        {
            // Path does not exist, create directory for "fusion_local_features"
            if (!boost::filesystem::create_directory(p_lf))
            {
                std::stringstream ss;
                ss << "ERROR: Could not create directory local features for " << model_name << " in fusion_local_features!\n\n";
                pcl::console::print_error(ss.str().c_str());
                std::exit (EXIT_FAILURE);
            }
        }
        ss.str ("");
        for (int j = 0; j < features[i].size(); ++j) {
            ss << p_lf.string () << "/level" << j+1 << ".pcd";
            pcl::io::savePCDFileBinaryCompressed (ss.str (), *(features[i][j]));
        }

    }
}
/**
  Adds the local features for the view-clouds in /views_original_pose 
  @param model_name The model name
  @param local_features Empty vector. The local features for the view-clouds will be loaded and stored in local_features 
  @param indices Vector containing the view indices to be loaded. If empty then all views will be loaded
*/
void 
Access_Model_Data::load_local_features (std::string model_name, std::vector<FeatureCloudL::Ptr> &local_features, std::vector<int> indices)
{
	// Add path to directory named "Model_data"
	boost::filesystem::path p = path_to_model_in_model_data (model_name);
	
	// Add path to local_features
	p /= "local_features";
	
	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		std::stringstream ss;
		ss << "ERROR: No directory found named local_features for " << model_name << " in model_data!\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE);
	}
	
	if (indices.size() == 0)
	{
		// Load all views
		std::vector<boost::filesystem::path> path_vec;
		copy(boost::filesystem::directory_iterator(p), boost::filesystem::directory_iterator(), back_inserter(path_vec)); // Get number of files in /local_features
		for (int i = 0; i < path_vec.size(); i++)
		{
			indices.push_back (i);
		}
	}
	
	for (int i = 0; i < indices.size(); i++)
	{
		// Load local features
		FeatureCloudL::Ptr features (new FeatureCloudL);
		std::stringstream ss;
		ss << p.string () << "/view" << indices[i] << ".pcd";
		if ( pcl::io::loadPCDFile <FeatureL> (ss.str (), *features) < 0 )
		{
			std::stringstream ss;
			ss << "ERROR: Could not load local features for view" << indices[i] << " in " << model_name << "/local_features!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
		local_features.push_back (features);
	}
}










