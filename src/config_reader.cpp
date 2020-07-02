#include "config_reader.h"

namespace pt = boost::property_tree;
Config_Reader *Config_Reader::cr = nullptr;
std::mutex Config_Reader::get_object_;
Config_Reader::GC Config_Reader::gc;
/**
	Loads all parameters associated with adding a new model to the system (Rendering views)
	@param filename The name of the config file (.ini)
 */
void
Config_Reader::add_model_load_config (const std::string &filename)
{
	// Create empty property tree object
	pt::ptree tree;

	// Parse the INI into the property tree.
	pt::read_ini (filename, tree);

	// Read all config parameters associated with adding a new model 
	resolution = tree.get<int> ("Add_Model-Default.resolution");
	tessellation_level = tree.get<int> ("Add_Model-Default.tessellation_level"); 
	radius_tessellated_sphere = tree.get<float> ("Add_Model-Default.radius_tessellated_sphere");
	if (tree.get<std::string> ("Add_Model-Default.view_angle") == "optimal")
	{
		optimal_view_angle = true;
	}
	else
	{
		view_angle = tree.get<float> ("Add_Model-Default.view_angle");
		optimal_view_angle = false;
	}

	scale_factor = tree.get<double> ("Add_Model-Default.scale_factor");
	model_name = tree.get<std::string> ("Add_Model-Default.model_name");
	scene_name = tree.get<std::string> ("Add_Model-Default.scene_name");
    base_dir = tree.get<std::string> ("Add_Model-Default.base_dir");

	leaf_size = tree.get<float> ("Add_Model-Advanced.leaf_size");
	cluster_tolerance = tree.get<double> ("Add_Model-Advanced.cluster_tolerance");
	min_cluster_size = tree.get<int> ("Add_Model-Advanced.min_cluster_size");
	max_cluster_size = tree.get<int> ("Add_Model-Advanced.max_cluster_size");
	search_radius_mls = tree.get<double> ("Add_Model-Advanced.search_radius_mls");
	mean_k = tree.get<int> ("Add_Model-Advanced.mean_k");
	std_dev_mul_thresh = tree.get<float> ("Add_Model-Advanced.std_dev_mul_thresh");
	bad_normals_threshold = tree.get<float> ("Add_Model-Advanced.bad_normals_threshold"); 
	use_k_search = tree.get<bool> ("Add_Model-Advanced.use_k_search");
	k_search_normals = tree.get<int> ("Add_Model-Advanced.k_search_normals");
	use_radius_search = tree.get<bool> ("Add_Model-Advanced.use_radius_search");
	radius_search_normals = tree.get<double> ("Add_Model-Advanced.radius_search_normals");
	largest_cluster_extraction = tree.get<bool> ("Add_Model-Advanced.largest_cluster_extraction");
	downsample = tree.get<bool> ("Add_Model-Advanced.downsample");
	smooth = tree.get<bool> ("Add_Model-Advanced.smooth");
	remove_outliers = tree.get<bool> ("Add_Model-Advanced.remove_outliers");
	view_processed_clouds = tree.get<bool> ("Add_Model-Advanced.view_processed_clouds");
	view_normals = tree.get<bool> ("Add_Model-Advanced.view_normals");
	normal_magnitude = tree.get<float> ("Add_Model-Advanced.normal_magnitude");
	view_complete_model = tree.get<bool> ("Add_Model-Advanced.view_complete_model");
	view_graph = tree.get<bool> ("Add_Model-Advanced.view_graph");
	avg_glb_feature = tree.get<bool> ("Add_Model-Advanced.avg_glb_feature");
	use_delay = tree.get<bool> ("Add_Model-Advanced.use_delay");
    fusion_level = tree.get<int> ("Add_Model-Default.fusion_level");
}
/**
  Loads all parameters associated with visualization
  @param filename The name of the config file (.ini)
*/
void
Config_Reader::add_visualization_config (const std::string &filename)
{
    // Create empty property tree object
    pt::ptree tree;

    // Parse the INI into the property tree.
    pt::read_ini (filename, tree);

    normal_level = tree.get<int> ("Visualization.normal_level");
    point_size = tree.get<double> ("Visualization.point_size");
}

/**
  Loads all parameters associated with descriptors
  @param filename The name of the config file (.ini)
*/
void
Config_Reader::add_descriptors_config (const std::string &filename)
{
	// Create empty property tree object
	pt::ptree tree;

	// Parse the INI into the property tree.
	pt::read_ini (filename, tree);

	local_descriptors_type_ = tree.get<std::string> ("Descriptors.local_descriptors_type");
	global_descriptors_type_ = tree.get<std::string> ("Descriptors.global_descriptors_type");
}
/**
	Loads all parameters associated with the system
	@param filename The name of the config file (.ini)
 */
void
Config_Reader::system_load_config (const std::string &filename)
{
	// Create empty property tree object
	pt::ptree tree;

	// Parse the INI into the property tree.
	pt::read_ini (filename, tree);

	// Read all config parameters associated with the system
	save_path = tree.get<std::string> ("System-Default.save_path");

	hint_view_weight = tree.get<float> ("System-Default.hint_view_weight");
	hint_normal_weight = tree.get<float> ("System-Default.hint_normal_weight");
	hint_feature_weight = tree.get<float> ("System-Default.hint_feature_weight");
	hint_distinguish_weight = tree.get<float> ("System-Default.hint_distinguish_weight");
	hint_below_plane_threshold = tree.get<double> ("System-Advanced.hint_below_plane_threshold");
	hint_misalignment_angle_threshold = tree.get<int> ("System-Advanced.hint_misalignment_angle_threshold");
	hint_view_search_results = tree.get<bool> ("System-Advanced.hint_view_search_results");
	hint_normalize_search_results = tree.get<bool> ("System-Advanced.hint_normalize_search_results");
	hint_print_info = tree.get<bool> ("System-Advanced.hint_print_info");

	rdf_enabled = tree.get<bool> ("System-Default.rdf_enabled");
	rdf_url = tree.get<std::string> ("System-Default.rdf_url");
	rdf_repo_name = tree.get<std::string> ("System-Default.rdf_repo_name");

	use_centered_cluster = tree.get<bool> ("System-Default.use_centered_cluster");
	centered_threshold = tree.get<double> ("System-Default.centered_threshold");

	max_id_views = tree.get<int> ("System-Default.max_id_views");
	max_pose_views = tree.get<int> ("System-Default.max_pose_views");
	max_merged_views = tree.get<int> ("System-Default.max_merged_views");

	visualize_merged_views = tree.get<bool> ("System-Advanced.visualize_merged_views");

	web_service_username = tree.get<std::string> ("System-Default.web_service_username");
	web_service_password = tree.get<std::string> ("System-Default.web_service_password");
	hand_web_service_url = tree.get<std::string> ("System-Default.hand_web_service_url");

	// Pose
	pose_max_iterations = tree.get<int> ("System-Advanced.pose_max_iterations");
	pose_max_correspondence_distance = tree.get<double> ("System-Advanced.pose_max_correspondence_distance");
	pose_correspondence_randomness = tree.get<double> ("System-Advanced.pose_correspondence_randomness");
	pose_similarity_threshold = tree.get<double> ("System-Advanced.pose_similarity_threshold");
	pose_inlier_fraction = tree.get<double> ("System-Advanced.pose_inlier_fraction");
	pose_inverse_inlier_fraction = tree.get<double> ("System-Advanced.pose_inverse_inlier_fraction");
	pose_feature_radius_search = tree.get<double> ("System-Advanced.pose_feature_radius_search");
	pose_visualization_mode = tree.get<int> ("System-Advanced.pose_visualization_mode");
	pose_print_mode = tree.get<int> ("System-Advanced.pose_print_mode");

	// Segmentation
	segmentation_leaf_size = tree.get<double> ("System-Advanced.segmentation_leaf_size");
	segmentation_plane_distance_threshold = tree.get<double> ("System-Advanced.segmentation_plane_distance_threshold");
	segmentation_plane_max_iterations = tree.get<int> ("System-Advanced.segmentation_plane_max_iterations");
	segmentation_background_segmentation_distance = tree.get<double> ("System-Advanced.segmentation_background_segmentation_distance");
	segmentation_normal_radius_search = tree.get<double> ("System-Advanced.segmentation_normal_radius_search");
	segmentation_cluster_tolerance = tree.get<double> ("System-Advanced.segmentation_cluster_tolerance");
	segmentation_cluster_min_size = tree.get<int> ("System-Advanced.segmentation_cluster_min_size");
	segmentation_cluster_max_size = tree.get<int> ("System-Advanced.segmentation_cluster_max_size");
}

/**
	Loads all parameters associated with the librealsense_capturer
	@param filename The name of the config file (.ini)
 */
void
Config_Reader::capturer_load_config (const std::string &filename)
{
	// Create empty property tree object
	pt::ptree tree;

	// Parse the INI into the property tree.
	pt::read_ini (filename, tree);

	// Read all config parameters
	save_path = tree.get<std::string> ("System-Default.save_path");
}

/**
	Loads all parameters associated with the background_creator
	@param filename The name of the config file (.ini)
 */
void
Config_Reader::background_load_config (const std::string &filename)
{
	// Create empty property tree object
	pt::ptree tree;

	// Parse the INI into the property tree.
	pt::read_ini (filename, tree);

	// Read all config parameters
	save_path = tree.get<std::string> ("System-Default.save_path");
	web_service_username = tree.get<std::string> ("System-Default.web_service_username");
	web_service_password = tree.get<std::string> ("System-Default.web_service_password");
	hand_web_service_url = tree.get<std::string> ("System-Default.hand_web_service_url");
}
void
Config_Reader::add_verbose_config (const std::string &filename)
{
    // Create empty property tree object
    pt::ptree tree;

    // Parse the INI into the property tree.
    pt::read_ini (filename, tree);

    // Read all config parameters
    visual = tree.get<bool> ("Verbose.visual");
    verbose = tree.get<bool> ("Verbose.verbose");
}
void Config_Reader::add_path_config(const std::string &filename){
    // Create empty property tree object
    pt::ptree tree;

    // Parse the INI into the property tree.
    pt::read_ini (filename, tree);

    // Read all config parameters
    report_path = tree.get<std::string> ("Path.report_path");
    pure_GA_path = tree.get<std::string> ("Path.pure_GA_path");
    pure_icp_path = tree.get<std::string> ("Path.pure_icp_path");
    GA_icp_path = tree.get<std::string> ("Path.GA_icp_path");
    fine_icp_path = tree.get<std::string> ("Path.fine_icp_path");
    time_path = tree.get<std::string> ("Path.time_path");
    cad_models_path = tree.get<std::string>("Path.cad_models_path");
    model_data_path = tree.get<std::string>("Path.model_data_path");
    scene_data_path = tree.get<std::string>("Path.scene_data_path");
}
void Config_Reader::add_GA_parameters(const std::string &filename){
    // Create empty property tree object
    pt::ptree tree;

    // Parse the INI into the property tree.
    pt::read_ini (filename, tree);

    // Read all config parameters
    range_scale_t_ = tree.get<double> ("GA.range_scale_t");
    range_scale_euler_ = tree.get<double> ("GA.range_scale_euler");
    cluster_radius = tree.get<double> ("GA.cluster_radius_");
    population = tree.get<int> ("GA.population");
    generation_max = tree.get<int> ("GA.generation_max");
    best_stall_max = tree.get<int> ("GA.best_stall_max");
    average_stall_max = tree.get<int> ("GA.average_stall_max");
    tol_stall_average = tree.get<double> ("GA.tol_stall_average");
    tol_stall_best = tree.get<double> ("GA.tol_stall_best");
    crossover_fraction = tree.get<double> ("GA.crossover_fraction");
    mutation_rate = tree.get<double> ("GA.mutation_rate");
    elite_count = tree.get<int> ("GA.elite_count");
    multi_threading = tree.get<bool> ("GA.multi_threading");
    dynamic_threading = tree.get<bool> ("GA.dynamic_threading");
    mutation_method = tree.get<std::string> ("GA.mutation_method");
    fitness_method = tree.get<std::string> ("GA.fitness_method");
    fusion_level_ga = tree.get<int> ("GA.fusion_level_ga");
    fusion_mode = tree.get<std::string> ("GA.fusion_mode");
    F0 = tree.get<float>("GA.F0");
    set_threads = tree.get<bool>("GA.set_threads");

}
void Config_Reader::add_view_config(const std::string &filename){
    // Create empty property tree object
    pt::ptree tree;

    // Parse the INI into the property tree.
    pt::read_ini (filename, tree);

    // Read all config parameters
    window_width = tree.get<int> ("View.window_width");
    window_height = tree.get<int> ("View.window_height");
    point_size = tree.get<double> ("View.point_size");
    color_style = tree.get<std::string>("View.color_style");
}