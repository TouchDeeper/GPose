
#include "add_model/render_synthetic_views.h"
#include "config_reader.h"
#include "parameters.h"

/**
TODO:
	-Smoothing view clouds is not optimal when using view-merging later in system!
	-Fix function to remove a model
	-Save only views in original pose? Look where views are used in other programs (Could be resolved by just adding the transformation to views_original_pose in load_views). Is it nessesary to process views_xyz into views_N???
	-Remove ugly lines created by the rendering program
*/
void bug2(){
    td::pclib::PointNCloudPtr cloud(new td::pclib::PointNCloud);
    pcl::io::loadPCDFile<td::pclib::PointN>("bug_test.pcd",*cloud);
    pcl::visualization::PCLVisualizer viewer ("cloud viewer");
    viewer.addPointCloud<td::pclib::PointN>(cloud,"raw");
    viewer.spinOnce();
    pcl::VoxelGrid<td::pclib::PointN> sor;
    sor.setInputCloud (cloud);
    double leaf_size_ = 0.003;
    sor.setLeafSize (leaf_size_, leaf_size_, leaf_size_);
    sor.filter (*cloud);
    viewer.removeAllPointClouds();
    viewer.addPointCloud<td::pclib::PointN>(cloud,"downsample");
    viewer.spinOnce();

}
int main (int argc, char** argv)
{
	// Read configuration file
	Config_Reader* cr = Config_Reader::GetInstance();
	cr->add_model_load_config ("../../config.ini");
	ReadParameters("../../config.ini");
	// Create render object
	Render_Synthetic_Views render;
	
	// Set synthetic view properties
	render.set_resolution (cr->resolution);
	render.set_tessellation_level (cr->tessellation_level);
	render.set_radius_tessellated_sphere (cr->radius_tessellated_sphere);
	if (cr->optimal_view_angle)
	{
		render.set_use_optimal_view_angle ();
	}
	else
	{
		render.set_view_angle (cr->view_angle);
	}
	
	// Set view-processing parameters
	render.set_scale (cr->scale_factor);
	render.set_use_largest_cluster_extraction (cr->largest_cluster_extraction, cr->cluster_tolerance, cr->min_cluster_size, cr->max_cluster_size);
	render.set_use_downsample (cr->downsample, cr->leaf_size);
	render.set_use_smoothing (cr->smooth, cr->search_radius_mls);
	render.set_use_outlier_removal (cr->remove_outliers, cr->mean_k, cr->std_dev_mul_thresh);
	render.set_bad_normals_threshold (cr->bad_normals_threshold);
	render.set_view_processed_clouds (cr->view_processed_clouds);
	render.set_view_normals (cr->view_normals, cr->normal_magnitude);
	render.set_view_complete_model (cr->view_complete_model);
	render.set_view_graph (cr->view_graph);
	render.set_use_average_global_feature (cr->avg_glb_feature); 
	render.set_use_delay(cr->use_delay);
	render.set_fusion_level(cr->fusion_level);

	
	if (cr->use_k_search && cr->use_radius_search)
	{
		std::stringstream ss;
		ss << "ERROR: Please select either use_k_search or use_radius_search in config.ini! (Both cannot be true)\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE); 
	}
	else if (!cr->use_k_search && !cr->use_radius_search)
	{
		std::stringstream ss;
		ss << "ERROR: Please select either use_k_search or use_radius_search in config.ini! (One must be true)\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE); 
	}
	
	// Set variables for normal estimation
	if (cr->use_k_search)
	{
		render.set_k_search_normals (cr->k_search_normals);
	}
	else if (cr->use_radius_search)
	{
		render.set_radius_search_normals (cr->radius_search_normals);
	}
    bug2();
	// Render synthetic views
	render.start_rendering (argc, argv);
}













































