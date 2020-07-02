#include <parameters.h>
#include "add_model/render_synthetic_views.h"
/** 
  Constructor. 
*/
Render_Synthetic_Views::Render_Synthetic_Views (void)
{
	optimal_view_angle_ = false;
	largest_cluster_extraction_ = false;
	downsample_ = false;
	smooth_ = false;
	outlier_removal_ = false;
	view_processed_clouds_ = false;
	view_normals_ = false;
	view_complete_model_ = false;
	view_graph_ = false;
	global_feature_avg_ = false;
	use_k_search_ = false;
	use_radius_search_ = false;
}

/**
  Sets the resolution (window size)
  @param resolution The resolution
*/	
void 
Render_Synthetic_Views::set_resolution (int resolution)
{
	resolution_ = resolution;
}

/**
  Sets the tessellation level of the sphere
  @param tessellation_level The tessellation level
*/
void 
Render_Synthetic_Views::set_tessellation_level (int tessellation_level)
{
	tessellation_level_ = tessellation_level;
}

/**
  Enables the program to use the optimal filed of view for the camera
*/
void
Render_Synthetic_Views::set_use_optimal_view_angle ()
{
	optimal_view_angle_ = true;
}

/**
  Sets the view angle of the camera
  @param view_angle The view angle
*/
void 
Render_Synthetic_Views::set_view_angle (double view_angle)
{
	view_angle_ = view_angle;
}

/**
  Sets the radius of the tesselated sphere
  @param radius_tesselated_sphere
*/
void
Render_Synthetic_Views::set_radius_tessellated_sphere (double radius_tessellated_sphere)
{
	radius_tessellated_sphere_ = radius_tessellated_sphere;
}

/**
  Set the scaling of the CAD-model
  @param scale_factor The scaling factor
*/
void 
Render_Synthetic_Views::set_scale (double scale_factor)
{
	scale_factor_ = scale_factor;
}

/**
  Set the bad normal threshold when computing the normal-utilities. Bad normals are points that have normals almost perpendicular towards the viewing direction
  @param bad_normals_threshold The bad normal threshold (degrees). 
*/
void 
Render_Synthetic_Views::set_bad_normals_threshold (float bad_normals_threshold)
{
	bad_normals_threshold_ = bad_normals_threshold;
}

/**
  Set if the program should use largest cluster extraction for the rendered view-point-clouds
  @param largest_cluster_extraction True if use largest cluster extraction, false otherwise
  @param cluster_tolerance The cluster tolerance
  @param min_cluster_size The minimum size of the cluster
  @param max_cluster_size The maximum cluster size
*/
void 
Render_Synthetic_Views::set_use_largest_cluster_extraction (bool largest_cluster_extraction, 
															double cluster_tolerance, 
															int min_cluster_size, 
															int max_cluster_size )
{
	largest_cluster_extraction_ = largest_cluster_extraction;
	cluster_tolerance_ = cluster_tolerance;
	min_cluster_size_ = min_cluster_size;
	max_cluster_size_ = max_cluster_size;
}

/**
  Set if program should use downsampling
  @param downsample True is use downsampling, false otherwise
  @param leaf_size The voxel size
*/
void 
Render_Synthetic_Views::set_use_downsample (bool downsample, float leaf_size)
{
	downsample_ = downsample;
	leaf_size_ = leaf_size;
}

/**
  Set if program should use point cloud smoothing
  @param smooth True if use smoothing, false otherwise
  @param search_radius_mls The radius used for smoothing
*/
void 
Render_Synthetic_Views::set_use_smoothing (bool smooth, double search_raduis_mls)
{
	smooth_ = smooth;
	search_raduis_mls_ = search_raduis_mls;
}

/**
  Set if program should view the complete model once all point clouds have been rendered
  @param view_complete_model True if view complete model, false otherwise
*/
void
Render_Synthetic_Views::set_view_complete_model (bool view_complete_model)
{
	view_complete_model_ = view_complete_model;
}

/**
  Set if program should use outlier removal for rendered point clouds
  @param outlier_removal True if use outlier removal, false otherwise
  @param mean_k The number of nearest neighbors to use for mean distance estimation 
  @param std_dev_mul_thresh The standard deviation multiplier for the distance threshold calculation
*/
void 
Render_Synthetic_Views::set_use_outlier_removal (bool outlier_removal, int mean_k, double std_dev_mul_thresh)
{
	outlier_removal_ = outlier_removal;
	mean_k_ = mean_k;
	std_dev_mul_thresh_ = std_dev_mul_thresh;
}

/**
  Set the number of nearest neighbors to use during normal estimtaion
  @param k_search_normals The number of nearest neighbors
*/
void 
Render_Synthetic_Views::set_k_search_normals (int k_search_normals)
{
	use_k_search_ = true;
	k_search_normals_ = k_search_normals;
}

/**
  Set the search radius to use during normal estimation
  @param radius_search_normals The serach radius
*/
void 
Render_Synthetic_Views::set_radius_search_normals (double radius_search_normals)
{
	use_radius_search_ = true;
	radius_search_normals_ = radius_search_normals;
}

/**
  Set if program should show the processed point clouds
  @param view_processed_clouds True if view processed clouds, false otherwise
*/
void
Render_Synthetic_Views::set_view_processed_clouds (bool view_processed_clouds)
{
	view_processed_clouds_ = view_processed_clouds;
}

/**
  Set if program should show estimated normals for the rendered point clouds
  @param view_normals True if view normals, false otherwise
  @param normal_magnitude The magnitude of the displayed normal vectors (length)
*/
void 
Render_Synthetic_Views::set_view_normals (bool view_normals, float normal_magnitude)
{
	normal_magnitude_ = normal_magnitude;
	view_normals_ = view_normals;
}

/**
  Set if program should show view-graph
  @param view_graph True if show view-graph, false otherwise
*/
void
Render_Synthetic_Views::set_view_graph (bool view_graph)
{
	view_graph_ = view_graph;
}

/**
  Set if program should use the average global feature. If true then the program will estimate 10 global ESF features and take the average from all histograms
  @param global_feature_avg True if use average gloabl feature, false otherwise
*/
void
Render_Synthetic_Views::set_use_average_global_feature (bool global_feature_avg)
{
	global_feature_avg_ = global_feature_avg;
}

/**
  Set if program should use a small delay before rendering the first view. 
  @param use_delay True if use small delay, false otherwise
*/
void
Render_Synthetic_Views::set_use_delay(bool use_delay)
{
	use_delay_ = use_delay;
}

/**
  Demeans the point cloud
  @param cloud The point cloud
*/
void
Render_Synthetic_Views::set_fusion_level(int level){
    fusion_level_ = level;
}
void 
Render_Synthetic_Views::demean_cloud (PointCloudT::Ptr cloud)
{
	Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*cloud, centroid);
	pcl::demeanPointCloud<PointT> (*cloud, centroid, *cloud);
}

/**
  Performs euclidean cluster extraction on the point cloud. Only the largest cluster is kept. If the largets cluster is smaller than min_cluster_size (which means that no clusters were found) then the whole original point cloud is kept
  @param cloud The point cloud
*/
void 
Render_Synthetic_Views::euclidean_cluster_extraction (PointCloudT::Ptr cloud)
{
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (cloud);

	// Find cluster indices in the cloud
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (cluster_tolerance_);
	ec.setMinClusterSize (min_cluster_size_);
	ec.setMaxClusterSize (max_cluster_size_);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);
	
	// Extract the clusters
	std::vector<PointCloudT::Ptr> cloud_clusters;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		PointCloudT::Ptr cloud_cluster (new PointCloudT);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		{
			cloud_cluster->points.push_back (cloud->points[*pit]);
		}
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		cloud_clusters.push_back (cloud_cluster);
	}
	
	// Keep the largest cluster
	int max_size = 0;
	int index = 0;
	for (int i = 0; i < cloud_clusters.size(); i++)
	{
		if (cloud_clusters[i]->points.size() > max_size)
		{
			index = i;
			max_size = cloud_clusters[i]->points.size();
		}
	}
	
	if (max_size != 0)
	{
		copyPointCloud (*cloud_clusters[index], *cloud);
	}
}

/**
  Downsample the point cloud
  @param cloud The point cloud
*/
void 
Render_Synthetic_Views::downsample_cloud (PointCloudT::Ptr cloud)
{
//    pcl::io::savePCDFileASCII("bug_test.pcd",*cloud);
//    pcl::visualization::PCLVisualizer viewer ("cloud viewer");
//    viewer.addPointCloud<PointT>(cloud);
//    viewer.spin();
	pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (leaf_size_, leaf_size_, leaf_size_);
	sor.filter (*cloud);

//	viewer.removeAllPointClouds();
//    viewer.addPointCloud<PointT>(cloud);
//    viewer.spin();
}
void
Render_Synthetic_Views::downsample_cloud (PointCloud_N::Ptr cloud)
{
    pcl::VoxelGrid<Point_N> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (leaf_size_, leaf_size_, leaf_size_);
    sor.filter (*cloud);
}

/**
  Smooths the point clouds. Sharp edges are smoothed mimicing the real behaviour of the SR300 depth sensor
  @param cloud The point cloud
*/
void 
Render_Synthetic_Views::smooth_cloud (PointCloudT::Ptr cloud)
{
	// Create a KD-Tree
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	PointCloudT::Ptr mls_points (new PointCloudT ());

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<PointT, PointT> mls;

	mls.setComputeNormals (false);

	// Set parameters
	mls.setInputCloud (cloud);
	mls.setPolynomialFit (true);
	mls.setSearchMethod (tree);
	mls.setSearchRadius (search_raduis_mls_);

	// Reconstruct
	mls.process (*mls_points);
	copyPointCloud(*mls_points, *cloud);
}

/**
  Removes statistical outliers in the point cloud
  @param cloud The point cloud
*/
void 
Render_Synthetic_Views::sor_filter (PointCloudT::Ptr cloud)
{
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (mean_k_);
	sor.setStddevMulThresh (std_dev_mul_thresh_);
	sor.filter (*cloud);
}

/**
  Estimates surface normals in the point cloud
  @param cloud_xyz Point cloud without normals (pcl::PointXYZ)
  @param cloud_N Point cloud with normals (pcl::PointNormal)
  @param vp_x Sensor viewpoint x position
  @param vp_y Sensor viewpoint y position
  @param vp_z Sensor viewpoint z position
*/
void 
Render_Synthetic_Views::estimate_normals (	PointCloudT::Ptr cloud_xyz,  
											PointCloud_N::Ptr cloud_N,
											float vp_x,
											float vp_y,
											float vp_z )
{
	NormalEstimationNT normal_estimator;
	if (use_k_search_)
	{
		normal_estimator.setKSearch (k_search_normals_);
	}
	else if (use_radius_search_)
	{
		normal_estimator.setRadiusSearch (radius_search_normals_);
	}
	
	normal_estimator.setViewPoint(vp_x, vp_y, vp_z);
	PointCloudNT::Ptr normals (new PointCloudNT);
	normal_estimator.setInputCloud (cloud_xyz);
	normal_estimator.compute (*normals);

    // Concatenate cloud_xyz and normals into cloud_N
	pcl::concatenateFields(*cloud_xyz, *normals, *cloud_N);
}

/**
  Loads the CAD-model from file
  @param polydata The polydata from the loaded CAD-model
  @param argc Input argument index
  @param argv Input argument string
  @return model_name The model name
*/
std::string 
Render_Synthetic_Views::load_obj (vtkSmartPointer<vtkPolyData> &polydata, int argc, char** argv)
{
	std::vector<int> obj_file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".obj");
	if (obj_file_indices.size () != 1)
	{
		pcl::console::print_error ("\nNeed a single input OBJ file to continue.\n");
		return 0;
	}
	vtkSmartPointer<vtkOBJReader> readerQuery = vtkSmartPointer<vtkOBJReader>::New ();
    readerQuery->SetFileName (argv[obj_file_indices[0]]);
    readerQuery->Update ();
    polydata = readerQuery->GetOutput ();
    
	std::string model_name (argv[1]);
	model_name.erase (model_name.end() - 4, model_name.end()); // remove .obj
	
	// extract model name 
	while (model_name.find ("/") != std::string::npos) 
	{
		model_name = model_name.substr (model_name.find ("/") + 1);
	}
    
    return model_name;
}

/**
  Loads the CAD model from file. All CAD models are found in the CAD_models folder. Possible file formats are: OBJ, STL, PLY
  @param polydata The polydata from the loaded CAD-model
  @param argc Input argument index
  @param argv Input argument string
  @return model_name The model name
*/
std::string
Render_Synthetic_Views::load_polydata (vtkSmartPointer<vtkPolyData> &polydata, int argc, char** argv)
{	
	if (argc > 2)
	{
		pcl::console::print_error ("\nOnly one input file is allowed!\n");
		std::exit (EXIT_FAILURE);
	}
	else if (argc == 1)
	{
		pcl::console::print_error ("Please select an input file!\n\n");
		std::cout << "Usage: offline_data_generation INPUT\n\nINPUT is the specified CAD model placed in the CAD_models folder.\nSupported types are OBJ, STL and PLY\n" << std::endl;
		std::exit (EXIT_FAILURE);
	}
	
	// Load model
	std::vector<int> obj_file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".obj");
	std::vector<int> stl_file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".stl");
	std::vector<int> ply_file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".ply");
	Access_Model_Data* amd = Access_Model_Data::GetInstance();
	std::string model_name (argv[1]);
	if (obj_file_indices.size () == 1)
	{
		vtkSmartPointer<vtkOBJReader> readerQuery = vtkSmartPointer<vtkOBJReader>::New ();
		std::string file_path = amd->path_to_model_in_CAD_models (model_name);
		readerQuery->SetFileName (file_path.c_str());
		readerQuery->Update ();
		polydata = readerQuery->GetOutput ();
	}
	else if (stl_file_indices.size () == 1)
	{
		vtkSmartPointer<vtkSTLReader> readerQuery = vtkSmartPointer<vtkSTLReader>::New ();
		std::string file_path = amd->path_to_model_in_CAD_models (model_name);
		readerQuery->SetFileName (file_path.c_str());
		readerQuery->Update ();
		polydata = readerQuery->GetOutput ();
	}
	else if (ply_file_indices.size () == 1)
	{
		vtkSmartPointer<vtkPLYReader> readerQuery = vtkSmartPointer<vtkPLYReader>::New ();
		std::string file_path = amd->path_to_model_in_CAD_models (model_name);
		readerQuery->SetFileName (file_path.c_str());
		readerQuery->Update ();
		polydata = readerQuery->GetOutput ();
	}
	else
	{
		pcl::console::print_error ("\nNeed a single input OBJ, STL or PLY file to continue.\n");
		std::exit (EXIT_FAILURE);
	}
    
    // Remove file extension (.obj, .stl or .ply)
    model_name.erase (model_name.end() - 4, model_name.end()); 
    
    return model_name;
}	

/**
  Renderes the view-point-clouds
  @param polydata The polydata of the CAD-model
  @param views_xyz The rendered view-point-clouds
  @param poses, The poses (transformations) of the rendered view-point-clouds
  @param cam_pos The camera positions
*/
void 
Render_Synthetic_Views::render_views (	vtkSmartPointer<vtkPolyData> &polydata, 
										std::vector<PointCloudT::Ptr> &views_xyz, 
										std::vector <Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &poses, 
										std::vector <Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &cam_pos )
{
	Render_Views_Tesselated_Sphere_Modified view_render;
	view_render.addModelFromPolyData (polydata);
	view_render.setResolution (resolution_);
	view_render.setUseVertices (false);
	view_render.setRadiusSphere (radius_tessellated_sphere_);
	view_render.setComputeEntropies (false);
	view_render.setTessellationLevel (tessellation_level_);
	if(use_delay_)
		view_render.setUseDelay(use_delay_);

	// Create a bounding box that encloses the object. Use the diagonals of the bounding box to get the object radius.
	double bounds[6];
	polydata->GetBounds(bounds);
	double object_radius = sqrt (pow (bounds[0] - bounds[1], 2) + pow (bounds[2] - bounds[3], 2) + pow (bounds[4] - bounds[5] ,2)) * scale_factor_;
	view_render.setObjectRadius (object_radius);
	printf ("Object radius: %3.10f\n", object_radius);
	
	if (radius_tessellated_sphere_ - object_radius < 0.0)
	{
		std::cerr << "\nERROR: Object is to close to the camera! Move the camera further away by increasing the radius of the tesselated sphere!\n" << std::endl;
		std::exit (EXIT_FAILURE);
	}
	
	if (optimal_view_angle_)
	{
		// Compute optimal viewing angle
		double pi_const = atan (1.0) * 4; // Pi
		view_angle_ = 2 * atan ((object_radius/2)/radius_tessellated_sphere_) * (180/pi_const);

		// Add 10% to view_angle_ 
		view_angle_ *= 1.1;
		
		printf ("Optimal view angle: %3.10f\n", view_angle_);
	}
	
	view_render.setViewAngle (view_angle_);
	view_render.setScaleFactor (scale_factor_);
	
	view_render.generateViews ();
	
	view_render.getPoses (poses);
	view_render.getCamPositions (cam_pos);
	view_render.getViews (views_xyz);
}


/**
  Processes the rendered view-point-clouds
  @param views_xyz The unprocessed point clouds
  @param views_N The processed point clouds with estimated normals
*/
void 
Render_Synthetic_Views::process_clouds (std::vector<PointCloudT::Ptr> views_xyz, std::vector<PointCloud_N::Ptr> &views_N)
{	
	for (int i = 0; i < views_xyz.size(); i++)
	{
		// Copy views_xyz to views_processed
		PointCloudT::Ptr processed_cloud (new PointCloudT);
		copyPointCloud (*views_xyz[i], *processed_cloud);
		
		// Center the clouds at origin
		demean_cloud (processed_cloud);
		
		if (downsample_) 
		{
			// Downsample the clouds
			downsample_cloud (processed_cloud);
		}
		
		if (outlier_removal_)
		{
			// Remove statistical outliers from the clouds
			sor_filter (processed_cloud);
		}
		
		if (smooth_)
		{
			// Smooth the clouds
			smooth_cloud (processed_cloud);
		}
	
		if (largest_cluster_extraction_)
		{
			// Extract the largest cluster from the clouds
			euclidean_cluster_extraction (processed_cloud);
		}
		
		// Center the clouds at origin
		demean_cloud (processed_cloud);
		
		// Estimate normals and store the result in PointNormal clouds
		PointCloud_N::Ptr processed_cloud_N (new PointCloud_N);
		//because the cloud has been translate(0,0.-object_center of processed_cloud) to the origin in the camera coordinate system,
		// so viewpoint need to translate from (0,0,0) to (0,0,-radius_tessellated_sphere_)
		estimate_normals (processed_cloud, processed_cloud_N, 0.0, 0.0, -radius_tessellated_sphere_);

		if (processed_cloud->points.size() == 0)
		{
			pcl::console::print_error ("Processed point cloud is empty! Please check the config.ini file for less aggressive procesing\n\n");
			std::exit (EXIT_FAILURE);
		}
		
		views_N.push_back (processed_cloud_N);
	}
}

/**
  Processes the rendered view-point-clouds in origianl pose
  @param views_xyz The unprocessed point clouds 
  @param views_original_pose_N The processed point clouds with estimated normals in origianl pose
*/
void 
Render_Synthetic_Views::process_clouds_original_pose (	std::vector<PointCloudT::Ptr> views_xyz, 
														std::vector<PointCloud_N::Ptr> &views_original_pose_N, 
														std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > poses,
														std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > cam_pos ) 
{	
	for (int i = 0; i < views_xyz.size(); i++)
	{
		// Rotate cloud back to original pose
		Eigen::Matrix3d rotation_matrix = poses[i].block<3,3> (0,0).inverse ();
		Eigen::Vector3d translation_vector = cam_pos[i];
		Eigen::Matrix4d transformation;
		transformation.block<3,3> (0,0) = rotation_matrix;
		transformation (0,3) = radius_tessellated_sphere_*translation_vector (0);
		transformation (1,3) = radius_tessellated_sphere_*translation_vector (1);
		transformation (2,3) = radius_tessellated_sphere_*translation_vector (2);
		transformation (3,3) = 1.0;

		//modified by myself
//		Eigen::Matrix4d transformation = poses[i].inverse();

		
		PointCloudT::Ptr processed_cloud (new PointCloudT);
		pcl::transformPointCloud (*views_xyz[i], *processed_cloud, transformation);
		
		// Rotate viewpoint 
		PointCloudT::Ptr temp (new PointCloudT);
		PointT p;
		p.x = 0.0;
		p.y = 0.0;
		p.z = 0.0;
		temp->push_back (p);
		pcl::transformPointCloud (*temp, *temp, transformation);
		
		if (downsample_) 
		{
			// Downsample the clouds
			downsample_cloud (processed_cloud);
		}
		
		if (outlier_removal_)
		{
			// Remove statistical outliers from the clouds
			sor_filter (processed_cloud);
		}
		
		if (smooth_)
		{
			// Smooth the clouds
			smooth_cloud (processed_cloud);
		}
	
		if (largest_cluster_extraction_)
		{
			// Extract the largest cluster from the clouds
			euclidean_cluster_extraction (processed_cloud);
		}
		
		// Estimate normals and store the result in PointNormal clouds
		PointCloud_N::Ptr processed_cloud_N (new PointCloud_N);
		estimate_normals (processed_cloud, processed_cloud_N, temp->points[0].x, temp->points[0].y, temp->points[0].z);
		
		if (processed_cloud->points.size() == 0)
		{
			pcl::console::print_error ("Processed point cloud is empty! Please check the config.ini file for less aggressive procesing\n\n");
			std::exit (EXIT_FAILURE);
		}
		
		views_original_pose_N.push_back (processed_cloud_N);
	}
}

/**
  Processes the complete model point cloud
  @param cloud The point cloud of the complete model
*/
void 
Render_Synthetic_Views::process_complete_model (PointCloudT::Ptr cloud)
{	
	// Downsample the cloud
	downsample_cloud (cloud);
	
	if (smooth_)
	{
		// Smooth the cloud
		smooth_cloud (cloud);
	}
}

/**
  Computes the average ESF feature
  @param vec Vector containing the ESF features
  @return f_avg The average ESF feature
*/
Render_Synthetic_Views::FeatureG 
Render_Synthetic_Views::average_feature (std::vector<FeatureCloudG::Ptr> vec)
{
	FeatureG f_avg;
	for (int i = 0; i < 640; i++)
	{
		f_avg.histogram[i] = 0.0f;
	}
	
	for (int i = 0; i < 640; i++)
	{
		for (int j = 0; j < vec.size(); j++)
		{
			f_avg.histogram[i] += vec[j]->points[0].histogram[i];
		}
		f_avg.histogram[i] /= (float)vec.size();
	}
	return f_avg;
}

/**
  Estimates global ESF features for the rendered view-point-clouds
  @param views_original_pose_N The view-point-clouds in original pose 
  @param features Point cloud containing all the global ESF features
*/
void 
Render_Synthetic_Views::estimate_features_esf (std::vector<PointCloud_N::Ptr> views_original_pose_N, FeatureCloudG::Ptr features)
{
	for (int i = 0; i < views_original_pose_N.size(); i++)
	{
		std::cout << "\tEstimating global ESF feature for view " << i << endl;
		
		if (global_feature_avg_)
		{
			// Estimate 10 global ESF features for each view and store them in vec
			std::vector<FeatureCloudG::Ptr> vec;
			for (int j = 0; j < 10; j++)
			{
				FeatureEstimationG feature_estimator;
				FeatureCloudG::Ptr f_cloud (new FeatureCloudG);
				feature_estimator.setInputCloud (views_original_pose_N[i]);
				feature_estimator.compute (*f_cloud);
				vec.push_back (f_cloud);
			}
		
			// Take the average of vec and add it to the feature cloud
			features->push_back (average_feature (vec));
		}
		else
		{
			// Estimate 1 global ESF features for each view and add it to the feature cloud
			FeatureEstimationG feature_estimator;
			FeatureCloudG::Ptr f_cloud (new FeatureCloudG);
			feature_estimator.setInputCloud (views_original_pose_N[i]);
			feature_estimator.compute (*f_cloud);
			features->push_back (f_cloud->points[0]);
		}
	}
}

/**
  Views the processed point clouds
  @param views_xyz The original unprocessed point clouds
  @param views_N The processed point clouds
*/
void 
Render_Synthetic_Views::processed_cloud_viewer (std::vector<PointCloudT::Ptr> views_xyz, std::vector<PointCloud_N::Ptr> views_N)
{
	pcl::visualization::PCLVisualizer viewer ("Viewer");
	int vp_1;
	int vp_2;
	viewer.createViewPort (0.0, 0, 0.5, 1.0, vp_1);
	viewer.createViewPort (0.5, 0, 1.0, 1.0, vp_2);
	viewer.addText ("Unprocessed", 10, 10, 18, 1.0, 1.0, 1.0, "text1", vp_1);
	viewer.addText ("Processed", 10, 10, 18, 1.0, 1.0, 1.0, "text2", vp_2);
	viewer.addCoordinateSystem (0.03);
	
	for (int i = 0; i < views_xyz.size(); i++)
	{
		PointCloudT::Ptr temp (new PointCloudT);
		copyPointCloud (*views_xyz[i], *temp);
		demean_cloud (temp);
		viewer.removePointCloud ("view_original");
		viewer.removePointCloud ("view_processed");
		viewer.addPointCloud<PointT> (temp, "view_original", vp_1);
		viewer.addPointCloud<Point_N> (views_N[i], "view_processed", vp_2);
		if (i == 0)
		{
			std::cout << "Press Q to view next rendered point cloud..." << std::endl;
		}
		
		viewer.spin ();
	}
}

/**
  Views the normals of the processed point clouds
  @param views_N The processed point clouds with normals
  @param views_original_pose_N The processed point clouds with normals in original pose
*/
void 
Render_Synthetic_Views::normals_viewer (std::vector<PointCloud_N::Ptr> views_N, std::vector<PointCloud_N::Ptr> views_original_pose_N)
{
	//View point clouds
	pcl::visualization::PCLVisualizer viewer ("Virtual scene rendering");
	int vp_1;
	int vp_2;
	int vp_3;
	int vp_4;
	viewer.createViewPort (0.0, 0.5, 0.5, 1.0, vp_1);
	viewer.createViewPort (0.5, 0.5, 1.0, 1.0, vp_2);
	viewer.createViewPort (0.0, 0.0, 0.5, 0.5, vp_3);
	viewer.createViewPort (0.5, 0.0, 1.0, 0.5, vp_4);
	viewer.addText ("Cloud", 10, 10, 18, 1.0, 1.0, 1.0, "text1", vp_1);
	viewer.addText ("Normals", 10, 10, 18, 1.0, 1.0, 1.0, "text2", vp_2);
	viewer.addText ("Cloud in original pose", 10, 10, 18, 1.0, 1.0, 1.0, "text3", vp_3);
	viewer.addText ("Normals for cloud in original pose", 10, 10, 18, 1.0, 1.0, 1.0, "text4", vp_4);
	viewer.addCoordinateSystem (0.05);
	
	for (int i = 0; i < views_N.size(); i++)
	{
		// View xyz in left viewport
		viewer.removePointCloud ("view_xyz");
		viewer.addPointCloud<Point_N> (views_N[i], "view_xyz", vp_1);
		viewer.removePointCloud ("view_original_pose_xyz");
		viewer.addPointCloud<Point_N> (views_original_pose_N[i], "view_original_pose_xyz", vp_3);
		
		// View normals in right viewport
		viewer.removePointCloud ("view_normals");
		viewer.addPointCloudNormals<Point_N> (views_N[i], 1, normal_magnitude_, "view_normals", vp_2);
		viewer.removePointCloud ("view_original_pose_normals");
		viewer.addPointCloudNormals<Point_N> (views_original_pose_N[i], 1, normal_magnitude_, "view_original_pose_normals", vp_4);
		viewer.removePointCloud ("view_original_pose_xyz_vp4");
		pcl::visualization::PointCloudColorHandlerCustom<Point_N> single_color(views_original_pose_N[i], 0, 255, 0);
		viewer.addPointCloud<Point_N> (views_original_pose_N[i],single_color,"view_original_pose_xyz_vp4", vp_4);
		if (i == 0)
		{
			std::cout << "Press Q to view next rendered point cloud..." << std::endl;
		}
		viewer.spin ();
	}
}

/**
  Merges all rendered view-point-clouds into one complete point cloud
  @param view_xyz original point clouds
  @param poses The transformations
  @param cam_pos The camera positions
  @param merged_cloud_processed 
*/
void 
Render_Synthetic_Views::merge_views (	std::vector<PointCloudT::Ptr> views_xyz, 
										std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > poses,
										std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > cam_pos, 
										PointCloudT::Ptr merged_cloud_processed )
{
	PointCloudT::Ptr merged_cloud (new PointCloudT);
	for (int i = 0; i < views_xyz.size(); i++)
	{
		//*
		Eigen::Matrix3d rotation_matrix = poses[i].block<3,3> (0,0).inverse ();
		Eigen::Vector3d translation_vector = cam_pos[i];
		Eigen::Matrix4d transformation;
		transformation.block<3,3> (0,0) = rotation_matrix;
		transformation (0,3) = radius_tessellated_sphere_*translation_vector (0);
		transformation (1,3) = radius_tessellated_sphere_*translation_vector (1);
		transformation (2,3) = radius_tessellated_sphere_*translation_vector (2);
		transformation (3,3) = 1.0;
		//*/

//		Eigen::Matrix4d transformation = poses[i].inverse ();
//
		PointCloudT::Ptr rotated_model (new PointCloudT ());

		pcl::transformPointCloud (*views_xyz[i], *rotated_model, transformation);

		*merged_cloud += *rotated_model;
	}
	
	// Process complete model
	copyPointCloud (*merged_cloud, *merged_cloud_processed);
	process_complete_model (merged_cloud_processed);

	if (view_complete_model_)
	{
		// View complete model before and after processing
		pcl::visualization::PCLVisualizer viewer ("Viewer");
		int vp_1;
		int vp_2;
		viewer.createViewPort (0.0, 0, 0.5, 1.0, vp_1);
		viewer.createViewPort (0.5, 0, 1.0, 1.0, vp_2);
		viewer.addCoordinateSystem(0.03,"reference1",vp_1);
		viewer.addCoordinateSystem(0.03,"reference2",vp_2);
		viewer.addPointCloud<PointT> (merged_cloud, "merged_cloud", vp_1);
		viewer.addPointCloud<PointT> (merged_cloud_processed, "merged_cloud_processed", vp_2);
		std::cout << "Press Q to continue..." << std::endl;
		viewer.spin ();
	}
}

/**
  Normalizes a vector between 0 and 1
  @param vec The vector to be normalized
*/
void 
Render_Synthetic_Views::normalize (std::vector<float> &vec)
{			
	// Sort vec
	std::vector<float> temp = vec;
	std::sort(temp.begin(), temp.end());
	
	float max_value = temp.back();
	float min_value = temp.front();
	for (int i = 0; i < vec.size(); i++)
	{
		vec[i] = (vec[i] - min_value) / (max_value - min_value);
	}
}

/**
  Returns the view-utilities for all rendered point clouds
  @param views_original_pose_N The rendered point clouds in original pose
  @param utilities The utility vector containing a view-utility value for each view
*/
void 
Render_Synthetic_Views::get_utilities (std::vector<PointCloud_N::Ptr> views_original_pose_N, std::vector<float> &utilities)
{
	// Measure utility by calculating how many points that are visible in the processed point clouds
	for (int i = 0; i < views_original_pose_N.size(); i++)
	{
		utilities[i] = views_original_pose_N[i]->points.size();
	}
	
	// Normalize view-utilities between 1.0 and 0.0
	normalize (utilities);
}

/**
  Generates a graph using the information from the rendered tesselated sphere
  @param poses The point cloud transformations
  @param graph The resulting graph
*/
void 
Render_Synthetic_Views::generate_graph (std::vector <Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > poses, View_Graph &graph )
{
	// Extract viewpoints
	PointCloudT::Ptr viewpoint_cloud (new PointCloudT);
	Eigen::Vector3d translation;
	PointT p;
	for (int i = 0; i < poses.size(); i++)
	{
		translation = (poses[i].inverse()).block<3,1>(0, 3);
		p.x = translation (0);
		p.y = translation (1);
		p.z = translation (2);
		viewpoint_cloud->push_back (p);
	}
	
	// Demean viewpoint_cloud
	demean_cloud (viewpoint_cloud);
	
	// Extract rotations
	std::vector<Eigen::Matrix3d> rotation_vec;
	for (int i = 0; i < poses.size(); i++)
	{
		Eigen::Matrix3d rot;
		rot = (poses[i].inverse()).block<3,3>(0, 0);
		rotation_vec.push_back (rot);
	}
	
	// Connect the graph by establishing links between the nearest neighbors of the nodes. The faces of a icosahedron always has three nearest neighbors
	std::vector<std::vector<int> > neighbors_vec;
	pcl::search::KdTree<PointT> match_search;
	match_search.setInputCloud (viewpoint_cloud);
	int k = 4; // Three closest neighbors + the face itself
	
	for (int i = 0; i < poses.size(); i++)
	{
		std::vector<int> neigh_indices (k); 
		std::vector<float> neigh_sqr_dists (k);
		int found_neighs = match_search.nearestKSearch (viewpoint_cloud->points[i], k, neigh_indices, neigh_sqr_dists); // search for nearest neighbors 
		
		// Add neighbors to node (no self loops)
		std::vector<int> neighbors;
		for (int n = 0; n < k; n++)
		{
			if (neigh_indices[n] != i)
			{
				neighbors.push_back (neigh_indices[n]);
			}
		} 
		
		neighbors_vec.push_back (neighbors);
	}	
	
	// Add nodes to graph
	for (int i = 0; i < viewpoint_cloud->points.size(); i++)
	{
		graph.add_node (viewpoint_cloud->points[i], rotation_vec[i], neighbors_vec[i]);
	}
}

/**
  Views the view-graph
  @param graph The view-graph
  @param complete_model The complete point cloud model
*/
void
Render_Synthetic_Views::graph_viewer (View_Graph graph, PointCloudT::Ptr complete_model)
{
	// Add graph to viewer
	pcl::visualization::PCLVisualizer viewer ("Viewer");
	viewer.setBackgroundColor(VIEWSTYLE.background_color[0],VIEWSTYLE.background_color[1],VIEWSTYLE.background_color[2]);
	graph.add_graph_to_viewer (viewer, 0.02, 0, true, true);
	
	// Add complete model to viewer
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(complete_model,VIEWSTYLE.model_rgb[0],VIEWSTYLE.model_rgb[1],VIEWSTYLE.model_rgb[2]);
	viewer.addPointCloud<PointT> (complete_model,rgb);
    viewer.addCoordinateSystem(0.05);
	std::cout << "Press Q to continue..." << std::endl;
	viewer.spin ();
}

/**
  Generates local FPFH features
  @param views_original_pose_N The view-point-clouds in original pose
  @param local_features The FPFH features
*/
void
Render_Synthetic_Views::generate_local_features (std::vector<PointCloud_N::Ptr> views_original_pose_N, std::vector<FeatureCloudL::Ptr> &local_features)
{
	for (int i = 0; i < views_original_pose_N.size(); i++)
	{
		//save the only point
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(*views_original_pose_N[i], *cloud);
		//save the only normal
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::copyPointCloud(*views_original_pose_N[i], *normals);
//        //View point clouds
//        pcl::visualization::PCLVisualizer viewer ("Virtual scene rendering");
//        viewer.addCoordinateSystem (0.05);
//
//
//
//        // View xyz in left viewport
//        viewer.addPointCloud<Point_N> (views_original_pose_N[0], "view_original_pose_xyz");
//
//        // View normals in right viewport
//        viewer.addPointCloudNormals<Point_N> (views_original_pose_N[0], 1, normal_magnitude_, "view_original_pose_normals");
//        pcl::visualization::PointCloudColorHandlerCustom<Point_N> single_color(views_original_pose_N[0], 0, 255, 0);
//        viewer.spin ();

        // Estimate local FPFH features
		FeatureCloudL::Ptr features (new FeatureCloudL);
		FeatureEstimationL feature_estimator;

		feature_estimator.setRadiusSearch(0.01);
		feature_estimator.setInputCloud(views_original_pose_N[i]);
		feature_estimator.setInputNormals(views_original_pose_N[i]);
		feature_estimator.compute(*features);

		// FPFH estimation object.
//		pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
//		fpfh.setInputCloud(cloud);
//		fpfh.setInputNormals(normals);
//
//		// A kd-tree is a data structure that makes searches efficient. More about it later.
//		// The normal estimation object will use it to find nearest neighbors.
//		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>);
//		fpfh.setSearchMethod(kdTree);
//		// Search radius, to look for neighbors. Note: the value given here has to be
//		// larger than the radius used to estimate the normals.
//		fpfh.setRadiusSearch(0.01);
//		// Object for storing the FPFH descriptors for each point.
//		pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(new pcl::PointCloud<pcl::FPFHSignature33>());
//		fpfh.compute(*features);

		local_features.push_back (features);
	}
}

/**
  Generates fusion view local FPFH features
  @param views_original_pose_N The view-point-clouds in original pose
  @param local_features The FPFH features
*/
void
Render_Synthetic_Views::generate_fusion_local_features (std::vector<PointCloud_N::Ptr> views_original_pose_N, std::vector<std::vector<FeatureCloudL::Ptr>> &local_features, View_Graph &view_graph)
{
    for (int i = 0; i < views_original_pose_N.size(); i++)
    {
        std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> fuse_results;
//        fuse_multi_level_neighbor(i,fuse_results,view_graph,views_original_pose_N);
        std::vector<FeatureCloudL::Ptr> local_feature;
        for (int j = 0; j < fuse_results.size(); ++j) {
            downsample_cloud(fuse_results[j]);
            pcl::visualization::PCLVisualizer viewer("fusion view");
            viewer.addPointCloud<pcl::PointNormal>(fuse_results[j],"fusion_view");
            viewer.spin();
            //save the only point
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*fuse_results[j], *cloud);


            //save the only normal
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//            pcl::copyPointCloud(*fuse_results[j], *normals);

            // FPFH estimation object.
            pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
            fpfh.setInputCloud(cloud);
            fpfh.setInputNormals(normals);

            // A kd-tree is a data structure that makes searches efficient. More about it later.
            // The normal estimation object will use it to find nearest neighbors.
            pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>);
            fpfh.setSearchMethod(kdTree);
            // Search radius, to look for neighbors. Note: the value given here has to be
            // larger than the radius used to estimate the normals.
            fpfh.setRadiusSearch(0.01);
            // Object for storing the FPFH descriptors for each point.
            pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(new pcl::PointCloud<pcl::FPFHSignature33>());
            fpfh.compute(*features);
            local_feature.push_back(features);

        }

        // Estimate local FPFH features
//		FeatureCloudL::Ptr features (new FeatureCloudL);
//		FeatureEstimationL feature_estimator;
//
//		feature_estimator.setRadiusSearch(0.01);
//		feature_estimator.setInputCloud(views_original_pose_N[i]);
//		feature_estimator.setInputNormals(views_original_pose_N[i]);
//		feature_estimator.compute(*features);



        local_features.push_back (local_feature);
    }
}

/**
  Renderes synthetic 2.5D point clouds of a CAD-model. The synthetic point clouds are generated by placing a virtual camera around the object in a sphere-like pattern. 
  @param argc Input argument index
  @param argv Input argument string
*/
void
Render_Synthetic_Views::start_rendering (int argc, char** argv)
{
	// Load CAD model as polydata
	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New ();
	std::string model_name = load_polydata (polydata, argc, argv);


	// Render virtuall 2.5D views of CAD model
	std::vector<PointCloudT::Ptr> views_xyz;
	std::vector <Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > poses;
	std::vector <Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > cam_pos;
	std::cout << "\nRendering views of " <<model_name<<" ..." << std::endl;
	render_views (polydata, views_xyz, poses, cam_pos);
	std::cout << "Done\n" << std::endl;
	
	std::cout << "Number of views rendered: " << views_xyz.size() << "\n" << std::endl;
	
	// Process and estimate normals for rendered point clouds
	std::cout << "Processing and estimating normals for all views..." << std::endl;
	std::vector<PointCloud_N::Ptr> views_N;
	process_clouds (views_xyz, views_N);
	std::cout << "Done\n" << std::endl;
	
	// Process and estimate normals for rendered point clouds in original pose
	std::cout << "Processing and estimating normals for all views in original pose..." << std::endl;
	std::vector<PointCloud_N::Ptr> views_original_pose_N;
	process_clouds_original_pose (views_xyz, views_original_pose_N, poses, cam_pos);
	std::cout << "Done\n" << std::endl;
	
	// Merge all views to get a complete 3D point cloud model
	std::cout << "Merging all views to create a complete 3D model..." << std::endl;
	PointCloudT::Ptr complete_model (new PointCloudT);
	merge_views (views_xyz, poses, cam_pos, complete_model);
	std::cout << "Done\n" << std::endl;
	
	// Save rendered point clouds
	std::cout << "Saving point clouds..." << std::endl;
	Access_Model_Data* access = Access_Model_Data::GetInstance();
	access->save_view_clouds (model_name, views_N, views_original_pose_N, complete_model);
	std::cout << "Done\n" << std::endl;
	
	if (view_processed_clouds_)
	{
		// View original and processed point clouds
		processed_cloud_viewer (views_xyz, views_N);
	}
	
	if (view_normals_)
	{
		// View normals
		normals_viewer (views_N, views_original_pose_N);
	}
	
	// Generate view-graph
	View_Graph graph;
	std::cout << "Generating view-graph..." << std::endl;
	generate_graph (poses, graph);
	std::cout << "Done\n" << std::endl;
	
	// Save graph
	std::cout << "Saving graph..." << std::endl;
	graph.set_model_name (model_name);
	graph.save_graph ();
	std::cout << "Done\n" << std::endl;
	
	if (view_graph_)
	{
		// View graph and complete model
		graph_viewer (graph, complete_model);
	}
	std::cout<<"generate relative matrix and save it"<<std::endl;
	graph.generate_relative_matrix();
	std::cout<<"Done"<<std::endl;

	// Calculate view-utilities 
	std::cout << "Calculating view-utilities..." << std::endl;
	std::vector<float> view_utilities(views_original_pose_N.size());
	get_utilities (views_original_pose_N, view_utilities);
	std::cout << "Done\n" << std::endl;
	
	// Save view-utilities 
	std::cout << "Saving view-utilities..." << std::endl;
	access->save_view_utilities (model_name, view_utilities);
	std::cout << "Done\n" << std::endl;
	
	// Generate global features for each view
	FeatureCloudG::Ptr global_features (new FeatureCloudG);
	std::cout << "Estimating global features for each view...\n" << std::endl;
	estimate_features_esf (views_original_pose_N, global_features);
	std::cout << "\nDone\n" << std::endl;
	
	// Save global features
	std::cout << "Saving global features..." << std::endl;
	access->save_global_features (model_name, global_features);
	std::cout << "Done\n" << std::endl;
	
	// Search for similar objects
//	std::cout << "Searching for similar objects..." << std::endl;
//	Similar_Object_Recognition sim_obj;
//	sim_obj.add_model (model_name, global_features);
//	std::cout << "Done\n" << std::endl;
	
	// Generate feature utilities
//	std::cout << "Generating feature utilities..." << std::endl;
//	View_Feature_Score vfs;
//	std::vector<double> feature_utilities;
//	feature_utilities = vfs.compute_view_score_normalized(views_original_pose_N);
//	std::cout << "Done\n" << std::endl;
//
//	// Save feature utilities
//	std::cout << "Saving feature utilities..." << std::endl;
//	access->save_feature_utilities (model_name, feature_utilities);
//	std::cout << "Done\n" << std::endl;
//
//	// Generate normal utilities
//	std::cout << "Generating normal utilities..." << std::endl;
//	Normal_Utility nu;
//	std::vector<float> normal_utilities;
//	PointT v_p;
//	v_p.x = 0.0;
//	v_p.y = 0.0;
//	v_p.z = -radius_tessellated_sphere_;
//	normal_utilities = nu.generate_normal_utilities (views_N, v_p, bad_normals_threshold_);
//	std::cout << "Done\n" << std::endl;
//
//	// Save normal utilities
//	std::cout << "Saving normal utilities..." << std::endl;
//	nu.save_normal_utilities (model_name, normal_utilities);
//	std::cout << "Done\n" << std::endl;
	
	// Generate local features
	std::cout << "Generating local features..." << std::endl;
	std::vector<FeatureCloudL::Ptr> local_features;
	generate_local_features (views_original_pose_N, local_features);
	std::cout << "Done\n" << std::endl;
	
	// Save local features
	std::cout << "Saving local features..." << std::endl;
	access->save_local_features (model_name, local_features);
	std::cout << "Done\n" << std::endl;

	// Generate fusion view local features
    std::cout << "Generating fusion view ..." << std::endl;
    std::vector<std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>> fuse_results;
//    std::vector<std::vector<FeatureCloudL::Ptr>> fusion_local_features;
//    generate_fusion_local_features(views_original_pose_N,fusion_local_features,graph);
    fuse_multi_level_neighbor(fuse_results,graph,views_original_pose_N);
    std::cout<<"Done"<<std::endl;

    // Save fusion local features
    std::cout << "Saving fusion view..." << std::endl;
//    access->save_fusion_local_features (model_name, fusion_local_features);
    access->save_fusion_view_clouds(model_name, fuse_results);
    std::cout << "Done\n" << std::endl;

}
void
Render_Synthetic_Views::bug_rendering (int argc, char** argv)
{
    // Load CAD model as polydata
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New ();
    std::string model_name = load_polydata (polydata, argc, argv);


    // Render virtuall 2.5D views of CAD model
    std::vector<PointCloudT::Ptr> views_xyz;
    std::vector <Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > poses;
    std::vector <Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > cam_pos;

    std::cout << "\nRendering views..." << std::endl;
    render_views (polydata, views_xyz, poses, cam_pos);
    std::cout << "Done\n" << std::endl;

    std::cout << "Number of views rendered: " << views_xyz.size() << "\n" << std::endl;

    // Process and estimate normals for rendered point clouds
    std::cout << "Processing and estimating normals for all views..." << std::endl;
    std::vector<PointCloud_N::Ptr> views_N;
    process_clouds (views_xyz, views_N);
    std::cout << "Done\n" << std::endl;

    // Process and estimate normals for rendered point clouds in original pose
    std::cout << "Processing and estimating normals for all views in original pose..." << std::endl;
    std::vector<PointCloud_N::Ptr> views_original_pose_N;
    process_clouds_original_pose (views_xyz, views_original_pose_N, poses, cam_pos);
    std::cout << "Done\n" << std::endl;

    // Merge all views to get a complete 3D point cloud model
    std::cout << "Merging all views to create a complete 3D model..." << std::endl;
    PointCloudT::Ptr complete_model (new PointCloudT);
    merge_views (views_xyz, poses, cam_pos, complete_model);
    std::cout << "Done\n" << std::endl;

    // Save rendered point clouds
    std::cout << "Saving point clouds..." << std::endl;
    Access_Model_Data* access = Access_Model_Data::GetInstance();
    access->save_view_clouds (model_name, views_N, views_original_pose_N, complete_model);
    std::cout << "Done\n" << std::endl;

    if (view_processed_clouds_)
    {
        // View original and processed point clouds
        processed_cloud_viewer (views_xyz, views_N);
    }

    if (view_normals_)
    {
        // View normals
        normals_viewer (views_N, views_original_pose_N);
    }

    // Generate view-graph
    View_Graph graph;
    std::cout << "Generating view-graph..." << std::endl;
    generate_graph (poses, graph);
    std::cout << "Done\n" << std::endl;

    // Save graph
    std::cout << "Saving graph..." << std::endl;
    graph.set_model_name (model_name);
    graph.save_graph ();
    std::cout << "Done\n" << std::endl;

    if (view_graph_)
    {
        // View graph and complete model
        graph_viewer (graph, complete_model);
    }
    std::cout<<"generate relative matrix and save it"<<std::endl;
    graph.generate_relative_matrix();
    std::cout<<"Done"<<std::endl;
//
//    // Calculate view-utilities
//    std::cout << "Calculating view-utilities..." << std::endl;
//    std::vector<float> view_utilities(views_original_pose_N.size());
//    get_utilities (views_original_pose_N, view_utilities);
//    std::cout << "Done\n" << std::endl;
//
//    // Save view-utilities
//    std::cout << "Saving view-utilities..." << std::endl;
//    access->save_view_utilities (model_name, view_utilities);
//    std::cout << "Done\n" << std::endl;
//
    // Generate global features for each view
    FeatureCloudG::Ptr global_features (new FeatureCloudG);
    std::cout << "Estimating global features for each view...\n" << std::endl;
    estimate_features_esf (views_original_pose_N, global_features);
    std::cout << "\nDone\n" << std::endl;

    // Save global features
    std::cout << "Saving global features..." << std::endl;
    access->save_global_features (model_name, global_features);
    std::cout << "Done\n" << std::endl;
//
//    // Search for similar objects
//    std::cout << "Searching for similar objects..." << std::endl;
//    Similar_Object_Recognition sim_obj;
//    sim_obj.add_model (model_name, global_features);
//    std::cout << "Done\n" << std::endl;

    // Generate feature utilities
//	std::cout << "Generating feature utilities..." << std::endl;
//	View_Feature_Score vfs;
//	std::vector<double> feature_utilities;
//	feature_utilities = vfs.compute_view_score_normalized(views_original_pose_N);
//	std::cout << "Done\n" << std::endl;
//
//	// Save feature utilities
//	std::cout << "Saving feature utilities..." << std::endl;
//	access->save_feature_utilities (model_name, feature_utilities);
//	std::cout << "Done\n" << std::endl;
//
//	// Generate normal utilities
//	std::cout << "Generating normal utilities..." << std::endl;
//	Normal_Utility nu;
//	std::vector<float> normal_utilities;
//	PointT v_p;
//	v_p.x = 0.0;
//	v_p.y = 0.0;
//	v_p.z = -radius_tessellated_sphere_;
//	normal_utilities = nu.generate_normal_utilities (views_N, v_p, bad_normals_threshold_);
//	std::cout << "Done\n" << std::endl;
//
//	// Save normal utilities
//	std::cout << "Saving normal utilities..." << std::endl;
//	nu.save_normal_utilities (model_name, normal_utilities);
//	std::cout << "Done\n" << std::endl;

    // Generate local features
    std::cout << "Generating local features..." << std::endl;
    std::vector<FeatureCloudL::Ptr> local_features;
    generate_local_features (views_original_pose_N, local_features);
    std::cout << "Done\n" << std::endl;

    // Save local features
    std::cout << "Saving local features..." << std::endl;
    access->save_local_features (model_name, local_features);
    std::cout << "Done\n" << std::endl;

    // Generate fusion view local features
    std::cout << "Generating fusion view ..." << std::endl;
    std::vector<std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>> fuse_results;
//    std::vector<std::vector<FeatureCloudL::Ptr>> fusion_local_features;
//    generate_fusion_local_features(views_original_pose_N,fusion_local_features,graph);
    fuse_multi_level_neighbor(fuse_results,graph,views_original_pose_N);
    std::cout<<"Done"<<std::endl;

    // Save fusion local features
    std::cout << "Saving fusion view..." << std::endl;
//    access->save_fusion_local_features (model_name, fusion_local_features);
    access->save_fusion_view_clouds(model_name, fuse_results);
    std::cout << "Done\n" << std::endl;

}
/**
 * fuse a view with it's neighbor
 * @param index view index
 * @param fuse_result object to store the fusion result
 */
void
Render_Synthetic_Views::fuse_multi_level_neighbor(std::vector<std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>> &fuse_results,
                                                  View_Graph &view_graph,
                                                  const std::vector<PointCloud_N::Ptr> &views_original_pose_N){
    for (int i = 0; i < views_original_pose_N.size(); ++i) {
        std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> fuse_result_view_i;
        for (int j = 1; j <=fusion_level_; ++j) {
            pcl::PointCloud<pcl::PointNormal>::Ptr fuse_result(new pcl::PointCloud<pcl::PointNormal>);
            fuse_neighbor(i,fuse_result,view_graph,views_original_pose_N,j);
            downsample_cloud(fuse_result);

//            pcl::visualization::PCLVisualizer viewer("fusion view");
//            viewer.addPointCloud<pcl::PointNormal>(fuse_result,"fusion_view");
//            viewer.spin();

            fuse_result_view_i.push_back(fuse_result);
        }
        fuse_results.push_back(fuse_result_view_i);
    }
}
/**
* fuse a view with it's neighbor
* @param index view index
* @param fuse_result object to store the fusion result
* @param view_graph
* @param views_original_pose_N multi-view point cloud
*/
void
Render_Synthetic_Views::fuse_neighbor(const int &index, pcl::PointCloud<pcl::PointNormal>::Ptr fuse_result,
              View_Graph &view_graph, const std::vector<PointCloud_N::Ptr> &views_original_pose_N, int fusion_level){
    fusion_level--;
    std::vector<int> neighbor_indexs = view_graph.get_neighbors(index);
    for (int i = 0; i < neighbor_indexs.size(); ++i) {
        *fuse_result += *views_original_pose_N[neighbor_indexs[i]];

        if(fusion_level>=1)
            fuse_neighbor(neighbor_indexs[i],fuse_result, view_graph, views_original_pose_N, fusion_level);
    }
}






























