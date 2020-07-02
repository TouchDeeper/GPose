#ifndef RENDER_SYNTHETIC_VIEWS_H_
#define RENDER_SYNTHETIC_VIEWS_H_

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <vtkOBJReader.h>
#include <vtkSTLReader.h>
#include <vtkPLYReader.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/esf.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PointIndices.h>
#include <pcl/common/transforms.h>
#include "view_graph/view_graph.h"
#include "access_model_data.h"
#include "similar_object_recognition.h"
#include "view_feature_score.h"
#include "render_views_tesselated_sphere_modified.h"
#include <pcl/features/fpfh_omp.h>
#include "normal_utility.h"
#include <unistd.h>
#include "parameters.h"
#include <TdLibrary/PCL/local_features.h>

class Render_Synthetic_Views
{
	public:	
		/** 
		  Constructor. 
	  	*/
		Render_Synthetic_Views (void);
		
		/**
		  Sets the resolution (window size)
		  @param resolution The resolution
		*/
		void
		set_resolution (int resolution);
		
		/**
		  Set if program should view the complete model once all point clouds have been rendered
		  @param view_complete_model True if view complete model, false otherwise
		*/
		void
		set_view_complete_model (bool view_complete_model);
		
		/**
		  Sets the tessellation level of the sphere
		  @param tessellation_level The tessellation level
		*/
		void
		set_tessellation_level (int tessellation_level);
		
		/**
		  Enables the program to use the optimal filed of view for the camera
		*/
		void
		set_use_optimal_view_angle ();
		
		/**
		  Sets the view angle of the camera
		  @param view_angle The view angle
		*/
		void 
		set_view_angle (double view_angle);
		
		/**
		  Sets the radius of the tesselated sphere
		  @param radius_tesselated_sphere
		*/
		void 
		set_radius_tessellated_sphere (double radius_tessellated_sphere);
		
		/**
		  Set the scaling of the CAD-model
		  @param scale_factor The scaling factor
		*/
		void 
		set_scale (double scale_factor);
		
		/**
		  Set the bad normal threshold when computing the normal-utilities. Bad normals are points that have normals almost perpendicular towards the viewing direction
		  @param bad_normals_threshold The bad normal threshold (degrees). 
		*/
		void 
		set_bad_normals_threshold (float bad_normals_threshold);
		
		/**
		  Set if the program should use largest cluster extraction for the rendered view-point-clouds
		  @param largest_cluster_extraction True if use largest cluster extraction, false otherwise
		  @param cluster_tolerance The cluster tolerance
		  @param min_cluster_size The minimum size of the cluster
		  @param max_cluster_size The maximum cluster size
		*/
		void
		set_use_largest_cluster_extraction (bool largest_cluster_extraction, double cluster_tolerance, int min_cluster_size, int max_cluster_size);
		
		/**
		  Set if program should use downsampling
		  @param downsample True is use downsampling, false otherwise
		  @param leaf_size The voxel size
		*/
		void
		set_use_downsample (bool downsample, float leaf_size);
		
		/**
		  Set if program should use point cloud smoothing
		  @param smooth True if use smoothing, false otherwise
		  @param search_radius_mls The radius used for smoothing
		*/
		void
		set_use_smoothing (bool smooth, double search_raduis_mls);
		
		/**
		  Set if program should use outlier removal for rendered point clouds
		  @param outlier_removal True if use outlier removal, false otherwise
		  @param mean_k The number of nearest neighbors to use for mean distance estimation 
		  @param std_dev_mul_thresh The standard deviation multiplier for the distance threshold calculation
		*/
		void
		set_use_outlier_removal (bool outlier_removal, int mean_k, double std_dev_mul_thresh);
		
		/**
		  Set the number of nearest neighbors to use during normal estimtaion
		  @param k_search_normals The number of nearest neighbors
		*/
		void
		set_k_search_normals (int k_search_normals);
		
		/**
		  Set the search radius to use during normal estimation
		  @param radius_search_normals The serach radius
		*/
		void
		set_radius_search_normals (double radius_search_normals);
		
		/**
		  Set if program should show the processed point clouds
		  @param view_processed_clouds True if view processed clouds, false otherwise
		*/
		void
		set_view_processed_clouds (bool view_processed_clouds);
		
		/**
		  Set if program should show estimated normals for the rendered point clouds
		  @param view_normals True if view normals, false otherwise
		  @param normal_magnitude The magnitude of the displayed normal vectors (length)
		*/
		void 
		set_view_normals (bool view_normals, float normal_magnitude);
		
		/**
		  Set if program should show view-graph
		  @param view_graph True if show view-graph, false otherwise
		*/
		void
		set_view_graph (bool view_graph);
		
		/**
		  Set if program should use the average global feature. If true then the program will estimate 10 global ESF features and take the average from all histograms
		  @param global_feature_avg True if use average gloabl feature, false otherwise
		*/
		void
		set_use_average_global_feature (bool global_feature_avg);
		
		/**
		  Set if program should use a small delay before rendering the first view. 
		  @param use_delay True if use small delay, false otherwise
		*/
		void
		set_use_delay(bool use_delay);

		/**
		 * set fusion level
		 * @param level
		 */
        void
        set_fusion_level(int level);
		
		/**
		  Renderes synthetic 2.5D point clouds of a CAD-model. The synthetic point clouds are generated by placing a virtual camera around the object in a sphere-like pattern. 
		  @param argc Input argument index
		  @param argv Input argument string
		*/
		void
		start_rendering (int argc, char** argv);
        void
        bug_rendering (int argc, char** argv);
	private:
		typedef pcl::PointXYZ PointT;
		typedef pcl::Normal PointNT;
		typedef pcl::PointNormal Point_N;
		typedef pcl::PointCloud<PointT> PointCloudT;
		typedef pcl::PointCloud<PointNT> PointCloudNT;
		typedef pcl::PointCloud<Point_N> PointCloud_N;
		typedef pcl::ESFSignature640 FeatureG;
		typedef pcl::FPFHSignature33 FeatureL;
		typedef pcl::PointCloud<FeatureG> FeatureCloudG;
		typedef pcl::PointCloud<FeatureL> FeatureCloudL;
		typedef pcl::NormalEstimationOMP<PointT, PointNT> NormalEstimationNT;
		typedef pcl::ESFEstimation<Point_N, FeatureG> FeatureEstimationG;
		typedef pcl::FPFHEstimationOMP<Point_N, Point_N, FeatureL> FeatureEstimationL;
		
		int resolution_;
		int tessellation_level_;
		bool optimal_view_angle_;
		double view_angle_;
		double radius_tessellated_sphere_;
		double scale_factor_;
		bool demean_;
		bool largest_cluster_extraction_;
		bool downsample_;
		bool smooth_;
		bool outlier_removal_;
		int k_search_normals_;
		double radius_search_normals_;
		bool view_processed_clouds_;
		bool view_normals_;
		float normal_magnitude_;
		bool view_complete_model_;
		bool view_graph_;
		float vp_x_;
		float vp_y_;
		float vp_z_;
		float leaf_size_;
		double bad_normals_threshold_;
		bool global_feature_avg_;
		int mean_k_;
		double std_dev_mul_thresh_;
		double cluster_tolerance_;
		int min_cluster_size_;
		int max_cluster_size_;
		double search_raduis_mls_;
		bool use_k_search_;
		bool use_radius_search_;
		bool use_delay_;
		int fusion_level_;
		
		/**
		  Demeans the point cloud
		  @param cloud The point cloud
		*/
		void
		demean_cloud (PointCloudT::Ptr cloud) ;
		
		/**
		  Performs euclidean cluster extraction on the point cloud. Only the largest cluster is kept. If the largets cluster is smaller than min_cluster_size (which means that no clusters were found) then the whole original point cloud is kept
		  @param cloud The point cloud
		*/
		void
		euclidean_cluster_extraction (PointCloudT::Ptr cloud);
		
		/**
		  Downsample the point cloud
		  @param cloud The point cloud
		*/
		void
		downsample_cloud (PointCloudT::Ptr cloud);
        void
        downsample_cloud (PointCloud_N::Ptr cloud);
		
		/**
		  Smooths the point clouds. Sharp edges are smoothed mimicing the real behaviour of the SR300 depth sensor
		  @param cloud The point cloud
		*/
		void
		smooth_cloud (PointCloudT::Ptr cloud);
		
		/**
		  Removes statistical outliers in the point cloud
		  @param cloud The point cloud
		*/
		void
		sor_filter (PointCloudT::Ptr cloud);
		
		/**
		  Estimates surface normals in the point cloud
		  @param cloud_xyz Point cloud without normals (pcl::PointXYZ)
		  @param cloud_N Point cloud with normals (pcl::PointNormal)
		  @param vp_x Sensor viewpoint x position
		  @param vp_y Sensor viewpoint y position
		  @param vp_z Sensor viewpoint z position
		*/
		void 
		estimate_normals (	PointCloudT::Ptr cloud_xyz,  
							PointCloud_N::Ptr cloud_N,
							float vp_x,
							float vp_y,
							float vp_z );
		
		/**
		  Loads the CAD-model from file
		  @param polydata The polydata from the loaded CAD-model
		  @param argc Input argument index
		  @param argv Input argument string
		  @return model_name The model name
		*/
		std::string 
		load_obj (vtkSmartPointer<vtkPolyData> &polydata, int argc, char** argv);
		
		/**
		  Loads the CAD model from file. All CAD models are found in the CAD_models folder. Possible file formats are: OBJ, STL, PLY
		  @param polydata The polydata from the loaded CAD-model
		  @param argc Input argument index
		  @param argv Input argument string
		  @return model_name The model name
		*/
		std::string
		load_polydata (vtkSmartPointer<vtkPolyData> &polydata, int argc, char** argv);
		
		/**
		  Renderes the view-point-clouds
		  @param polydata The polydata of the CAD-model
		  @param views_xyz The rendered view-point-clouds
		  @param poses, The poses (transformations) of the rendered view-point-clouds
		  @param cam_pos The camera positions
		*/
		void
		render_views (	vtkSmartPointer<vtkPolyData> &polydata, 
						std::vector<PointCloudT::Ptr> &views_xyz, 
						std::vector <Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &poses,
						std::vector <Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &cam_pos );
		
		/**
		  Processes the rendered view-point-clouds
		  @param views_xyz The unprocessed point clouds
		  @param views_N The processed point clouds with estimated normals
		*/			
		void
		process_clouds (std::vector<PointCloudT::Ptr> views_xyz, std::vector<PointCloud_N::Ptr> &views_N);
		
		/**
		  Processes the rendered view-point-clouds in origianl pose
		  @param views_xyz The unprocessed point clouds 
		  @param views_original_pose_N The processed point clouds with estimated normals in origianl pose
		*/
		void 
		process_clouds_original_pose (	std::vector<PointCloudT::Ptr> views_xyz, 
										std::vector<PointCloud_N::Ptr> &views_original_pose_N, 
										std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > poses,
										std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > cam_pos );
		
		/**
		  Processes the complete model point cloud
		  @param cloud The point cloud of the complete model
		*/
		void
		process_complete_model (PointCloudT::Ptr cloud);
		
		/**
		  Computes the average ESF feature
		  @param vec Vector containing the ESF features
		  @return f_avg The average ESF feature
		*/
		FeatureG
		average_feature (std::vector<FeatureCloudG::Ptr> vec);
		
		/**
		  Estimates global ESF features for the rendered view-point-clouds
		  @param views_original_pose_N The view-point-clouds in original pose 
		  @param features Point cloud containing all the global ESF features
		*/
		void 
		estimate_features_esf (std::vector<PointCloud_N::Ptr> views_original_pose_N, FeatureCloudG::Ptr features);
		
		/**
		  Views the processed point clouds
		  @param views_xyz The original unprocessed point clouds
		  @param views_N The processed point clouds
		*/
		void 
		processed_cloud_viewer (std::vector<PointCloudT::Ptr> views_xyz, std::vector<PointCloud_N::Ptr> views_N);
		
		/**
		  Views the normals of the processed point clouds
		  @param views_N The processed point clouds with normals
		  @param views_original_pose_N The processed point clouds with normals in original pose
		*/
		void 
		normals_viewer (std::vector<PointCloud_N::Ptr> views_N, std::vector<PointCloud_N::Ptr> views_original_pose_N);
		
		/**
		  Merges all rendered view-point-clouds into one complete point cloud
		  @param view_xyz original point clouds
		  @param poses The transformations
		  @param cam_pos The camera positions
		  @param merged_cloud_processed 
		*/
		void
		merge_views (	std::vector<PointCloudT::Ptr> views_xyz, 
						std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > poses, 
						std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > cam_pos,
						PointCloudT::Ptr merged_cloud_processed );
		
		/**
		  Normalizes a vector between 0 and 1
		  @param vec The vector to be normalized
		*/
		void
		normalize (std::vector<float> &vec);
			
		/**
		  Returns the view-utilities for all rendered point clouds
		  @param views_original_pose_N The rendered point clouds in original pose
		  @param utilities The utility vector containing a view-utility value for each view
		*/				
		void 
		get_utilities (std::vector<PointCloud_N::Ptr> views_original_pose_N, std::vector<float> &utilities);
		
		/**
		  Generates a graph using the information from the rendered tesselated sphere
		  @param poses The point cloud transformations
		  @param graph The resulting graph
		*/
		void
		generate_graph (std::vector <Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > poses, View_Graph &graph );

public:
		/**
		  Views the view-graph
		  @param graph The view-graph
		  @param complete_model The complete point cloud model
		*/			
		void
		graph_viewer (View_Graph graph, PointCloudT::Ptr complete_model);
private:
		/**
		  Generates local FPFH features
		  @param views_original_pose_N The view-point-clouds in original pose
		  @param local_features The FPFH features
		*/
		void
		generate_local_features (std::vector<PointCloud_N::Ptr> views_original_pose_N, std::vector<FeatureCloudL::Ptr> &local_features);

        /**
        Generates fusion view local FPFH features
        @param views_original_pose_N The view-point-clouds in original pose
        @param local_features The FPFH features
        */
        void
        generate_fusion_local_features (std::vector<PointCloud_N::Ptr> views_original_pose_N, std::vector<std::vector<FeatureCloudL::Ptr>> &local_features, View_Graph &view_graph);
        /**
         * fuse a view with it's multi-level neighbor
         * @param index view index
         * @param fuse_result object to store the fusion result
         * @param view_graph
         * @param views_original_pose_N multi-view point cloud
         */
        void
        fuse_multi_level_neighbor(std::vector<std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>> &fuse_result,
                                  View_Graph &view_graph, const std::vector<PointCloud_N::Ptr> &views_original_pose_N);
        /**
        * fuse a view with it's neighbor
        * @param index view index
        * @param fuse_result object to store the fusion result
        * @param view_graph
        * @param views_original_pose_N multi-view point cloud
        */
        void
        fuse_neighbor(const int &index, pcl::PointCloud<pcl::PointNormal>::Ptr fuse_result,
                                  View_Graph &view_graph, const std::vector<PointCloud_N::Ptr> &views_original_pose_N,int fusion_level);


};
		
#endif		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
