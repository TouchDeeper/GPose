#ifndef ACCESS_MODEL_DATA_H_
#define ACCESS_MODEL_DATA_H_

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/esf.h>
#include <pcl/filters/voxel_grid.h>
#include <parameters.h>


class Access_Model_Data
{
  	private:
		typedef pcl::PointXYZ PointT;
		typedef pcl::PointNormal Point_N;
		typedef pcl::PointXYZRGB PointT_color;
		typedef pcl::PointCloud<PointT> PointCloudT;
		typedef pcl::PointCloud<Point_N> PointCloud_N;
		typedef pcl::PointCloud<PointT_color> PointCloudT_color;
		typedef pcl::ESFSignature640 FeatureG;
		typedef pcl::FPFHSignature33 FeatureL;
		typedef pcl::PointCloud<FeatureG> FeatureCloudG;
		typedef pcl::PointCloud<FeatureL> FeatureCloudL;

private:
    Config_Reader* cr;

    static Access_Model_Data* amd;
    static std::mutex lock_ac;
    Access_Model_Data(){
        std::cout<<"Access_Model_Data construction"<<std::endl;
        cr = Config_Reader::GetInstance();
    }
    class GC{
    public:
        GC(){
            std::cout<<"Access_Model_Data GC construction"<<std::endl;
        }
        ~GC(){
            std::cout<<"Access_Model_Data GC deconstruction"<<std::endl;
            if(amd != nullptr)
            {
                delete amd;
                amd = nullptr;
                std::cout<<"Access_Model_Data deconstruction"<<std::endl;
            }
        }
    };
    static GC gc;
  	public:
        static Access_Model_Data* GetInstance(){
            if(amd == nullptr)
            {
                std::lock_guard<std::mutex> lk(lock_ac);
                if(amd == nullptr)
                    amd = new Access_Model_Data;
            }
            return amd;
        }
	  	/**
		  Returns the path to masters_thesis
		*/
		boost::filesystem::path 
		path_to_root (void);

		/**
		  Returns the path to Data in /masters_thesis
		*/
		boost::filesystem::path
		path_to_data (void);

		/**
		 *
		 * @return the path to the scene
		 */
        boost::filesystem::path
        path_to_scene_data(void);
		/**
		  Returns the path to Model_data in /Data
		*/
		boost::filesystem::path
		path_to_model_data (void);

		/**
		  Returns the path to model in /Model_data
		*/
		boost::filesystem::path
		path_to_model_in_model_data (std::string model);
		
		/**
		  Returns the path to CAD_models folder
		*/
		boost::filesystem::path
		path_to_cad_models ();

		/**
		  Returns the path to model in /CAD_models
		*/
		std::string 
		path_to_model_in_CAD_models (std::string model);

		/**
		  Saves the view-clouds
		  @param model_name The model name
		  @param views_N Vector containing the view clouds
		  @param views_original_pose Vector containing the view clouds in original CAD-pose
		  @param complete_model Complete model obtained by merging all the views in original pose
		*/
		void
		save_view_clouds (	std::string model_name,
							std::vector<PointCloud_N::Ptr> views_N,
							std::vector<PointCloud_N::Ptr> views_original_pose,
							PointCloudT::Ptr complete_model );
        /**
        Saves the fusion view-clouds
        @param model_name The model name
        @param fuse_results Vector containing the multi-level fusion view
        */
        void
        save_fusion_view_clouds (	std::string model_name,
                                                        std::vector<std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>> &fuse_results);

		/**
		  Saves the view-utilities for each view. The utility is a measure of the visibility of the object in each view.
		  @param model_name The model name
		  @param utilities Vector containing the view-utilities
		*/
		void
		save_view_utilities (std::string model_name, std::vector<float> utilities);

		/**
		  Saves the global feature for each view.
		  @param model_name The model name
		  @param global_features Point cloud containing the global features
		*/
		void
		save_global_features (std::string model_name, FeatureCloudG::Ptr global_features);

		/**
		  Adds the view-clouds in /views to (@param views)
		  @param model_name The model name
		  @param views Empty vector. The view-clouds will be loaded and stored in views
		  @param indices Vector containing the view indices to be loaded. If empty then all views will be loaded
		*/
		void
		load_model_views (std::string model_name, std::vector<PointCloud_N::Ptr> &views, std::vector<int> indices);

        /**
        Adds the fusion view-clouds in /views to (@param views)
        @param model_name The model name
        @param views Empty vector. The view-clouds will be loaded and stored in views
        @param indices Vector containing the view indices to be loaded. If empty then all views will be loaded
        @param level the level need to load
        */
        void
        load_fusion_model_views (std::string model_name, std::vector<PointCloud_N::Ptr> &views, std::vector<int> indices, int level);
		/**
		  Adds the view-clouds in /views_original_pose to (@param views)
		  @param model_name The model name
		  @param views Empty vector. The view-clouds will be loaded and stored in views
		  @param indices Vector containing the view indices to be loaded. If empty then all views will be loaded
		*/
		void
		load_model_views_original_pose (std::string model_name, std::vector<PointCloud_N::Ptr> &views, std::vector<int> indices);

		/**
		  Returns a vector of all models in Model_data
		*/
		std::vector<std::string>
		get_model_names (void);

		/**
		  Loads the global features for a given model. The global features are ESF features!
		  @param model The name of the model
		  @param models Empty point cloud. Adds the global features in model
		*/
		void
		load_global_features (std::string model, FeatureCloudG::Ptr features);

		/**
		  Loads the complete/merged point cloud model from the rendered views. Note that the complete model has some offset when merging the rendered views
		  @param model The name of the model
		  @param models Empty point cloud. Adds the complete point cloud model
		*/
		void
		load_complete_model (std::string model, PointCloudT::Ptr cloud);

		/**
		  Saves the distinguish-utility
		  @param source_name The name of the model
		  @param target_name The name of the target (matching) model
		  @param similar_views Vector containing the matching scores
		*/
		void
		save_similar_model_data (std::string source_name, std::string target_name, std::vector<float> similar_views);

		/**
		  Saves the feature utilities
		  @param model_name The name of the model
		  @param utilities The feature utilities
		*/
		void
		save_feature_utilities (std::string model_name, std::vector<double> utilities);

		/**
		  Loads the view utilities
		  @param model_name the name of the model
		  @return a vector containing all view-utilities
		*/
		std::vector<float>
		load_view_utilities (std::string model_name);

		/**
		  Loads the feature utilities
		  @param model_name the name of the model
		  @return a vector containing all feature-utilities
		*/
		std::vector<float>
		load_feature_utilities (std::string model_name);

		/**
		  Loads the distinguish-utility
		  @param model_name the name of the model
		  @param similar_model_name the name of the similar model
		  @return a vector containing the distinguish-utilities for all similar models
		*/
		std::vector<std::vector<float> >
		load_distinguish_utilities (std::string model_name, std::vector<std::string> similar_models);
		
		/**
		Loads the normal-utilities
		@param model_name the name of the model
		@return a vector containing all normal-utilities
		*/
		std::vector<float>
		load_normal_utilities (std::string model_name);
		
		/**
		  Loads the views in indices and merges the result
		  @param model_name The name of the model
		  @param indices The indices of the views
		  @param merged_cloud The merged cloud
		*/
		void 
		load_merged_views (std::string model_name, std::vector<int> indices, PointCloud_N::Ptr merged_cloud);
		
		/**
		  Returns a merged point cloud of all the point clouds in views 
		  @param views Vector containing all the point clouds
		  @param merged_cloud The merged cloud
		*/
		void
		merge_views (std::vector<PointCloud_N::Ptr> views, PointCloud_N::Ptr merged_cloud);
		
		/**
		  Saves the local features for all views 
		  @param model_name The model name
		  @param features The local feature clouds
		*/
		void 
		save_local_features (std::string model_name, std::vector<FeatureCloudL::Ptr> features);

        /**
        Saves the fusion local features for all views
        @param model_name The model name
        @param features The local feature clouds
        */
        void
        save_fusion_local_features (std::string model_name, std::vector<std::vector<FeatureCloudL::Ptr>> features);
		
		/**
		  Adds the local features for the view-clouds in /views_original_pose 
		  @param model_name The model name
		  @param local_features Empty vector. The local features for the view-clouds will be loaded and stored in local_features 
		  @param indices Vector containing the view indices to be loaded. If empty then all views will be loaded
		*/
		void 
		load_local_features (std::string model_name, std::vector<FeatureCloudL::Ptr> &local_features, std::vector<int> indices);

};

#endif
