#ifndef CONFIG_READER_H
#define CONFIG_READER_H

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/foreach.hpp>
#include <string>
#include <set>
#include <exception>
#include <iostream>
#include <mutex>

class Config_Reader
{
public:
    typedef boost::shared_ptr< ::Config_Reader> Ptr;
    typedef boost::shared_ptr< ::Config_Reader const> ConstPtr;
    static Config_Reader* cr;
    static Config_Reader* GetInstance(){
        if(cr == nullptr){
            //!这里写的不熟练
            std::lock_guard<std::mutex> lk(get_object_);
            if(cr == nullptr)
                cr = new Config_Reader;
        }
        return cr;
    }
    // Add_Model
    int resolution;
    int tessellation_level;
    float view_angle;
    bool optimal_view_angle;
    float radius_tessellated_sphere;
    double scale_factor;
    std::string model_name;
    std::string scene_name;
    std::string base_dir;
    float leaf_size;
    double cluster_tolerance;
    int min_cluster_size;
    int max_cluster_size;
    double search_radius_mls;
    int mean_k;
    float std_dev_mul_thresh;
    float bad_normals_threshold;
    bool use_k_search;
    int k_search_normals;
    bool use_radius_search;
    double radius_search_normals;
    bool scale;
    bool largest_cluster_extraction;
    bool downsample;
    bool smooth;
    bool remove_outliers;
    bool view_processed_clouds;
    bool view_normals;
    float normal_magnitude;
    bool view_complete_model;
    bool view_graph;
    bool avg_glb_feature;
    bool use_delay;
    int fusion_level;

    //Descriptors
    std::string local_descriptors_type_;
    std::string global_descriptors_type_;

    // System
    std::string save_path;
    int pose_visualization_mode;
    int pose_print_mode;
    float hint_view_weight;
    float hint_normal_weight;
    float hint_feature_weight;
    float hint_distinguish_weight;
    double hint_below_plane_threshold;
    int hint_misalignment_angle_threshold;
    bool hint_view_search_results;
    bool hint_normalize_search_results;
    bool hint_print_info;
    bool rdf_enabled;
    std::string rdf_url;
    std::string rdf_repo_name;
    bool use_centered_cluster;
    double centered_threshold;
    int max_id_views;
    int max_pose_views;
    int max_merged_views;
    bool visualize_merged_views;
    std::string web_service_username;
    std::string web_service_password;
    std::string hand_web_service_url;

    // Pose
    int pose_max_iterations;
    double pose_max_correspondence_distance;
    double pose_correspondence_randomness;
    double pose_similarity_threshold;
    double pose_inlier_fraction;
    double pose_inverse_inlier_fraction;
    double pose_feature_radius_search;
    int visualization_mode;
    int print_mode;

    // Segmentation
    double segmentation_leaf_size;
    double segmentation_plane_distance_threshold;
    int segmentation_plane_max_iterations;
    double segmentation_background_segmentation_distance;
    double segmentation_normal_radius_search;
    double segmentation_cluster_tolerance;
    int segmentation_cluster_min_size;
    int segmentation_cluster_max_size;

    // Visualization
    int normal_level;
    double point_size;

    // Verbose
    bool visual;
    bool verbose;

    // path
    std::string report_path;
    std::string pure_GA_path;
    std::string pure_icp_path;
    std::string fine_icp_path;
    std::string GA_icp_path;
    std::string time_path;
    std::string cad_models_path;
    std::string model_data_path;
    std::string scene_data_path;

    //GA
    double range_scale_t_;
    double range_scale_euler_;
    double cluster_radius;
    int population;
    int generation_max;
    int best_stall_max;
    int average_stall_max;
    double tol_stall_average;
    double tol_stall_best;
    float crossover_fraction;
    int elite_count;
    float mutation_rate;
    bool multi_threading;
    bool dynamic_threading;
    std::string mutation_method;
    std::string fitness_method;
    int fusion_level_ga;
    std::string fusion_mode;
    float F0;
    bool set_threads;

    //view
    int window_width;
    int window_height;
    std::string color_style;

    /**
      Loads all parameters associated with adding a new model to the system (Rendering views)
      @param filename The name of the config file (.ini)
    */
    void
    add_model_load_config (const std::string &filename);

    /**
    Loads all parameters associated with descriptors
    @param filename The name of the config file (.ini)
    */
    void
    add_descriptors_config (const std::string &filename);

    /**
      Loads all parameters associated with visualization
      @param filename The name of the config file (.ini)
    */
    void
    add_visualization_config (const std::string &filename);
    /**
        Loads all parameters associated with the system
        @param filename The name of the config file (.ini)
     */
    void
    system_load_config (const std::string &filename);

    /**
        Loads all parameters associated with the librealsense_capturer
        @param filename The name of the config file (.ini)
     */
    void
    capturer_load_config (const std::string &filename);

    /**
        Loads all parameters associated with the background_creator
        @param filename The name of the config file (.ini)
     */
    void
    background_load_config (const std::string &filename);
    void add_verbose_config (const std::string &filename);
    void add_path_config(const std::string &filename);
    void add_GA_parameters(const std::string &filename);
    void add_view_config(const std::string &filename);

private:
    static std::mutex get_object_;
    Config_Reader(){std::cout<<"config reader construction"<<std::endl;};
    //程序结束销毁单例的机制
    class GC{
    public:
        GC(){
            std::cout<<"Config_Reader GC construction"<<std::endl;
        }
        ~GC(){
            std::cout<<"Config_Reader GC deconstruction"<<std::endl;
            if(cr != nullptr)
            {
                delete cr;
                cr = nullptr;
                std::cout<<"config reader deconstruction"<<std::endl;
            }
        }

    };
    static GC gc;
};


#endif
