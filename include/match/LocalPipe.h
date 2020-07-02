//
// Created by wang on 19-9-10.
//

#ifndef LOCALPIPELINE_LOCAL_PIPE_H
#define LOCALPIPELINE_LOCAL_PIPE_H

#include <iostream>
//#include "common_typedef.h"
#include <TdLibrary/td_eigen/eigen_common_typedef.h>
#include <TdLibrary/td_eigen/eigen_common_typedef.h>
#include <string>
#include <sophus/se3.hpp>
#include "sample_consensus_prerejective.h"
#include "config_reader.h"
#include "Optimization.h"

#include <TdLibrary/tool/random_tool.hpp>
#include <TdLibrary/tool/random_tool.h>
#include <TdLibrary/openGA/src/openga.hpp>
#include "ModelDataStructure.h"
#include "SceneDataStructure.h"
#include <pcl/registration/icp.h>
#include <TdLibrary/PCL/icp_related.h>
#include "parameters.h"
//td::pclib::PointNCloudPtr getOutlierCloud(td::pclib::PointNCloudPtr input_source, pcl::search::KdTree<pcl::PointNormal> target_cloud_kdtree_,
//                                          double inlier_squared_threshold,Eigen::Matrix4d tg_T_sr);
#include "Result.h"
class LocalPipe {

public:
    LocalPipe();

    void TransferData(ModelData::Ptr target_data, SceneData::Ptr source_data, int view_candidate_index){
        view_candidate_index_ = view_candidate_index;
        target_data_ = target_data;
        source_data_ = source_data;
        input_target_ = target_data_->cloud_N_original_pose_vec[view_candidate_index];
//        input_source_ = source_data_->cloud_with_normal;


    }
    /**
     * compute the local descriptors
     * @param cloud_with_normal
     * @param descriptors_ptr
     * @param feature_radius_search
     */
    void DescriptorsTrain(const td::pclib::PointNCloudPtr cloud_with_normal, td::pclib::FpfhDescriptorCloudPtr &descriptors_ptr, const double feature_radius_search);

    void EstimatePose(td::pclib::PointNCloudPtr source_aligned);

    void OptimizationGa();
    void OptimizationGaMutation();
    void ShowRangeOfGaParameter();
    void SetFusionPartialCloud(td::pclib::PointNCloudPtr fusion_partial_source){
        fusion_partial_model_ = fusion_partial_source;
    }
    void SetFusionPartialCloud();
    void DynamicFusion();
    bool CheckResult(const Eigen::Matrix4d &final_result);
//    void OutputResult(const Result::Ptr result, std::string file_path);
//    void OutputResult(const Result &result, std::string file_path);
    void GetResult(Result::Ptr &icp,Result::Ptr &GA, Result::Ptr &GA_ICP){
//        std::cout<<"this views final fitness = "<<final_result_pure_GA_->final_fitness<<std::endl;
        GA = std::move(final_result_pure_GA_);
        GA_ICP = std::move(final_result_GA_icp_);
        icp = std::move(final_result_icp_);
    }
    //openGA
//    bool eval_solution (const MySolution &p, MyMiddleCost &c);
//    MySolution mutate(
//            const MySolution& X_base,
//            const std::function<double(void)> &rnd01,
//            double shrink_scale);
//    void init_genes(MySolution& p,const std::function<double(void)> &rnd01);
    size_t get_genes_range_size() const {
        return genes_range_.size();
    }
private:
    void set_descriptor_type(std::string descriptor_type);
    void set_GA_parameters();
    enum DescriptorType_ {
        fpfh
    };
    void ShowCandidate();
    void CandidateClustering();
    void ShowFilteredCandidate();
    void AlignerToGACandidate(int indices);//transfer the AlignerClusters's data to CandidateCluster
    /**
     * process the  ICP
     * @param guess_and_result object for storing the initial guess transformations and the result
     * @param source is the point cloud to be registered to the target.
     * @param target is target point cloud
     * @return whether icp converge
     */
    bool Icp(Eigen::Matrix4d &guess_and_result, td::pclib::PointNCloudPtr source, td::pclib::PointNCloudPtr target,double cor_distance,int num_iteration, double transformation_epsilon, double euclidean_fitness_epsilon, int num_ransac, bool verbose);
//    void TrimmedIcp(td::VecMat4 guess_and_result, float init_ratio, td::pclib::PointNCloudPtr source, td::pclib::PointNCloudPtr target);
    void RunIcp();
    void FineIcp();
    void FineIcp(Result::Ptr pure_GA);
    void LoadCheckResult();
private:
    DescriptorType_ descriptor_type_; //local descriptor type
    Config_Reader* cr_;
    //openGA

//    td::pclib::PointNCloudPtr input_source_;
    pcl::search::KdTree<pcl::PointNormal> source_cloud_kdtree_;
    pcl::search::KdTree<pcl::PointNormal> target_cloud_kdtree_;
    td::pclib::PointNCloudPtr input_target_;
    std::vector<td::pclib::PointNCloudPtr> fusion_partial_model_vec; // array to store each candidate_clustering's fusion partial point cloud
    std::vector<pcl::search::KdTree<pcl::PointNormal>> fusion_partial_model_vec_kdtree_;
    td::pclib::PointNCloudPtr fusion_partial_model_;
    enum FusionMode{
        no_fusion,
        dynamic_fusion,
        static_fusion,
        complete_model
    };
    FusionMode fusion_mode_;
    ModelData::Ptr target_data_;
    SceneData::Ptr source_data_;
    int view_candidate_index_;

    td::VecMat4 vec_o_T_s1_;

//    vec_pair_Matrix4d_double first_two_transformation_;
    pcl::SampleConsensusPrerejective<td::pclib::PointN, td::pclib::PointN, td::pclib::FpfhDescriptor, double> aligner_;
    td::pclib::PointCloudPtr candidate_euler_angles_cloud_;
    td::VecMat4 candidate_tg_T_sr_;
    std::vector<double> candidate_errors_;
    std::vector<double> candidate_inlier_fraction_;

    // clustering
    double cluster_radius_;
    double default_half_range;
    std::vector<pcl::PointIndices> cluster_indices_;//Array to store each cluster's indice in candidate_euler_angles_cloud_ or candidate_tg_T_sr_
    struct AlignerClusters{
        std::vector<td::pclib::PointCloudPtr> vec_aligner_euler_cluster;
        std::vector<td::VecMat4> vec_aligner_transformations;
        std::vector<int> min_error_indices;
        std::vector<int> max_inlier_indices;
        std::vector<double> mean_error;
    };
    struct CandidateCluster{
        std::vector<pcl::PointIndices> candidate_clusters_indices;// Array to store the indices of each cluster's every pose in candidate_euler_angles_cloud_ or candidate_tg_T_sr_
        std::vector<int> min_error_indices;// Array to store the indices of each cluster's min-final_fitness pose in candidate_euler_angles_cloud_ or candidate_tg_T_sr_
        std::vector<td::pclib::PointCloudPtr> vec_candidate_euler_cluster;// Array to store the euler anlge cloud of each cluster
        std::vector<td::VecMat4> vec_candidate_transformations;// Array to store the transformations of each cluster
        std::vector<int> max_inlier_indices; // Array to store the indices of each cluster's max inlier fraction pose in candidate_inlier_fraction_
        std::vector<double> mean_error; // Array to store the mean final_fitness of each cluster
    };
    AlignerClusters aligner_clusters_;
    CandidateCluster candidate_clusters_;

    int GA_size;
    bool set_threads_;
    std::vector<std::vector<MySolution>> init_genes_;
    double outlier_fitness_;
    std::vector<std::vector<std::vector<double>>> genes_range_; //each_cluster<range<min,max>>
//    Eigen::Matrix4d o1_T_o2_;
//    Eigen::Vector3d o1_euler_angle_o2_;
    struct ResultGA{
        MySolution result;
        double fitness;
    };

    std::vector<ResultGA> results_GA_;
    std::vector<double> range_scale_;
    Result::Ptr final_result_pure_GA_;
    Result::Ptr final_result_icp_;
    Result::Ptr final_result_GA_icp_;
    td::VecMat4 check_results_;
    std::string pure_GA_path_;
    std::string fine_icp_path_;
    std::string pure_icp_path_;
    std::string GA_icp_path_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW




};


#endif //LOCALPIPELINE_LOCAL_PIPE_H
