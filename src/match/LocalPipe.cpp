//
// Created by wang on 19-9-10.
//

//#include "include/match/LocalPipe.h"
#include <TdLibrary/PCL/local_features.h>
#include <TdLibrary/PCL/Viewer.h>
#include "parameters.h"
#include <TdLibrary/PCL/tools.h>
#include <TdLibrary/PCL/filter.h>
#include <TdLibrary/slam_tool/motion_transformation.h>
#include <TdLibrary/tool/tic_toc.h>
#include <TdLibrary/FileOperation/FileOperation.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <omp.h>
#include <match/LocalPipe.h>
#include "Optimization.h"
//#include <pcl/recognition/ransac_based/auxiliary.h>
//#include <pcl/recognition/ransac_based/trimmed_icp.h>
/**
 * get outlier cloud after the souce be registered to the target
 * @param input_source source cloud
 * @param target_cloud_kdtree_
 * @param inlier_squared_threshold squared distance threshold for inlier check
 * @param tg_T_sr transformation from source to target
 * @return outlier_cloud ptr
 */
//td::pclib::PointNCloudPtr getOutlierCloud(td::pclib::PointNCloudPtr input_source, pcl::search::KdTree<pcl::PointNormal> target_cloud_kdtree_,
//                               double inlier_squared_threshold,Eigen::Matrix4d tg_T_sr) {
//    td::pclib::PointNCloudPtr outlier_cloud(new td::pclib::PointNCloud);
//    // Initialize variables
//    std::vector<int> inliers;
//    inliers.reserve (input_source->size ());
////            fitness_score = 0.0f;
//
//    // Transform the input dataset using the population transformation
//    td::pclib::PointNCloudPtr source_transformed(new td::pclib::PointNCloud);
////    source_transformed->resize (target_cloud_->size ());
//    pcl::transformPointCloud (*input_source, *source_transformed, tg_T_sr);
//
//    // For each point in the source dataset
//    for (size_t i = 0; i < source_transformed->points.size (); ++i)
//    {
//        // Find its nearest neighbor in the target
//        std::vector<int> nn_indices (1);
//        std::vector<float> nn_dists (1);
//        target_cloud_kdtree_.nearestKSearch (source_transformed->points[i], 1, nn_indices, nn_dists);
//
//        // Check if point is an inlier
//        if (nn_dists[0] < inlier_squared_threshold)
//        {
//            // Update inliers
//            inliers.push_back (static_cast<int> (i));
//
//            // Update fitness score
////                    fitness_score += nn_dists[0];
//        } else
//            outlier_cloud->push_back(input_source->points[i]);
//
//    }
//
//    // Calculate MSE
////            if (inliers.size () > 0)
////                fitness_score /= static_cast<float> (inliers.size ());
////            else
////                fitness_score = std::numeric_limits<float>::max ();
////            inlier_fraction = static_cast<float> (inliers.size ()) / static_cast<float> (input_source->size ());
////            inlier_size = inliers.size();
//    return outlier_cloud;
//
//}
/**
 * compute the local descriptors
 * @param cloud_with_normal
 * @param descriptors_ptr
 * @param feature_radius_search
 */
void LocalPipe::DescriptorsTrain(const td::pclib::PointNCloudPtr cloud_with_normal, td::pclib::FpfhDescriptorCloudPtr &descriptors_ptr, const double feature_radius_search){
    if(descriptor_type_ == DescriptorType_::fpfh)
    {
        std::cout<<"estimate the"<< typeid(td::pclib::FpfhDescriptor).name() << "descriptor"<<std::endl;

        td::pclib::LocalDescriptorEstimation(cloud_with_normal, descriptors_ptr, feature_radius_search);
    }
}

void LocalPipe::set_descriptor_type(std::string descriptor_type = "fpfh"){
    if(descriptor_type == "fpfh")
        descriptor_type_ = DescriptorType_::fpfh;
}
void LocalPipe::EstimatePose(td::pclib::PointNCloudPtr source_aligned){

    aligner_.SetBoundary(source_data_->non_boundaries_points, source_data_->non_boundaries_indices);
    aligner_.setInputSource(source_data_->cloud_with_normal);
    aligner_.setSourceFeatures(source_data_->fpfh_descriptors_ptr);
    aligner_.setInputTarget(input_target_);
    aligner_.setTargetFeatures(target_data_->fpfh_descriptors_vector[view_candidate_index_]);
    aligner_.setMaximumIterations(cr_->pose_max_iterations);
    aligner_.setNumberOfSamples(3);
    aligner_.setCorrespondenceRandomness(cr_->pose_correspondence_randomness);
    aligner_.setSimilarityThreshold(cr_->pose_similarity_threshold);
    aligner_.setMaxCorrespondenceDistance(cr_->pose_max_correspondence_distance);
    aligner_.setInlierFraction(cr_->pose_inlier_fraction);
    aligner_.setNumberOfTransformations(10);

    if(VISUAL)
    {
        pcl::ScopeTime t("\t Alignment");
        aligner_.align (*source_aligned);
    } else
        aligner_.align (*source_aligned);


//    std::vector<double> inlier_ratio;

    std::vector<std::vector<std::vector<int>>> correspondence;
    aligner_.getTransformations(candidate_tg_T_sr_,candidate_inlier_fraction_, correspondence, candidate_errors_);
//    aligner_.getCorrespondences(&correspondence);
//    aligner_.getErrorsAndInlierFraction(candidate_errors_,candidate_inlier_fraction_);

//    Eigen::Matrix4d transformation;
//    transformation = aligner_.getFinalTransformation();
//    std::cout<<"final transformation\n"<<transformation<<std::endl;

    if(aligner_.hasConverged()) {
        if(VISUAL)
        {
            pcl::console::print_info("\t Alignment successful!\n");
            pcl::console::print_info ("Inliers: %i/%i\n", aligner_.getInliers ().size (), source_data_->cloud_with_normal->size ());
            td::pclib::Viewer view("aligner pose", WINDOW_SIZE);
            for(size_t i = 0; i < correspondence.size(); i++)
            {
                Eigen::Matrix4d transformation_temp = candidate_tg_T_sr_[i];
                view.ShowCorrespondence(correspondence[i],source_data_->cloud_with_normal, input_target_, transformation_temp,VIEWSTYLE.model_color,VIEWSTYLE.scene_color,VIEWSTYLE.line_rgb,VIEWSTYLE.point_size,VIEWSTYLE.background_color);
                view.ClearView();
            }
        }
    } else {
        if(VISUAL)
            pcl::console::print_info("\t Alignment failed!\n");
    }

    ShowCandidate();
    CandidateClustering();

    for (int m = 0; m < candidate_clusters_.vec_candidate_euler_cluster.size(); ++m) {
        int  min_error_indices = candidate_clusters_.min_error_indices[m];
        Eigen::Matrix4d o_T_s1 = candidate_tg_T_sr_[min_error_indices];
        vec_o_T_s1_.push_back(o_T_s1);
    }

    if(VISUAL)
        ShowFilteredCandidate();
    SetFusionPartialCloud();


    RunIcp();


//    TrimmedIcp(vec_o_T_s1_,0.8f,source_data_->cloud_with_normal,input_target);

    OptimizationGa();

    //run three mutation method separately
//    OptimizationGaMutation();
    FineIcp();

}

void LocalPipe::FineIcp()
{
    std::cout<<"****** Fine ICP ******"<<std::endl;
    td::TicToc timer_fine_icp;
    final_result_GA_icp_->pose = final_result_pure_GA_->pose;
    bool converged = td::pclib::Icp(final_result_GA_icp_->pose,source_data_->cloud_with_normal, final_result_pure_GA_->fusion_partial_cloud, cr_->pose_max_correspondence_distance, 1000, 1e-10, 1e-10, 40, VERBOSE);
    final_result_GA_icp_->solve_time = timer_fine_icp.tos();
    std::cout<<"fine icp take "<<timer_fine_icp.tos()<<" s"<<std::endl;
    final_result_GA_icp_->solve_time += final_result_pure_GA_->solve_time;

    double inlier_fraction;
    float error;
    int inlier_size;
    aligner_.getFitness_ga(inlier_fraction,inlier_size,error,final_result_GA_icp_->pose);
    final_result_GA_icp_->final_error = 1000 * error;
    final_result_GA_icp_->final_inlier_fraction = inlier_fraction;
    final_result_GA_icp_->init_error = final_result_pure_GA_->init_error;
    final_result_GA_icp_->init_inlier_fraction = final_result_pure_GA_->init_inlier_fraction;
    final_result_GA_icp_->compute_error_decline_fraction();
    if(LOCAL_FLAG){
        std::ofstream fine_icp_output;
        fine_icp_output.open(fine_icp_path_,fstream::app | fstream::out);
        OutputResult(final_result_GA_icp_,GA_icp_path_);
    }


    if(VISUAL){
        std::string viewer_name = "ICP fine align";
        td::pclib::ShowBeforeAndAfterAlign(source_data_->cloud_with_normal,final_result_pure_GA_->fusion_partial_cloud,final_result_pure_GA_->pose,final_result_GA_icp_->pose, viewer_name,VIEWSTYLE.model_color,VIEWSTYLE.scene_color,WINDOW_SIZE,VIEWSTYLE.point_size,VIEWSTYLE.background_color);
    }

}

void LocalPipe::RunIcp()
{
    std::cout<<"-------- ICP ---------"<<std::endl;
    td::TicToc timer_icp;
    std::vector<bool> vec_converged;
    td::VecMat4 guess_and_result = vec_o_T_s1_;
    td::TicToc timer;
    for (int i = 0; i < vec_o_T_s1_.size(); ++i) {
        bool converged = td::pclib::Icp(guess_and_result[i],source_data_->cloud_with_normal, fusion_partial_model_vec[i], cr_->pose_max_correspondence_distance, 1000, 1e-10, 1e-10, 40, VERBOSE);
        if(converged)
            vec_converged.push_back(true);
        else
            vec_converged.push_back(false);
    }

    double inlier_squared_threshold = cr_->pose_max_correspondence_distance * cr_->pose_max_correspondence_distance;
    double min_fitness_score = 300;
    int final_result_index = -1;
    for (int k = 0; k < guess_and_result.size(); ++k) {
        double inlier_fraction;
        int inlier_size;
        float error;
        td::pclib::getFitness(source_data_->cloud_with_normal, fusion_partial_model_vec_kdtree_[k],inlier_squared_threshold, inlier_fraction, inlier_size, error, guess_and_result[k] );

        double fitness_score = pose_fitness(inlier_fraction, error);
//        std::cout<<"inlier_fractioin = "<<inlier_fraction<<"  error = "<<error<<std::endl;
//        std::cout<<"fitness_score = "<<fitness_score<<std::endl;
        if(fitness_score < min_fitness_score)
        {
            final_result_index = k;
            min_fitness_score = fitness_score;
        }
    }
    std::cout<<"ICP take "<<timer.toc()<<" ms"<<std::endl;

    if(VISUAL)
    {
        for (int j = 0; j < vec_converged.size(); ++j) {

            std::string viewer_name = "icp align result"+std::to_string(j);
            td::pclib::ShowBeforeAndAfterAlign(source_data_->cloud_with_normal,fusion_partial_model_vec[j],vec_o_T_s1_[j],guess_and_result[j], viewer_name,VIEWSTYLE.model_color,VIEWSTYLE.scene_color,WINDOW_SIZE,VIEWSTYLE.point_size,VIEWSTYLE.background_color);

        }
    }

    double inlier_fraction;
    float error;
    int inlier_size;
    Eigen::Matrix4d best_tg_T_sr = guess_and_result[final_result_index];
    aligner_.getFitness_ga(inlier_fraction,inlier_size,error,best_tg_T_sr);
    final_result_icp_->pose = best_tg_T_sr;
    final_result_icp_->final_error = 1000 * error;
    final_result_icp_->final_inlier_fraction = inlier_fraction;
    int  min_error_indices = candidate_clusters_.min_error_indices[final_result_index];
    double init_min_error = 1000 * candidate_errors_[min_error_indices];
    final_result_icp_->init_error = init_min_error;//存储每个GA手动设置的初始种群的min_fitness_score
    final_result_icp_->fusion_partial_cloud = fusion_partial_model_vec[final_result_index];
    final_result_icp_->init_inlier_fraction = candidate_inlier_fraction_[min_error_indices];
    final_result_icp_->compute_error_decline_fraction();
    final_result_icp_->solve_time = timer_icp.tos();
    std::cout<<"ICP take "<<timer_icp.tos()<<" s"<<std::endl;
    if(LOCAL_FLAG)
        OutputResult(final_result_icp_,pure_icp_path_);
    if(VERBOSE)
    {
        std::cout<<"before icp, inlier_fraction = "<<final_result_icp_->init_inlier_fraction<<"  error = "<<final_result_icp_->init_error<<std::endl;
        std::cout<<"after icp, inlier_fraction = "<<final_result_icp_->final_inlier_fraction<<"  error = "<<final_result_icp_->final_error<<std::endl;
    }


    if(VISUAL)
    {
        std::string viewer_name = "pure icp final result";
        td::pclib::ShowBeforeAndAfterAlign(source_data_->cloud_with_normal,final_result_icp_->fusion_partial_cloud,vec_o_T_s1_[final_result_index],final_result_icp_->pose, viewer_name,VIEWSTYLE.model_color,VIEWSTYLE.scene_color,WINDOW_SIZE,VIEWSTYLE.point_size,VIEWSTYLE.background_color);
    }


}
//void LocalPipe::TrimmedIcp(td::VecMat4 guess_and_result, float init_ratio, td::pclib::PointNCloudPtr source, td::pclib::PointNCloudPtr target){
//    pcl::recognition::TrimmedICP<pcl::PointNormal,double> trimmed_icp;
//    trimmed_icp.init(target);
//    td::VecMat4 result = guess_and_result;
//    int num_source_points_to_use = int(init_ratio*source->size());
//    std::cout<<"-------- Trimmed ICP ---------"<<std::endl;
//    for (int i = 0; i < guess_and_result.size(); ++i) {
////        std::cout<<"in candidate pose "<<i<<std::endl<<guess_and_result[i]<<std::endl;
//        trimmed_icp.align(*source, num_source_points_to_use, result[i]);
//        std::string viewer_name = "trimmed icp align result"+std::to_string(i);
//        td::pclib::ShowBeforeAndAfterAlign(source,target,guess_and_result[i],result[i], viewer_name,VIEWSTYLE.model_color,VIEWSTYLE.scene_color,WINDOW_SIZE,VIEWSTYLE.point_size,VIEWSTYLE.background_color);
//
////        std::cout<<"the pose after trimmed icp is\n"<<guess_and_result[i]<<std::endl;
//    }
//
//}
void LocalPipe::SetFusionPartialCloud() {

    source_cloud_kdtree_.setInputCloud(source_data_->cloud_with_normal);
    switch (fusion_mode_){
        case FusionMode::dynamic_fusion:
            DynamicFusion();
            break;
        case FusionMode::static_fusion:
            fusion_partial_model_ = target_data_->fusion_cloud[view_candidate_index_];
            fusion_partial_model_vec.resize(candidate_clusters_.candidate_clusters_indices.size(), fusion_partial_model_);
            break;
        case FusionMode::complete_model:
            pcl::copyPointCloud(*target_data_->complete_model, *fusion_partial_model_);
//            fusion_partial_model_ = target_data_->complete_model;
            fusion_partial_model_vec.resize(candidate_clusters_.candidate_clusters_indices.size(), fusion_partial_model_);
            break;
        case FusionMode::no_fusion:
            std::cout<<"no_fusion mode"<<std::endl;
            fusion_partial_model_vec.resize(candidate_clusters_.candidate_clusters_indices.size(), input_target_);
            break;
    }
    td::TicToc timer;
    for (int l = 0; l < fusion_partial_model_vec.size(); ++l) {
        pcl::search::KdTree<pcl::PointNormal> kdtree;
        kdtree.setInputCloud(fusion_partial_model_vec[l]);
        fusion_partial_model_vec_kdtree_.push_back(kdtree);
    }
    assert(fusion_partial_model_vec.size() == fusion_partial_model_vec_kdtree_.size());
    std::cout<<"train fusion partial model take "<<timer.toc()<<" ms"<<std::endl;
    if(VISUAL)
    {
        for (int i = 0; i < fusion_partial_model_vec.size(); ++i) {
            std::string viewer_name = "fusion partial model" + std::to_string(i);
            pcl::visualization::PCLVisualizer viewer(viewer_name);
            if(!(WINDOW_SIZE[0] == 0 || WINDOW_SIZE[1] == 0))
                viewer.setSize(WINDOW_SIZE[0],WINDOW_SIZE[1]);
            viewer.setBackgroundColor(VIEWSTYLE.background_color[0],VIEWSTYLE.background_color[1],VIEWSTYLE.background_color[2]);
            td::pclib::ColorHandlerPointN model_color = VIEWSTYLE.model_color;
            model_color.setInputCloud(fusion_partial_model_vec[i]);
            viewer.addPointCloud<td::pclib::PointN>(fusion_partial_model_vec[i],model_color,"fusion_target");
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, VIEWSTYLE.point_size, "fusion_target");

            viewer.spin();
        }

    }

}
void LocalPipe::DynamicFusion(){
    double cor_dist_threshold = aligner_.getMaxCorrespondenceDistance();
    double inlier_squared_threshold = cor_dist_threshold*cor_dist_threshold;
    target_cloud_kdtree_.setInputCloud(input_target_);

    td::TicToc timer;
    for (int i = 0; i < candidate_clusters_.candidate_clusters_indices.size(); ++i) {
        Eigen::Matrix4d tg_T_sr = candidate_tg_T_sr_[candidate_clusters_.min_error_indices[i]];
        td::pclib::PointNCloudPtr source_outlier = td::pclib::getOutlierCloud(source_data_->cloud_with_normal,target_cloud_kdtree_,inlier_squared_threshold,tg_T_sr);
        std::cout<<"source outlier size = "<<source_outlier->size()<<std::endl;
        //level1 fusion
        auto neighbor_cloud_indexs = target_data_->view_graph->get_neighbors(view_candidate_index_);
        double max_inlier_size_increas = 0;
        double max_new_inlier_size = 0;
        int neighbor_index_max_inlier_increase = -1;
        td::pclib::PointNCloudPtr source_outlier_outlier;
        for (int j = 0; j < neighbor_cloud_indexs.size(); ++j) {
//                td::pclib::PointNCloudPtr test_fusion_model_cloud(new td::pclib::PointNCloud);
//                *test_fusion_model_cloud = *input_target_ + *target_data_->cloud_N_original_pose_vec[neighbor_cloud_indexs[j]];
//                td::pclib::DownsamplingSquareLeaf(test_fusion_model_cloud,LEAF_SIZE);
            pcl::search::KdTree<pcl::PointNormal> neighbor_kdtree;
            neighbor_kdtree.setInputCloud(target_data_->cloud_N_original_pose_vec[neighbor_cloud_indexs[j]]);

            double new_inlier_fraction;
            float fitness_score;
            int new_inlier_size;
//                td::pclib::ShowAlignResult(source_outlier,target_data_->cloud_N_original_pose_vec[neighbor_cloud_indexs[j]],tg_T_sr,"outlier align to neighbor", VIEWSTYLE.model_color,VIEWSTYLE.scene_color,WINDOW_SIZE,VIEWSTYLE.point_size,VIEWSTYLE.background_color);
            td::pclib::PointNCloudPtr source_outlier2 = td::pclib::getOutlierCloud(source_outlier,neighbor_kdtree,inlier_squared_threshold,tg_T_sr);
            std::cout<<"source outlier2 size = "<<source_outlier2->size()<<"  neighbor_index = "<<neighbor_cloud_indexs[j]<<std::endl;

//                td::pclib::getFitness(source_outlier,neighbor_kdtree,inlier_squared_threshold,new_inlier_fraction,new_inlier_size,fitness_score,candidate_tg_T_sr_[candidate_clusters_.min_error_indices[i]]);
//                int inlier_size_increase = new_inlier_size - source_data_->cloud_with_normal->size() * candidate_inlier_fraction_[candidate_clusters_.min_error_indices[i]];
//                std::cout<<"view"<<neighbor_cloud_indexs[j]<<"'s inlier_size_increase = "<<inlier_size_increase<<std::endl;
            new_inlier_size = source_outlier->size() - source_outlier2->size();
            if(new_inlier_size > max_new_inlier_size)
            {
//                    max_inlier_size_increas = new_inlier_size;
                neighbor_index_max_inlier_increase = neighbor_cloud_indexs[j];
                max_new_inlier_size = new_inlier_size;
                source_outlier_outlier = source_outlier2;
            }

        }

        td::pclib::PointNCloudPtr fusion_partial_model(new td::pclib::PointNCloud);
        if(neighbor_index_max_inlier_increase < 0)
        {
            *fusion_partial_model = *input_target_;
            fusion_partial_model_vec.push_back(fusion_partial_model);
        }

        else{
            *fusion_partial_model = *input_target_ + *target_data_->cloud_N_original_pose_vec[neighbor_index_max_inlier_increase];
            std::cout<<"level 1 max inlier_size index = "<<neighbor_index_max_inlier_increase<<"  max_new_inlier_size = "<<max_new_inlier_size<<std::endl;
            auto level2_neighbor = target_data_->view_graph->get_neighbors(neighbor_index_max_inlier_increase);
            //level2 fusion
            max_inlier_size_increas = 0;
            max_new_inlier_size = 0;
            neighbor_index_max_inlier_increase = -1;
            for (int k = 0; k < level2_neighbor.size(); ++k) {
                pcl::search::KdTree<pcl::PointNormal> neighbor_kdtree;
                neighbor_kdtree.setInputCloud(target_data_->cloud_N_original_pose_vec[neighbor_cloud_indexs[k]]);

                if(level2_neighbor[k] == view_candidate_index_)
                    continue;
//                    td::pclib::PointNCloudPtr test_fusion_model_cloud(new td::pclib::PointNCloud);
//                    *test_fusion_model_cloud = *fusion_partial_model + *target_data_->cloud_N_original_pose_vec[level2_neighbor[k]];
//                    td::pclib::DownsamplingSquareLeaf(test_fusion_model_cloud,LEAF_SIZE);
                double new_inlier_fraction;
                int new_inlier_size;
                float fitness_score;

                td::pclib::getFitness(source_outlier_outlier, neighbor_kdtree,inlier_squared_threshold,new_inlier_fraction,new_inlier_size,fitness_score,candidate_tg_T_sr_[candidate_clusters_.min_error_indices[i]]);
                int temp = lround(new_inlier_fraction * source_outlier_outlier->size()) ;
//                    std::cout<<temp<<"   "<<new_inlier_size<<std::endl;

                assert(temp == new_inlier_size);
//                    double inlier_size_increase = new_inlier_size - max_new_inlier_size;
//                    std::cout<<"level2 view"<<level2_neighbor[k]<<"'s inlier_size_increase = "<<inlier_size_increase<<std::endl;
                if(new_inlier_size > max_new_inlier_size)
                {
                    max_new_inlier_size = new_inlier_size;
                    neighbor_index_max_inlier_increase = level2_neighbor[k];
                }
            }
            if(neighbor_index_max_inlier_increase >= 0)
            {
                std::cout<<"level 2 max inlier_size index = "<<neighbor_index_max_inlier_increase<<"  max_new_inlier_size = "<<max_new_inlier_size<<std::endl;
                *fusion_partial_model += *target_data_->cloud_N_original_pose_vec[neighbor_index_max_inlier_increase];
            }
            td::pclib::DownsamplingSquareLeaf(fusion_partial_model,LEAF_SIZE);
            fusion_partial_model_vec.push_back(fusion_partial_model);

        }

    }
    double fusion_time = timer.toc();
    std::cout<<"dynamic fusion take"<<fusion_time<<"ms"<<std::endl;
}
void LocalPipe::ShowCandidate(){

//    pcl::visualization::PCLVisualizer viewer ("candidate pose");
    td::pclib::PointCloudPtr candidate_euler_angles_cloud(new td::pclib::PointCloud);
    for (int i = 0; i < candidate_tg_T_sr_.size(); ++i) {
        Eigen::Matrix4d candidate_pose = candidate_tg_T_sr_[i];
        Eigen::Matrix3d candidate_rotation = candidate_pose.block(0,0,3,3);
        Eigen::Vector3d candidate_euler_angle = candidate_rotation.eulerAngles(2,1,0);
//        std::cout<<"euler_angle = "<<candidate_euler_angle.transpose()<<std::endl;
        Eigen::Vector3d candidate_euler_angle_my = td::RotationToEulerAngle(candidate_rotation);
//        std::cout<<"euler angle my = "<<candidate_euler_angle_my.transpose()<<std::endl;
        pcl::PointXYZ euler_angle(candidate_euler_angle[0],candidate_euler_angle[1],candidate_euler_angle[2]);
//        std::cout<<"PointXYZ = "<<euler_angle<<std::endl;
        candidate_euler_angles_cloud->points.push_back(euler_angle);
    }
    candidate_euler_angles_cloud_ = candidate_euler_angles_cloud;
//    viewer.addPointCloud<Point>(candidate_euler_angles_cloud,ColorHandlerPoint (candidate_euler_angles_cloud, 0.0, 0.0, 255.0),"candidate");
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "candidate");
//    viewer.addCoordinateSystem (0.3);
//    viewer.spin();

}
void LocalPipe::CandidateClustering(){

    pcl::search::KdTree<td::pclib::Point>::Ptr tree (new pcl::search::KdTree<td::pclib::Point>);
    tree->setInputCloud (candidate_euler_angles_cloud_);//创建点云索引向量，用于存储实际的点云信息
    pcl::EuclideanClusterExtraction<td::pclib::Point> ec;
    ec.setClusterTolerance (cluster_radius_); //设置近邻搜索的搜索半径为
    ec.setMinClusterSize (1);//设置一个聚类需要的最少点数目为1
    ec.setMaxClusterSize (25000); //设置一个聚类需要的最大点数目为25000
    ec.setSearchMethod (tree);//设置点云的搜索机制
    ec.setInputCloud (candidate_euler_angles_cloud_);
    ec.extract (cluster_indices_);//从点云中提取聚类，并将点云索引保存在cluster_indices中


    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_.begin (); it != cluster_indices_.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        td::VecMat4 transformation_cluster;
        int min_error_indice = 0;
        double min_error = std::numeric_limits<double>::max ();
        int max_inlier_indice = 0;
        int max_inlier_fraction = 0;
        double mean_error = 0;
        //创建新的点云数据集cloud_cluster，将所有当前聚类写入到点云数据集中
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            cloud_cluster->points.push_back (candidate_euler_angles_cloud_->points[*pit]);
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            transformation_cluster.push_back(candidate_tg_T_sr_[*pit]);
            mean_error += candidate_errors_[*pit];
            if(candidate_errors_[*pit] < min_error)
            {
                min_error = candidate_errors_[*pit];
                min_error_indice = *pit;
            }
            if(candidate_inlier_fraction_[*pit] > max_inlier_fraction)
            {
                max_inlier_fraction = candidate_inlier_fraction_[*pit];
                max_inlier_indice = *pit;
            }
        }
        aligner_clusters_.mean_error.push_back(mean_error/it->indices.size());
        aligner_clusters_.min_error_indices.push_back(min_error_indice);
        aligner_clusters_.vec_aligner_euler_cluster.push_back(cloud_cluster);
        aligner_clusters_.vec_aligner_transformations.push_back(transformation_cluster);
        aligner_clusters_.max_inlier_indices.push_back(max_inlier_indice);
    }
    if (VISUAL){
        pcl::visualization::PCLVisualizer viewer ("aligner pose cluster");
//        if(COLOR_STYLE == "print")
        viewer.setBackgroundColor(VIEWSTYLE.background_color[0],VIEWSTYLE.background_color[1],VIEWSTYLE.background_color[2]);
        viewer.setSize(WINDOW_SIZE[0],WINDOW_SIZE[1]);
        viewer.setCameraPosition(0,0,30,0,0,0);
//        pcl::visualization::Camera camera;
//        viewer.getCameraParameters(camera);
//        std::cout<<"camera rotation:"<<camera.pos[0]<<","<<camera.pos[1]<<","<<camera.pos[2]<<std::endl;

        //show all the candidate
        for (int k = 0; k < aligner_clusters_.vec_aligner_euler_cluster.size(); ++k) {
            std::string cloud_name = "all_candidate"+std::to_string(k);
            auto r = td::UniformSampling<double>(0,255);
            auto g = td::UniformSampling<double>(0,255);
            auto b = td::UniformSampling<double>(0,255);
            viewer.addPointCloud<td::pclib::Point>(aligner_clusters_.vec_aligner_euler_cluster[k],td::pclib::ColorHandlerPoint (aligner_clusters_.vec_aligner_euler_cluster[k], r, g, b),cloud_name);
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloud_name);
            viewer.addCoordinateSystem (0.05);
        }
        while (!viewer.wasStopped())
            viewer.spin();
        viewer.removeAllPointClouds();
        viewer.close();

        for (int i = 0; i < aligner_clusters_.vec_aligner_euler_cluster.size(); ++i) {
            //显示聚类
            viewer.setCameraPosition(0,0,30,0,0,0);
            viewer.addPointCloud<td::pclib::Point>(aligner_clusters_.vec_aligner_euler_cluster[i],td::pclib::ColorHandlerPoint (aligner_clusters_.vec_aligner_euler_cluster[i], 0.0, 0.0, 255.0),"candidate");
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "candidate");
            viewer.addCoordinateSystem (0.3);
            while (!viewer.wasStopped())
                viewer.spin();
            viewer.removePointCloud("candidate");
            viewer.close();
//        int width = aligner_clusters_.vec_aligner_euler_cluster[i]->width;
//        int height = aligner_clusters_.vec_aligner_euler_cluster[i]->height;
            std::cout<<"candidate clustering "<< i << std::endl
                     <<"mean final_fitness = "<<aligner_clusters_.mean_error[i]<<std::endl
                     <<"max_inlier_fraction"<<candidate_inlier_fraction_[aligner_clusters_.max_inlier_indices[i]]<<std::endl;
//        for (int j = 0; j < height; ++j) {
//            for (int k = 0; k < width; ++k) {
            auto transformations = aligner_clusters_.vec_aligner_transformations[i];
            for (int j = 0; j < transformations.size(); ++j) {

//                int index_pcl = i * width +j;
                Eigen::Vector3d candidate_euler(aligner_clusters_.vec_aligner_euler_cluster[i]->points[j].x,aligner_clusters_.vec_aligner_euler_cluster[i]->points[j].y,aligner_clusters_.vec_aligner_euler_cluster[i]->points[j].z);
                td::pclib::PointNCloudPtr candidate_transformed(new td::pclib::PointNCloud);
//                Eigen::Matrix3d candidate_rotation = (Eigen::AngleAxisd( candidate_euler[0],Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(candidate_euler[1],Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(candidate_euler[2],Eigen::Vector3d::UnitX())).matrix();
//                Eigen::Matrix4d candidate_T = Eigen::Matrix4d::Zero();
                Eigen::Matrix4d candidate_T = transformations[j];
//                candidate_T.block(0,0,3,3) = candidate_rotation;
//                pcl::transformPointCloud(*source_data_->cloud_with_normal, *candidate_transformed, candidate_T);
                std::string candidate_name = "aligner pose " + std::to_string(i) + std::to_string(j);
                viewer.addCoordinateSystem(0.03);

                std::cout<<"euler angle = "<<candidate_euler.transpose()<<std::endl;
//                if(COLOR_STYLE == "print")
                    td::pclib::ShowAlignResult(source_data_->cloud_with_normal,input_target_,candidate_T,candidate_name,VIEWSTYLE.scene_color, VIEWSTYLE.model_color,WINDOW_SIZE,VIEWSTYLE.point_size, VIEWSTYLE.background_color);
//                else
//                    td::pclib::ShowAlignResult(source_data_->cloud_with_normal,source_data_->cloud_with_normal,candidate_T,candidate_name,WINDOW_SIZE,POINT_SIZE);

//                viewer.addPointCloud<PointN>(candidate_transformed,ColorHandlerPointN (candidate_transformed, r, g, b),candidate_name);
//                viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, candidate_name);
//                viewer.spin();
            }


        }
    }


    //找具有最大元素个数,最小误差，最大inlier_fraction的cluster,并计算每个cluster的平均误差
    //TODO 如果这几个cluster误差相差比较大，就可以直接剔除误差大的那个
    int max_size = cluster_indices_[0].indices.size();
    int max_size_cluster_indices = 0;
    int max_inlier_fraction_cluster_indices = 0;
    int min_error_cluster_indices = 0;
    double min_error = 100;

    double max_inlier_fraction = 0;
    aligner_clusters_.mean_error.resize(cluster_indices_.size(),0);
    bool min_error_update = false;
    bool min_error_decline_enough = false;

    for (int l = 0; l < cluster_indices_.size(); ++l) {
//        for (int i = 0; i < cluster_indices_[l].indices.size(); ++i)
//            aligner_clusters_.mean_error[l] += candidate_errors_[cluster_indices_[l].indices[i]];
//        aligner_clusters_.mean_error[l] /= cluster_indices_[l].indices.size();
        if(aligner_clusters_.mean_error[l] < min_error)
        {
            //TODO check the 0.05
            min_error_update = true;
            min_error_decline_enough = std::abs(aligner_clusters_.mean_error[l] - min_error)/min_error > 0.05;//如果error下降了一定程度
            min_error = aligner_clusters_.mean_error[l];
            min_error_cluster_indices = l;


        }
        //加上等于是因为有可能出现两个聚类的大小是一样的情况，这样有可能min_error_cluster更新了，而max_size_cluster_indice未更新
        if(cluster_indices_[l].indices.size() > max_size || (cluster_indices_[l].indices.size() == max_size && min_error_update))
        {
            max_size = cluster_indices_[l].indices.size();
            max_size_cluster_indices = l;
        }
        auto current_cluster_inlier_fraction = candidate_inlier_fraction_[aligner_clusters_.max_inlier_indices[l]];
        double fraction_delta = current_cluster_inlier_fraction - max_inlier_fraction;
        if( fraction_delta > 0.02 || ((std::abs(fraction_delta) <= 0.02) && min_error_decline_enough))
        {
            max_inlier_fraction = candidate_inlier_fraction_[aligner_clusters_.max_inlier_indices[l]];
            max_inlier_fraction_cluster_indices = l;
        }
        min_error_update = false;
        min_error_decline_enough = false;
    }

    if(max_size_cluster_indices == min_error_cluster_indices && max_size_cluster_indices == max_inlier_fraction_cluster_indices)
    {
        AlignerToGACandidate(max_size_cluster_indices);
        std::cout<<"max_size_cluster_indices == min_error_cluster_indices == max_inlier_fraction_cluster_indices"<<std::endl;
        return;
    }
    else{
        if(min_error_cluster_indices == max_inlier_fraction_cluster_indices)
        {
            AlignerToGACandidate(max_size_cluster_indices);
            AlignerToGACandidate(min_error_cluster_indices);
            std::cout<<"min_error_cluster_indices == max_inlier_fraction_cluster_indices"<<std::endl;
            return;
        }
        if(min_error_cluster_indices == max_size_cluster_indices)
        {
            AlignerToGACandidate(min_error_cluster_indices);
            AlignerToGACandidate(max_inlier_fraction_cluster_indices);
            std::cout<<"min_error_cluster_indices == max_size_cluster_indices"<<std::endl;
            return;
        }
        if(max_inlier_fraction_cluster_indices == max_size_cluster_indices)
        {
            AlignerToGACandidate(max_inlier_fraction_cluster_indices);
            AlignerToGACandidate(min_error_cluster_indices);
            std::cout<<"max_inlier_fraction_cluster_indices == max_size_cluster_indices"<<std::endl;
            return;
        }
        AlignerToGACandidate(max_size_cluster_indices);
        AlignerToGACandidate(max_inlier_fraction_cluster_indices);
        AlignerToGACandidate(min_error_cluster_indices);


        std::cout<<"!(max_size_indices == min_error_indices == max_inlier_fraction_cluster_indices)"<<std::endl;
        return;
    }


//    for (std::vector<pcl::PointIndices>::const_iterator it = candidate_clusters_.candidate_clusters_indices.begin (); it != candidate_clusters_.candidate_clusters_indices.end (); ++it)
//    {
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
//        //创建新的点云数据集cloud_cluster，将筛选过的聚类写入到点云数据集中
//        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
//        {
//            cloud_cluster->points.push_back (aligner_clusters_.candidate_euler_angles_cloud_->points[*pit]);
//            cloud_cluster->width = cloud_cluster->points.size ();
//            cloud_cluster->height = 1;
//            cloud_cluster->is_dense = true;
//        }
//        candidate_clusters_.vec_candidate_euler_cluster.push_back(cloud_cluster);
//    }



}
void LocalPipe::AlignerToGACandidate(int indices){
    candidate_clusters_.candidate_clusters_indices.push_back(cluster_indices_[indices]) ;
    candidate_clusters_.vec_candidate_euler_cluster.push_back(aligner_clusters_.vec_aligner_euler_cluster[indices]);
    candidate_clusters_.min_error_indices.push_back(aligner_clusters_.min_error_indices[indices]);
    candidate_clusters_.vec_candidate_transformations.push_back(aligner_clusters_.vec_aligner_transformations[indices]);
    candidate_clusters_.max_inlier_indices.push_back(aligner_clusters_.max_inlier_indices[indices]);
    candidate_clusters_.mean_error.push_back(aligner_clusters_.mean_error[indices]);
}
void LocalPipe::ShowFilteredCandidate(){
    pcl::visualization::PCLVisualizer viewer2 ("candidate pose clusters");
//    if(COLOR_STYLE == "print")
        viewer2.setBackgroundColor(VIEWSTYLE.background_color[0],VIEWSTYLE.background_color[1],VIEWSTYLE.background_color[2]);
    if(!(WINDOW_SIZE[0] == 0||WINDOW_SIZE[1]==0))
        viewer2.setSize(WINDOW_SIZE[0],WINDOW_SIZE[1]);
    viewer2.setCameraPosition(0,0,30,0,0,0);
    for (int i = 0; i < candidate_clusters_.vec_candidate_euler_cluster.size(); ++i) {
        //显示聚类

        viewer2.addPointCloud<td::pclib::Point>(candidate_clusters_.vec_candidate_euler_cluster[i],
                                     td::pclib::ColorHandlerPoint(candidate_clusters_.vec_candidate_euler_cluster[i], 0.0, 0.0,
                                                       255.0), "best");
        viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "best");
        viewer2.addCoordinateSystem(0.3);
        while (!viewer2.wasStopped())
            viewer2.spin();
        viewer2.removePointCloud("best");
        viewer2.close();
//        int width = aligner_clusters_.vec_aligner_euler_cluster[i]->width;
//        int height = aligner_clusters_.vec_aligner_euler_cluster[i]->height;
        std::cout<<"best clustering "<< i << std::endl
                 <<"mean final_fitness = "<<candidate_clusters_.mean_error[i]<<std::endl
                 <<"max_inlier_fraction"<<candidate_inlier_fraction_[candidate_clusters_.max_inlier_indices[i]]<<std::endl;
//        for (int j = 0; j < height; ++j) {
//            for (int k = 0; k < width; ++k) {
        auto transformations = candidate_clusters_.vec_candidate_transformations[i];
        for (int j = 0; j < transformations.size(); ++j) {

//                int index_pcl = i * width +j;
            Eigen::Vector3d best_euler(candidate_clusters_.vec_candidate_euler_cluster[i]->points[j].x,candidate_clusters_.vec_candidate_euler_cluster[i]->points[j].y,candidate_clusters_.vec_candidate_euler_cluster[i]->points[j].z);
            td::pclib::PointNCloudPtr candidate_transformed(new td::pclib::PointNCloud);
//                Eigen::Matrix3d candidate_rotation = (Eigen::AngleAxisd( best_euler[0],Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(best_euler[1],Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(best_euler[2],Eigen::Vector3d::UnitX())).matrix();
//                Eigen::Matrix4d candidate_T = Eigen::Matrix4d::Zero();
            Eigen::Matrix4d candidate_T = transformations[j];
//                candidate_T.block(0,0,3,3) = candidate_rotation;
//                pcl::transformPointCloud(*source_data_->cloud_with_normal, *candidate_transformed, candidate_T);
            std::string candidate_name = "candidate pose " + std::to_string(i) + std::to_string(j);
            std::cout<<"best cluster"<<i<<" o_T_s"<<j<<" = \n"<<candidate_T.inverse()<<std::endl;
//            viewer2.addCoordinateSystem(0.03);
//                auto r = td::UniformSampling<double>(0,255);
//                auto g = td::UniformSampling<double>(0,255);
//                auto b = td::UniformSampling<double>(0,255);
            std::cout<<"euler angle = "<<best_euler.transpose()<<std::endl;
//            if(COLOR_STYLE == "print")
                td::pclib::ShowAlignResult(source_data_->cloud_with_normal, input_target_, candidate_T, candidate_name, VIEWSTYLE.scene_color,VIEWSTYLE.model_color,WINDOW_SIZE,VIEWSTYLE.point_size,VIEWSTYLE.background_color);
//            else
//                td::pclib::ShowAlignResult(source_data_->cloud_with_normal, source_data_->cloud_with_normal, candidate_T, candidate_name,WINDOW_SIZE,POINT_SIZE);

//                viewer.addPointCloud<PointN>(candidate_transformed,ColorHandlerPointN (candidate_transformed, r, g, b),candidate_name);
//                viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, candidate_name);
//                viewer.spin();
        }
    }
}

void LocalPipe::set_GA_parameters(){

    GA_size = candidate_clusters_.vec_candidate_euler_cluster.size();
    assert(GA_size == vec_o_T_s1_.size());
    genes_range_.resize(GA_size,std::vector<std::vector<double>>(6,std::vector<double>(2,0)));
    Eigen::Vector4d centroid_model;
    Eigen::Vector4d centroid_scene;
    pcl::compute3DCentroid(*input_target_,centroid_model);
    pcl::compute3DCentroid(*source_data_->cloud_with_normal,centroid_scene);
//    std::cout<<"centroid_model = "<<centroid_model.transpose()<<"  centroid_scene = "<<centroid_scene.transpose()<<std::endl;
//    std::ofstream output_file_genes_range;
//    output_file_genes_range.open("../result/genes_range.txt",std::fstream::trunc | std::fstream::out);
    for (int l = 0; l < GA_size; ++l) {
        int  min_error_indices = candidate_clusters_.min_error_indices[l];
        Eigen::Matrix4d o_T_s1 = vec_o_T_s1_[l];

        Sophus::SE3d s_T_o1_sop_ = Sophus::SE3d(o_T_s1.block(0,0,3,3),o_T_s1.col(3).segment(0,3));
//        Eigen::Vector3d o1_euler_angle_max_o2(0,0,0);
        Eigen::Vector3d o_t_max(0,0,0);
        auto cluster_indices = candidate_clusters_.candidate_clusters_indices[l].indices;
        std::vector<MySolution> current_cluster_init_genes;
        if(cluster_indices.size() > 1)
        {

            std::cout<<"compute genes_range"<<std::endl;
            for (int m = 0; m < cluster_indices.size(); ++m) {
                if(cluster_indices[m] == min_error_indices)
                {
//                    std::cout<<"test cluster"<<l<<" o_T_s"<<m<<" = \n"<<o_T_s1.inverse()<<std::endl;
                    MySolution min_error_gene;
                    min_error_gene.x.resize(6,0);
                    current_cluster_init_genes.push_back(min_error_gene);
                    continue;
                }

                Eigen::Matrix4d o_T_s2 = candidate_tg_T_sr_[cluster_indices[m]];
                Eigen:: Matrix4d o_T = o_T_s2 * o_T_s1.inverse();
//                td::pclib::ShowAlignResult(input_target_, source_data_->cloud_with_normal, o_T_s1.inverse(),"o_T_s1",VIEWSTYLE.scene_color, VIEWSTYLE.model_color,WINDOW_SIZE, VIEWSTYLE.point_size, VIEWSTYLE.background_color);
//
//                Eigen::Matrix4d o_T_s2 = o_T_s1.inverse() * o_T;
//                td::pclib::ShowAlignResult(input_target_, source_data_->cloud_with_normal, o_T_s2,"o_T_s2",VIEWSTYLE.scene_color, VIEWSTYLE.model_color,WINDOW_SIZE, VIEWSTYLE.point_size, VIEWSTYLE.background_color);
//                std::cout<<"test cluster"<<l<<" o_T_s"<<m<<" = \n"<<o_T_s2<<std::endl;

                Eigen::Matrix3d o_R = o_T.block(0,0,3,3);

                Eigen::Vector3d o1_euler_angle_o2 = o_R.eulerAngles(2,1,0);
                //使用这个方式计算，在位姿相近的时候算出来也是小欧拉角，用eigen会算出大欧拉角
                Eigen::Vector3d o_euler_angle_my = td::RotationToEulerAngle(o_R);
//                Eigen::Vector3d s_t_o2 = o_T_s2.col(3).segment(0,3);
                Eigen::Vector3d o_t = o_T.col(3).segment(0,3);
//                Eigen::Vector3d o1_minus_o2 = s_t_o2.array() - o_T_s1.col(3).segment(0,3).array();
//                for (int k = 0; k < 3; ++k) {
//                    o1_max_o2[k] = std::abs(o_t[k]) > std::abs(o1_minus_o2[k]) ? std::abs(o_t[k]): std::abs(o1_minus_o2[k]);
//                }

//                std::cout<<"o_t = "<<o_t.transpose()<<std::endl;
//                std::cout<<"o1_minus_o2 = "<<o1_minus_o2.transpose()<<std::endl;
//                std::cout<<"o1_max_o2 = "<<o1_max_o2.transpose()<<std::endl;
//                Eigen::Matrix3d s_R_o2 = o_T_s2.block(0,0,3,3);
//                Eigen::Vector3d s_euler_angle_o2 = s_R_o2.eulerAngles(2,1,0);
//                Eigen::Matrix3d s_R_o1 = o_T_s1.block(0,0,3,3);
//                Eigen::Vector3d s_euler_angle_o1 = s_R_o1.eulerAngles(2,1,0);
//                Eigen::Vector3d o1_euler_angle_minus_o2 = s_euler_angle_o2.array() - s_euler_angle_o1.array();

//                for (int k = 0; k < 3; ++k) {
//                    o1_euler_angle_max_o2[k] = std::abs(o_euler_angle_my[k]) > std::abs(o1_euler_angle_minus_o2[k]) ? std::abs(o_euler_angle_my[k]): std::abs(o1_euler_angle_minus_o2[k]);
//                }
//                std::cout<<"R = \n"<<o_R<<std::endl;
//                std::cout<<"o1_euler_angle_o2 = "<<o1_euler_angle_o2.transpose()<<std::endl;
//                std::cout<<"o_euler_angle_my = "<<o_euler_angle_my.transpose()<<std::endl;
//                std::cout<<"o1_euler_angle_minus_o2 = "<<o1_euler_angle_minus_o2.transpose()<<std::endl;
//                std::cout<<"o1_euler_angle_max_o2 = "<<o1_euler_angle_max_o2.transpose()<<std::endl;
                MySolution init_gene;
                init_gene.x.resize(6,0);
                for (int i = 0; i < 3; ++i) {
                    init_gene.x[i] = o_t[i];
                    if(genes_range_[l][i][1] < std::abs(o_t[i]))
                        genes_range_[l][i][1] = std::abs(o_t[i]);
//                    std::cout<<"std::abs(o_t["<<i<<"]) = "<<std::abs(o_t[i])<<" genes_range_"<<l<<"_"<<i<<" = "<<genes_range_[l][i][1]<<std::endl;
                }
                for (int j = 0; j < 3; ++j) {
                    init_gene.x[j+3] = o_euler_angle_my[j];
                    if(genes_range_[l][j+3][1] < std::abs(o_euler_angle_my[j]))
                        genes_range_[l][j+3][1] = std::abs(o_euler_angle_my[j]);
//                    std::cout<<"std::abs(o_euler_angle_my["<<j<<"]) = "<<std::abs(o_euler_angle_my[j])<<" genes_range_"<<l<<j+3<<" = "<<genes_range_[l][j+3][1]<<std::endl;
                }
                current_cluster_init_genes.push_back(init_gene);
//            std::cout<<"o1_euler_angle_o2 my = "<<o_euler_angle_my.transpose()<<std::endl;
//            Sophus::SE3d o1_T_o2_sop = td::EulerTranslatetoSE3(o_euler_angle_my,o_T.col(3).segment(0,3));
//            std::cout<<"o_T:\n"<<o_T<<std::endl;
//            std::cout<<"o1_T_o2_sop:\n"<<o1_T_o2_sop.matrix()<<std::endl;
//            std::cout<<"o1_euler_angle_o2 = "<<o1_euler_angle_o2.transpose()<<std::endl;
            }
        } else
        {
            //cluster 中只有一个pose时，这个pose就是min_error_gene
            MySolution min_error_gene;
            min_error_gene.x.resize(6,0);
            current_cluster_init_genes.push_back(min_error_gene);
        }
        init_genes_.push_back(current_cluster_init_genes);

        {
            std::cout<<"set default genes_range"<<std::endl;
            //set default genes_range
            td::pclib::PointNCloudPtr scene_transformed(new td::pclib::PointNCloud);
            Eigen::Matrix4d o_T_s = candidate_clusters_.vec_candidate_transformations[l][0];
            pcl::transformPointCloud(*source_data_->cloud_with_normal, *scene_transformed, o_T_s);
            Eigen::Affine3d o_T_s_aff(o_T_s);
            Eigen::Vector3d centroid_scene_transformed;
            pcl::transformPoint(centroid_scene.segment(0,3), centroid_scene_transformed,o_T_s_aff);
//            std::cout<<"centroid_scene_transformed = "<<centroid_scene_transformed.transpose()<<std::endl;
//            Eigen::Vector3d centroid_scene_3 = centroid_scene.segment(0,3);
//            pcl::transformPoint(centroid_scene_3,centroid_scene_transformed,o_T_s);
//            Eigen::Vector4d centroid_scene_cloud;
//            pcl::compute3DCentroid(*scene_transformed,centroid_scene_cloud);
//            std::cout<<"centroid_scene_cloud = "<<centroid_scene_cloud.transpose()<<std::endl;

            Eigen::Vector3d centroid_model_3d = centroid_model.segment(0,3);
            Eigen::Vector3d centroid_scene_3d = centroid_scene_transformed;
            Eigen::Vector3d scene_centroid_model = centroid_model_3d - centroid_scene_3d;
            o_t_max = scene_centroid_model.array().abs();
//            std::cout<<"genes_range = "<<" ";
            for (int i = 0; i < 3; ++i) {
                genes_range_[l][i][1] = (o_t_max[i] > genes_range_[l][i][1]? o_t_max[i]:genes_range_[l][i][1]);
//                std::cout<<"o_t_max["<<i<<"] = "<<o_t_max[i]<<"  genes_range_"<<l<<"_"<<i<<" = "<<genes_range_[l][i][1]<<std::endl;
            }
            for (int j = 0; j < 3; ++j) {
                genes_range_[l][j+3][1] = default_half_range > genes_range_[l][j+3][1]?default_half_range:genes_range_[l][j+3][1];
//                std::cout<<"default_half_range = "<<default_half_range<<"  genes_range_"<<l<<"_"<<j+3<<" = "<<genes_range_[l][j+3][1]<<std::endl;

            }
            std::cout<<std::endl;
        }

        //每个cluster的range取好后再进行缩放
        for (int k = 0; k < 6; ++k) {
            genes_range_[l][k][1] *= range_scale_[k];
            genes_range_[l][k][0] = -genes_range_[l][k][1];
        }
        for (int n = 0; n < 3; ++n) {
//            std::cout<<"genes range = "<< genes_range_[l][n][1] << "  bound box = "<<target_data_->bound_box[n]/4<<std::endl;
            if(genes_range_[l][n][1] < target_data_->bound_box[n]/5){
                genes_range_[l][n][1] = target_data_->bound_box[n]/5;
                genes_range_[l][n][0] = -genes_range_[l][n][1];
            }

        }

//
//        output_file_genes_range << "genes_range "<<l<<std::endl;
//        for (int k = 0; k < genes_range_[l].size(); ++k) {
//            output_file_genes_range << genes_range_[l][k][0]<<" "<<genes_range_[l][k][1]<<std::endl;
//        }
//        output_file_genes_range.close();
        outlier_fitness_ = 50 * 1000 * candidate_errors_[candidate_clusters_.min_error_indices[l]];
    }
//    output_file_genes_range.close();
}
void LocalPipe::ShowRangeOfGaParameter(){
    pcl::visualization::PCLVisualizer viewer ("genes_range");
    if(!(WINDOW_SIZE[0] == 0||WINDOW_SIZE[1] == 0))
        viewer.setSize(WINDOW_SIZE[0],WINDOW_SIZE[1]);
    std::cout<<"show range of parameters"<<std::endl;
    int vp_1;
    int vp_2;
//    double point_size = 0.05;
    viewer.createViewPort (0.0, 0, 0.5, 1.0, vp_1);
    viewer.createViewPort (0.5, 0, 1.0, 1.0, vp_2);

    viewer.setBackgroundColor(VIEWSTYLE.background_color[0],VIEWSTYLE.background_color[1],VIEWSTYLE.background_color[2],vp_1);
    viewer.setBackgroundColor(VIEWSTYLE.background_color[0],VIEWSTYLE.background_color[1],VIEWSTYLE.background_color[2],vp_2);

    viewer.addCoordinateSystem (0.03);
    td::pclib::ColorHandlerPointN model_color = VIEWSTYLE.model_color;
    model_color.setInputCloud(input_target_);
    viewer.addPointCloud<td::pclib::PointN>(input_target_,model_color,"model_view1",vp_1);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, VIEWSTYLE.point_size, "model_view1",vp_1);

    viewer.addPointCloud<td::pclib::PointN>(input_target_,model_color,"model_view2",vp_2);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, VIEWSTYLE.point_size, "model_view2",vp_2);

//    viewer.addText ("genes_range min", 10, 10, 18, VIEWSTYLE.model_rgb[0], VIEWSTYLE.model_rgb[1], VIEWSTYLE.model_rgb[2], "text1", vp_1);
//    viewer.addText ("genes_range max", 10, 10, 18, VIEWSTYLE.model_rgb[0], VIEWSTYLE.model_rgb[1], VIEWSTYLE.model_rgb[2], "text2", vp_2);
    for (int j = 0; j < GA_size; ++j) {
        for (int i = 0; i < genes_range_[j].size(); ++i) {
            std::vector<double> bound(genes_range_[j].size(),0);
            //viewport 1
            bound[i] = genes_range_[j][i][0];
            Sophus::SE3d o_T = td::EulerTranslatetoSE3(bound);
//            Sophus::SE3d o_T_sb = tg_T_sr0_sop_ * o_T;
            Sophus::SE3d o_T_s1_sop_ = Sophus::SE3d(vec_o_T_s1_[j].block(0,0,3,3),vec_o_T_s1_[j].col(3).segment(0,3));
            Sophus::SE3d o_T_sb = o_T * o_T_s1_sop_;
//            std::cout<<"s_T_ob_min = \n"<<o_T_sb.matrix()<<std::endl;
            if(VERBOSE){
                std::cout<<"bmin"<<i<<"="<<bound[i]<<std::endl;
                std::cout<<"o_T min =\n"<<o_T.matrix()<<std::endl;
            }
            std::string range_title = "genes_range min" + std::to_string(j)+std::to_string(i);
            pcl::PointXYZ text_position;
            text_position.x=0;
            text_position.y=0.1;
            text_position.z=-0.1;

//            viewer.addText (range_title, 10, 10, 18, VIEWSTYLE.model_rgb[0], VIEWSTYLE.model_rgb[1], VIEWSTYLE.model_rgb[2], range_title, vp_1);

//            viewer.addText3D<pcl::PointXYZ> (range_title,text_position , 0.009, VIEWSTYLE.model_rgb[0], VIEWSTYLE.model_rgb[1], VIEWSTYLE.model_rgb[2], "text1", vp_1);
            td::pclib::PointNCloudPtr scene_view_transformed(new td::pclib::PointNCloud);
            pcl::transformPointCloud(*source_data_->cloud_with_normal, *scene_view_transformed, o_T_sb.matrix());
            td::pclib::ColorHandlerPointN scene_color = VIEWSTYLE.scene_color;
            scene_color.setInputCloud(scene_view_transformed);
            viewer.addPointCloud<td::pclib::PointN>(scene_view_transformed,scene_color,"scene_view1",vp_1);
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, VIEWSTYLE.point_size, "scene_view1",vp_1);

            //viewport 2
            bound = std::vector<double>(genes_range_[j].size(),0);
            bound[i] = genes_range_[j][i][1];
            o_T = td::EulerTranslatetoSE3(bound);

            o_T_sb =  o_T * o_T_s1_sop_;
//            std::cout<<"s_T_ob_max = \n"<<o_T_sb.matrix()<<std::endl;
            if(VERBOSE){
                std::cout<<"bmax"<<i<<"="<<bound[i]<<std::endl;
                std::cout<<"o_T max =\n"<<o_T.matrix()<<std::endl;
            }

//            viewer.addText (range_title, 10, 10, 18, VIEWSTYLE.model_rgb[0], VIEWSTYLE.model_rgb[1], VIEWSTYLE.model_rgb[2], range_title, vp_2);
            range_title = "genes_range max" + std::to_string(j)+std::to_string(i);
//            viewer.addText3D<pcl::PointXYZ> (range_title,text_position , 0.009, VIEWSTYLE.model_rgb[0], VIEWSTYLE.model_rgb[1], VIEWSTYLE.model_rgb[2], "text2", vp_2);
            pcl::transformPointCloud(*source_data_->cloud_with_normal, *scene_view_transformed, o_T_sb.matrix());
            scene_color.setInputCloud(scene_view_transformed);
            viewer.addPointCloud<td::pclib::PointN>(scene_view_transformed,scene_color,"scene_view2",vp_2);
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, VIEWSTYLE.point_size, "scene_view2",vp_2);

            viewer.spin();
//            viewer.removeText3D("text1",vp_1);
//            viewer.removeText3D("text2",vp_2);
            viewer.removePointCloud("scene_view1",vp_1);
            viewer.removePointCloud("scene_view2",vp_2);
        }
    }

}
void LocalPipe::OptimizationGa() {
    td::TicToc timer_GA;
    set_GA_parameters();
    if(VISUAL)
        ShowRangeOfGaParameter();

    results_GA_.resize(GA_size);
    assert(GA_size == fusion_partial_model_vec.size());
    assert(GA_size == genes_range_.size());
//    std::cout<<"construct optimization array"<<std::endl;
    std::vector<Optimization::Ptr> optimizers;

    std::vector<bool> optimization_done;
    optimization_done.resize(GA_size, false);

    for (int i = 0; i < GA_size; ++i) {
        Optimization optimizer;
        Optimization::Ptr optimizer_ptr = Optimization::Ptr(new Optimization);
//        std::cout<<"make shared_ptr success"<<std::endl;
        optimizers.push_back(optimizer_ptr);
    }

//    std::cout<<"done"<<std::endl;

    td::TicToc timer;

    omp_set_num_threads(GA_size);
#pragma omp parallel
    {
#pragma omp for
        for (int k = 0; k < GA_size; ++k) {
            int threads_index = omp_get_thread_num();
//            std::cout<<"threads "<<threads_index<<std::endl;
//            Optimization optimizer;
            Optimization::Ptr optimizer = optimizers[threads_index];
//#pragma omp critical
//            {
//                optimizers.push_back(optimizer);
//            }

            std::vector<MySolution> init_genes_manually;
            init_genes_manually = init_genes_[k];
            Sophus::SE3d o_T_s1_sop(vec_o_T_s1_[k].block(0,0,3,3),vec_o_T_s1_[k].col(3).segment(0,3));
            if(set_threads_)
                optimizer->SetThreads(6/GA_size);
            //注意避免使用引用，防止数据竞争和内存共享问题
            optimizer->transfer_data(aligner_.getMaxCorrespondenceDistance(),o_T_s1_sop,outlier_fitness_,genes_range_[k],fusion_partial_model_vec[k],source_data_->cloud_with_normal);
//            optimizer.transfer_data(aligner_.getMaxCorrespondenceDistance(),o_T_s1_sop,outlier_fitness_,genes_range_[k],fusion_partial_model_vec[k],source_data_->cloud_with_normal);
//            optimizer.set_init_genes_manually(init_genes_manually);
            optimizer->set_init_genes_manually(init_genes_manually);
            MySolution tg_T;
            double fitness;
            if(!optimizer->SolveGA(tg_T,fitness))
            {
                //            optimizer.SolveGA(tg_T,fitness);
                results_GA_[k].result = tg_T;
                results_GA_[k].fitness = fitness;
                optimization_done[threads_index] = true;
                for (int i = 0; i < GA_size; ++i) {
                    if(i == threads_index)
                        continue;
                    double fitness_i = optimizers[i]->get_current_best_fitness();
//                    std::cout<<"thread "<<i<<" fitness = "<<fitness_i<<std::endl;
                    if(fitness_i > fitness)
                    {
//                        std::cout<< "need to request stop"<<std::endl;
                        optimizers[i]->StopGA();
                    }

                }
            }
        }
    }


    //show openGA result
    double min_fitness = 300;
    int final_result_index = -1;
    td::VecMat4 best_tg_T_sr_vec;
    best_tg_T_sr_vec.resize(GA_size,Eigen::Matrix4d::Identity());

    for (int j = 0; j < GA_size; ++j) {
        if(!optimization_done[j])
            continue;
        std::vector<double> best_tg_T_vector = results_GA_[j].result.x;
        Sophus::SE3d best_tg_T = td::EulerTranslatetoSE3(best_tg_T_vector);
        Eigen::Matrix4d best_tg_T_sr = best_tg_T.matrix() * vec_o_T_s1_[j];
        best_tg_T_sr_vec[j] = best_tg_T_sr;
//        Eigen::Vector3d best_s1_t_sn(best_tg_T_vector[0],best_tg_T_vector[1],best_tg_T_vector[2]);
//        Eigen::Matrix3d best_s1_R_sn = (Eigen::AngleAxisd(best_tg_T_vector[3],Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(best_tg_T_vector[4],Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(best_tg_T_vector[5],Eigen::Vector3d::UnitX())).matrix();
//        Eigen::Isometry3d best_s1_T_sn = Eigen::Isometry3d::Identity();
//        best_s1_T_sn.rotate(best_s1_R_sn);
//        best_s1_T_sn.pretranslate(best_s1_t_sn);

//        Eigen::Matrix4d best_s_T_on = best_tg_T_sr.inverse();

        float fitness;

//        fitness *= 1000;
//        double fitness = final_inlier_fraction + 0.002 / fitness;

        fitness = results_GA_[j].fitness;

        if(fitness < min_fitness){

            min_fitness = fitness;
            final_result_index = j;
        }

        if(VISUAL)
        {
            td::pclib::ShowAlignResult(source_data_->cloud_with_normal, fusion_partial_model_vec[j], best_tg_T_sr,"GA result",VIEWSTYLE.model_color, VIEWSTYLE.scene_color,WINDOW_SIZE, VIEWSTYLE.point_size, VIEWSTYLE.background_color);
        }

    }
    final_result_pure_GA_->solve_time = timer_GA.tos();
    std::cout<<"pure GA take "<<timer_GA.tos()<<" s"<<std::endl;

    double inlier_fraction;
    float error;
    int inlier_size;
    Eigen::Matrix4d best_tg_T_sr = best_tg_T_sr_vec[final_result_index];
    aligner_.getFitness_ga(inlier_fraction,inlier_size,error,best_tg_T_sr);
    final_result_pure_GA_->pose = best_tg_T_sr;
    final_result_pure_GA_->final_fitness = results_GA_[final_result_index].fitness;
    final_result_pure_GA_->final_error = 1000 * error;
    final_result_pure_GA_->final_inlier_fraction = inlier_fraction;
    int  min_error_indices = candidate_clusters_.min_error_indices[final_result_index];
    double init_min_error = 1000 * candidate_errors_[min_error_indices];
    final_result_pure_GA_->init_error = init_min_error;//存储每个GA手动设置的初始种群的min_fitness_score
    final_result_pure_GA_->fusion_partial_cloud = fusion_partial_model_vec[final_result_index];
    final_result_pure_GA_->init_inlier_fraction = candidate_inlier_fraction_[min_error_indices];
    std::cout<<"before GA, inlier_fraction = "<<final_result_pure_GA_->init_inlier_fraction<<"  error = "<<final_result_pure_GA_->init_error<<std::endl;
    std::cout<<"after GA, inlier_fraction = "<<final_result_pure_GA_->final_inlier_fraction<<"  error = "<<final_result_pure_GA_->final_error<<std::endl;
    final_result_pure_GA_->compute_error_decline_fraction();

//    final_result_pure_GA_->solve_time = solve_time;
    if(LOCAL_FLAG)
        OutputResult(final_result_pure_GA_,pure_GA_path_);

    if(VISUAL)
        td::pclib::ShowAlignResult(source_data_->cloud_with_normal, final_result_pure_GA_->fusion_partial_cloud, final_result_pure_GA_->pose,"final result", VIEWSTYLE.model_color, VIEWSTYLE.scene_color,WINDOW_SIZE, VIEWSTYLE.point_size, VIEWSTYLE.background_color);


//    std::cout<<"final result:\n"<<final_result_pure_GA_->pose<<std::endl;
//    bool right_result = false;
//    right_result = CheckResult(final_result_pure_GA_->pose);


}
void LocalPipe::OptimizationGaMutation() {
    td::TicToc timer_GA;
    set_GA_parameters();
    if(VISUAL)
        ShowRangeOfGaParameter();

    results_GA_.resize(GA_size);
    assert(GA_size == fusion_partial_model_vec.size());
    assert(GA_size == genes_range_.size());
//    std::cout<<"construct optimization array"<<std::endl;
    std::vector<std::shared_ptr<Optimization>> optimizers;

    std::vector<bool> optimization_done;
    optimization_done.resize(GA_size, false);

    for (int i = 0; i < GA_size; ++i) {
        Optimization optimizer;
        std::shared_ptr<Optimization> optimizer_ptr = std::make_shared<Optimization>(optimizer);
//        std::cout<<"make shared_ptr success"<<std::endl;
        optimizers.push_back(optimizer_ptr);
    }

//    std::cout<<"done"<<std::endl;
    int threads_index = 0;
    td::TicToc timer;
    std::vector<std::string> mutation_method;
    mutation_method.push_back("uniform");
    mutation_method.push_back("non_uniform");
    mutation_method.push_back("mutate_by_chromosomes");
    for (int l = 0; l < mutation_method.size(); ++l) {

        for (int k = 0; k < GA_size; ++k) {
            Optimization optimizer__;
            std::shared_ptr<Optimization> optimizer = std::make_shared<Optimization>(optimizer__);

//#pragma omp critical
//            {
//                optimizers.push_back(optimizer);
//            }

            std::vector<MySolution> init_genes_manually;
            init_genes_manually = init_genes_[k];
            Sophus::SE3d o_T_s1_sop(vec_o_T_s1_[k].block(0, 0, 3, 3), vec_o_T_s1_[k].col(3).segment(0, 3));
            if (set_threads_)
                optimizer->SetThreads(6 / GA_size);
            //注意避免使用引用，防止数据竞争和内存共享问题
            optimizer->transfer_data(aligner_.getMaxCorrespondenceDistance(), o_T_s1_sop, outlier_fitness_,
                                     genes_range_[k], fusion_partial_model_vec[k], source_data_->cloud_with_normal);
//            optimizer.transfer_data(aligner_.getMaxCorrespondenceDistance(),o_T_s1_sop,outlier_fitness_,genes_range_[k],fusion_partial_model_vec[k],source_data_->cloud_with_normal);
//            optimizer.set_init_genes_manually(init_genes_manually);
            optimizer->set_init_genes_manually(init_genes_manually);
            MySolution tg_T;
            double fitness;
            if (!optimizer->SolveGA(tg_T, fitness, mutation_method[l])) {
                //            optimizer.SolveGA(tg_T,fitness);
                results_GA_[k].result = tg_T;
                results_GA_[k].fitness = fitness;
                optimization_done[threads_index] = true;
                for (int i = 0; i < GA_size; ++i) {
                    if (i == threads_index)
                        continue;
                    double fitness_i = optimizers[i]->get_current_best_fitness();
//                    std::cout<<"thread "<<i<<" fitness = "<<fitness_i<<std::endl;
                    if (fitness_i > fitness) {
//                        std::cout<< "need to request stop"<<std::endl;
                        optimizers[i]->StopGA();
                    }

                }
            }
        }
    }

    //show openGA result
    double min_fitness = 300;
    int final_result_index = -1;
    td::VecMat4 best_tg_T_sr_vec;
    best_tg_T_sr_vec.resize(GA_size,Eigen::Matrix4d::Identity());

    for (int j = 0; j < GA_size; ++j) {
        if(!optimization_done[j])
            continue;
        std::vector<double> best_tg_T_vector = results_GA_[j].result.x;
        Sophus::SE3d best_tg_T = td::EulerTranslatetoSE3(best_tg_T_vector);
        Eigen::Matrix4d best_tg_T_sr = best_tg_T.matrix() * vec_o_T_s1_[j];
        best_tg_T_sr_vec[j] = best_tg_T_sr;
//        Eigen::Vector3d best_s1_t_sn(best_tg_T_vector[0],best_tg_T_vector[1],best_tg_T_vector[2]);
//        Eigen::Matrix3d best_s1_R_sn = (Eigen::AngleAxisd(best_tg_T_vector[3],Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(best_tg_T_vector[4],Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(best_tg_T_vector[5],Eigen::Vector3d::UnitX())).matrix();
//        Eigen::Isometry3d best_s1_T_sn = Eigen::Isometry3d::Identity();
//        best_s1_T_sn.rotate(best_s1_R_sn);
//        best_s1_T_sn.pretranslate(best_s1_t_sn);

//        Eigen::Matrix4d best_s_T_on = best_tg_T_sr.inverse();

        float fitness;

//        fitness *= 1000;
//        double fitness = final_inlier_fraction + 0.002 / fitness;

        fitness = results_GA_[j].fitness;

        if(fitness < min_fitness){

            min_fitness = fitness;
            final_result_index = j;
        }

        if(VISUAL)
        {
            td::pclib::ShowAlignResult(source_data_->cloud_with_normal, fusion_partial_model_vec[j], best_tg_T_sr,"GA result",VIEWSTYLE.model_color, VIEWSTYLE.scene_color,WINDOW_SIZE, VIEWSTYLE.point_size, VIEWSTYLE.background_color);
        }

    }
    final_result_pure_GA_->solve_time = timer_GA.tos();
    std::cout<<"pure GA take "<<timer_GA.tos()<<" s"<<std::endl;

    double inlier_fraction;
    float error;
    int inlier_size;
    Eigen::Matrix4d best_tg_T_sr = best_tg_T_sr_vec[final_result_index];
    aligner_.getFitness_ga(inlier_fraction,inlier_size,error,best_tg_T_sr);
    final_result_pure_GA_->pose = best_tg_T_sr;
    final_result_pure_GA_->final_fitness = results_GA_[final_result_index].fitness;
    final_result_pure_GA_->final_error = 1000 * error;
    final_result_pure_GA_->final_inlier_fraction = inlier_fraction;
    int  min_error_indices = candidate_clusters_.min_error_indices[final_result_index];
    double init_min_error = 1000 * candidate_errors_[min_error_indices];
    final_result_pure_GA_->init_error = init_min_error;//存储每个GA手动设置的初始种群的min_fitness_score
    final_result_pure_GA_->fusion_partial_cloud = fusion_partial_model_vec[final_result_index];
    final_result_pure_GA_->init_inlier_fraction = candidate_inlier_fraction_[min_error_indices];
    std::cout<<"before GA, inlier_fraction = "<<final_result_pure_GA_->init_inlier_fraction<<"  error = "<<final_result_pure_GA_->init_error<<std::endl;
    std::cout<<"after GA, inlier_fraction = "<<final_result_pure_GA_->final_inlier_fraction<<"  error = "<<final_result_pure_GA_->final_error<<std::endl;
    final_result_pure_GA_->compute_error_decline_fraction();

//    final_result_pure_GA_->solve_time = solve_time;
    if(LOCAL_FLAG)
        OutputResult(final_result_pure_GA_,pure_GA_path_);

    if(VISUAL)
        td::pclib::ShowAlignResult(source_data_->cloud_with_normal, final_result_pure_GA_->fusion_partial_cloud, final_result_pure_GA_->pose,"final result", VIEWSTYLE.model_color, VIEWSTYLE.scene_color,WINDOW_SIZE, VIEWSTYLE.point_size, VIEWSTYLE.background_color);


//    std::cout<<"final result:\n"<<final_result_pure_GA_->pose<<std::endl;
//    bool right_result = false;
//    right_result = CheckResult(final_result_pure_GA_->pose);


}
//void LocalPipe:: OutputResult(const Result::Ptr result, std::string file_path){
//
//    std::cout<<"final result:\n"<<result->pose<<std::endl;
//    bool right_result = false;
//    right_result = CheckResult(result->pose);
//    std::ofstream file;
//    file.open(file_path,fstream::app | fstream::out);
//    if(!file)
//    {
//        std::cerr<<"there is no "<<file_path<<std::endl;
//    }
//    if(right_result)
//    {
//        file<<1<<" ";
//        std::cout<<"right result"<<std::endl;
//    }
//    else
//    {
//        file<<0<<" ";
//        std::cout<<"wrong result"<<std::endl;
//        std::cout<<"result should be : \n"<<check_results_[0]<<std::endl;
//    }
//    Eigen::Matrix<double,3,4> pose = result->pose.block(0,0,3,4);
//    Eigen::VectorXd v_pose(Eigen::Map<Eigen::VectorXd>(pose.data(),pose.cols()*pose.rows()));
//    file<<v_pose.transpose()<<" ";
//    file<<result->final_error<<" "<<result->final_inlier_fraction<<" "<<result->error_decline_fraction<<" "<<result->solve_time<<std::endl;
//    file.close();
//
//}
//void LocalPipe::OutputResult(const Result &result, std::string file_path){
//
//    std::cout<<"final result:\n"<<result.pose<<std::endl;
//    bool right_result = false;
//    right_result = CheckResult(result.pose);
//    std::ofstream file;
//    file.open(file_path,fstream::app | fstream::out);
//    if(!file)
//    {
//        std::cerr<<"there is no "<<file_path<<std::endl;
//    }
//    if(right_result)
//    {
//        file<<1<<" ";
//        std::cout<<"right result"<<std::endl;
//    }
//    else
//    {
//        file<<0<<" ";
//        std::cout<<"wrong result"<<std::endl;
//        std::cout<<"result should be : \n"<<check_results_[0]<<std::endl;
//    }
//    Eigen::Matrix<double,3,4> pose = result.pose.block(0,0,3,4);
//    Eigen::VectorXd v_pose(Eigen::Map<Eigen::VectorXd>(pose.data(),pose.cols()*pose.rows()));
//    file<<v_pose.transpose()<<" ";
//    file<<result.final_error<<" "<<result.final_inlier_fraction<<" "<<result.error_decline_fraction<<" "<<result.solve_time<<std::endl;
//    file.close();
//
//}
bool LocalPipe::CheckResult(const Eigen::Matrix4d &final_result) {
    for (int n = 0; n < check_results_.size(); ++n) {
        if(final_result.isApprox(check_results_[n],0.066))
            return true;
    }
    return false;
}

LocalPipe::LocalPipe() {
    cr_ = Config_Reader::GetInstance();
    cr_->add_model_load_config("../config.ini");
    cr_->add_descriptors_config("../config.ini");
    cr_->system_load_config("../config.ini");
    cr_->add_GA_parameters("../config.ini");
    cr_->add_path_config("../config.ini");
    pure_GA_path_ = BASE_DIR + cr_->pure_GA_path;
    pure_icp_path_ = BASE_DIR +cr_->pure_icp_path;
    GA_icp_path_ = BASE_DIR + cr_->GA_icp_path;
    fine_icp_path_ = BASE_DIR + cr_->fine_icp_path;

    set_threads_ = cr_->set_threads;
    set_descriptor_type(cr_->local_descriptors_type_);
    range_scale_.resize(6,0);

    final_result_pure_GA_ = Result::Ptr(new Result);
    final_result_icp_ = Result::Ptr(new Result);
    final_result_GA_icp_ = Result::Ptr(new Result);
    fusion_partial_model_ = td::pclib::PointNCloudPtr(new td::pclib::PointNCloud);
    std::cout<<"range scale : ";
    for (int i = 0; i < 3; ++i) {
        range_scale_[i] = cr_->range_scale_t_;
        std::cout<<range_scale_[i]<<" ";
    }

    for (int j = 3; j < 6; ++j){
        range_scale_[j] = cr_->range_scale_euler_;
        std::cout<<range_scale_[j]<<" ";
    }
    std::cout<<std::endl;
    cluster_radius_ = cr_->cluster_radius;
    default_half_range = sqrt(cluster_radius_ * cluster_radius_ / 3);
    if(cr_->fusion_mode == "dynamic_fusion")
        fusion_mode_ = FusionMode::dynamic_fusion;
    if(cr_->fusion_mode == "static_fusion")
        fusion_mode_ = FusionMode::static_fusion;
    if(cr_->fusion_mode == "complete_model")
        fusion_mode_ = FusionMode::complete_model;
    if(cr_->fusion_mode == "no_fusion")
        fusion_mode_ = FusionMode::no_fusion;
    LoadCheckResult();
//    Eigen::Matrix4d check_result;
//
//    check_result<< -0.995503,  -0.073469,  0.0598016, 0.00815071,
//            -0.0790468,    0.29634,  -0.951806,   0.341557,
//            0.0522066,  -0.952253,  -0.300815,   0.147992,
//            0,          0,          0,          1;
//    check_results_.push_back(check_result);

//        Eigen::Matrix4d delta = check_results_[1].array()-check_results_[0].array();
//        std::cout<<"delta.norm = "<<delta.norm()<<"  matrix.norm = "<<check_result.norm()<<std::endl;

}

void LocalPipe::LoadCheckResult() {
    std::string scene_path;
    scene_path = cr_->base_dir + "scene_data/"+cr_->model_name;
    boost::filesystem::path data_path;
    std::string data_name = cr_->scene_name+ ".txt";

    if(!td::find_file(scene_path,data_name,data_path))
    {
        std::cout<<"can't find pose file"<<std::endl;
        exit(-1);
    }
    std::ifstream result_file;
    result_file.open(data_path.string(), std::ifstream::in);
    if(!result_file)
    {
        std::cerr<<"cannot open "<<data_path.string()<<std::endl;
        exit(1);
    }
    std::string line;
    while (std::getline(result_file, line) && !line.empty()) {
        Eigen::Matrix4d check_result = Eigen::Matrix4d::Identity();
        int candidate_view_index = 0;
        std::istringstream line_data(line);
        line_data >> candidate_view_index >> check_result(0,0) >> check_result(0,1) >> check_result(0,2) >> check_result(0,3) >>
                                            check_result(1,0) >> check_result(1,1) >> check_result(1,2) >> check_result(1,3) >>
                                            check_result(2,0) >> check_result(2,1) >> check_result(2,2) >> check_result(2,3);
        check_results_.push_back(check_result);
    }


}





