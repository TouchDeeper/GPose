//
// Created by wang on 19-4-29.
//
#include "Matching.h"
#include "parameters.h"
#include "include/config_reader.h"
#include <string>
int main(int argc, char** argv){
    std::string config_path = "../config.ini";
    ReadParameters(config_path);

    Matching match_pipeline;
    match_pipeline.LoadModel();
    match_pipeline.LoadScene();
    match_pipeline.set_global_descripor_type("esf");
    match_pipeline.SceneGlobalDescriptorTrain();
    match_pipeline.SetSpecifiedGlobalResult();

//    match_pipeline.SceneLocalDescriptorsTrain();
    match_pipeline.LocalPipeMatch();
//    Eigen::Matrix4d transformation;
//    td::VecMat4 all_transformations;
//    std::vector<double> inlier_ratio;
//    match_pipeline.estimate_pose(&transformation, &candidate_tg_T_sr_, &inlier_ratio);
//    match_pipeline.estimate_pose_ga(&transformation, &candidate_tg_T_sr_, &inlier_ratio);
//    match_pipeline.estimate_pose_openGA(&transformation, &candidate_tg_T_sr_, &inlier_ratio);

    return 0;




}
