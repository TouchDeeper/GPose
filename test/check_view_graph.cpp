//
// Created by wang on 19-4-29.
//
#include "Matching.h"
#include "parameters.h"
#include "config_reader.h"

int main(int argc, char** argv){

    ReadParameters("../config.ini");
//    std::string base_dir = "../data/model_data";
//    std::string scene_path = base_dir + "/bun_zipper/views_original_pose/view1.pcd";
    Matching match_pipeline;
    match_pipeline.LoadModel();
//    match_pipeline.LoadScene(scene_path);
//    match_pipeline.SceneLocalDescriptorsTrain();
    match_pipeline.CheckViewGraph();
//    std::vector<int> fusion_index;
//    fusion_index.push_back(2);
//    fusion_index.push_back(3);
//    match_pipeline.FusionViews(fusion_index);
}
