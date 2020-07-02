//
// Created by wang on 19-4-29.
//
//#define EIGEN_DONT_VECTORIZE
//#define EIGEN_DISABLE_UNALIGNED_ASSERT
#include "DataProcessor.hpp"
//#include "Matching.h"
//#include "parameters.h"
//#include "config_reader.h"

int main(int argc, char** argv){

//    ReadParameters("../config.ini");
//
//    Matching match_pipeline;
//    match_pipeline.LoadModel();
//    match_pipeline.LoadScene();
//    match_pipeline.SceneLocalDescriptorsTrain();
//    for (int i = 0; i < 16; ++i) {
//        int index = 50 + i * 10;
//        std::stringstream ss;
//        ss << "../result/banana/population_local/" <<index<< ".txt";
//        match_pipeline.CheckResults(ss.str());
//    }

//    std::vector<std::string> paths;
    DataProcessor data_processor("joystick", "scene1");

//    std::string path = "/home/wang/imta_project/pose_estimation/result/joystick/whole_pipe/sim1/GA_icp.txt";
//    paths.push_back(path);
//    std::string path2 = "/home/wang/imta_project/pose_estimation/result/joystick/whole_pipe/sim1/pure_icp.txt";
//    paths.push_back(path2);
//    data_processor.LoadResult(paths);
//    data_processor.CorrectRate();
//    data_processor.Load_fine_reg_data();
    data_processor.Load_whole_pipeline_data();
//    data_processor.ReComputeCorrectRate(0.008);
    data_processor.OutputWholePipeResult();


}
