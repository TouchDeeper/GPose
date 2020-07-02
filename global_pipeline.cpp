//
// Created by wang on 19-4-29.
//
#include <TdLibrary/tool/tic_toc.h>
#include "Matching.h"
#include "parameters.h"
#include "include/config_reader.h"


int main(int argc, char** argv){

    ReadParameters("../config.ini");

    Matching match_pipeline;
    match_pipeline.LoadModel();
//    match_pipeline.LoadSceneImage();
//    match_pipeline.depthToPCD();
    match_pipeline.LoadScene();
    match_pipeline.set_global_descripor_type("esf");
    td::TicToc global_timer;
    match_pipeline.SceneGlobalDescriptorTrain();
    match_pipeline.MatchGlobalDescriptors();
    double global_t = global_timer.tos();

    td::TicToc local_timer;
    match_pipeline.LocalPipeMatch();
    double local_t = local_timer.tos();
    double total_t = global_t + local_t;
    std::cout<<"global pipeline take "<<global_t<<" s"<<std::endl;
    std::cout<<"local pipeline take "<<local_t<<" s"<<std::endl;
    std::cout<<"total pipeline take "<<total_t<<" s"<<std::endl;
    std::ofstream time_file;
    time_file.open(TIME_PATH, fstream::app | fstream::out);
    if(!time_file)
    {
        std::cerr<<"there is no "<<TIME_PATH<<std::endl;
    }
    time_file << global_t << "  "<< "  " << local_t << " " << total_t<<std::endl;






}
