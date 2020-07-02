//
// Created by wang on 19-9-10.
//

#ifndef OBJECT_POSE_ESTIMATION_PARAMETERS_H
#define OBJECT_POSE_ESTIMATION_PARAMETERS_H

#include <iostream>
#include <string>
#include <vector>
#include <TdLibrary/td_eigen/eigen_common_typedef.h>
#include <TdLibrary/td_eigen/eigen_common_typedef.h>
#include <TdLibrary/FileOperation/FileOperation.h>
#include "config_reader.h"
#include "Result.h"

extern bool VISUAL;
extern bool VERBOSE;
extern std::string k_ga_report_path;
extern std::vector<int> WINDOW_SIZE;//width, height
extern float LEAF_SIZE;
extern td::VecVec6 CHECK_RESULT;
extern std::string PURE_GA_PATH;
extern std::string GA_ICP_PATH;
extern std::string ICP_PATH;
extern std::string TIME_PATH;
extern std::string BASE_DIR;
extern bool LOCAL_FLAG;

struct ViewStyle{
public:
    ViewStyle(): model_color(td::pclib::ColorHandlerPointN(255,255,255)),scene_color(td::pclib::ColorHandlerPointN(255,255,255)){}
    td::pclib::ColorHandlerPointN scene_color;
//    std::vector<int> scene_color;//rgb
    std::vector<int> model_rgb;
    std::vector<int> line_rgb;
    std::vector<int> node_rgb;
    td::pclib::ColorHandlerPointN model_color;
    double point_size;
    std::vector<int> background_color;
//    std::vector<int> window_size;//width, height
};
extern ViewStyle VIEWSTYLE;
void ReadParameters(std::string config_path);
//void LoadCheckResult(Config_Reader &cr_);
bool CheckResult(const Eigen::Matrix4d &final_result);
void OutputResult(const Result::Ptr result, std::string file_path);
#endif //OBJECT_POSE_ESTIMATION_PARAMETERS_H
