//
// Created by wang on 19-9-10.
//
#include <sophus/se3.hpp>
#include "parameters.h"
#include "config_reader.h"
bool VISUAL;
bool VERBOSE;
std::string k_ga_report_path;
std::vector<int> WINDOW_SIZE;
float LEAF_SIZE;
//double POINT_SIZE;
//std::string COLOR_STYLE;
ViewStyle VIEWSTYLE;
td::VecVec6 CHECK_RESULT;
std::string PURE_GA_PATH;
std::string GA_ICP_PATH;
std::string ICP_PATH;
std::string TIME_PATH;
std::string BASE_DIR;
bool LOCAL_FLAG = false;


void ReadParameters(std::string config_path){
    Config_Reader* cr = Config_Reader::GetInstance();
    cr->add_verbose_config(config_path);
    cr->add_path_config(config_path);
    cr->add_view_config(config_path);
    cr->add_model_load_config(config_path);
    BASE_DIR = "../result/" + cr->model_name + "/";
    TIME_PATH = BASE_DIR + cr->time_path;
    LEAF_SIZE = cr->leaf_size;
    VISUAL = cr->visual;
    VERBOSE = cr->verbose;
    k_ga_report_path = BASE_DIR + cr->report_path;
    PURE_GA_PATH = BASE_DIR + cr->pure_GA_path;
    GA_ICP_PATH = BASE_DIR + cr->GA_icp_path;
    ICP_PATH = BASE_DIR + cr->pure_icp_path;
//    CAD_MODELS_PATH = cr->cad_models_path;
//    MODEL_DATA_PATH = cr->model_data_path;
//    SCENE_DATA_PATH = cr->scene_data_path;
    WINDOW_SIZE.resize(2);
    WINDOW_SIZE[0] = cr->window_width;
    WINDOW_SIZE[1] = cr->window_height;
//    POINT_SIZE = cr->point_size;
    if(cr->color_style == "print")
    {
//        VIEWSTYLE.scene_color = td::pclib::ColorHandlerPointN(227,207,87);
        VIEWSTYLE.scene_color = td::pclib::ColorHandlerPointN(255,0,0);
//        VIEWSTYLE.scene_color.resize(3);
//        VIEWSTYLE.scene_color[0] = 0;
//        VIEWSTYLE.scene_color[1] = 0;
//        VIEWSTYLE.scene_color[2] = 255;
        VIEWSTYLE.line_rgb.resize(3);
        VIEWSTYLE.line_rgb[0] = 0;
        VIEWSTYLE.line_rgb[1] = 0;
        VIEWSTYLE.line_rgb[2] = 255;

        VIEWSTYLE.model_rgb = std::vector<int>(3,0);
        VIEWSTYLE.model_color = td::pclib::ColorHandlerPointN(0,0,0);
        VIEWSTYLE.node_rgb.assign(VIEWSTYLE.model_rgb.begin(), VIEWSTYLE.model_rgb.end());
        VIEWSTYLE.point_size = cr->point_size;
        std::cout<<"point size = "<<VIEWSTYLE.point_size<<std::endl;
        VIEWSTYLE.background_color = std::vector<int>(3,255);


    } else
    {
        VIEWSTYLE.scene_color = td::pclib::ColorHandlerPointN(0,255,0);
//        VIEWSTYLE.scene_color.resize(3);
//        VIEWSTYLE.scene_color[0] = 0;
//        VIEWSTYLE.scene_color[1] = 255;
//        VIEWSTYLE.scene_color[2] = 0;
        VIEWSTYLE.line_rgb.resize(3);
        VIEWSTYLE.line_rgb[0] = 0;
        VIEWSTYLE.line_rgb[1] = 255;
        VIEWSTYLE.line_rgb[2] = 0;
        VIEWSTYLE.model_rgb = std::vector<int>(3,255);
        VIEWSTYLE.model_color = td::pclib::ColorHandlerPointN(255,255,255);
        VIEWSTYLE.node_rgb.resize(3);
        VIEWSTYLE.node_rgb[0] = 0;
        VIEWSTYLE.node_rgb[1] = 255;
        VIEWSTYLE.node_rgb[2] = 0;


        VIEWSTYLE.point_size = 1;
        VIEWSTYLE.background_color = std::vector<int>(3,0);
    }
    //    LoadCheckResult(cr);
}
//void LoadCheckResult(Config_Reader &cr_) {
//    std::string scene_path;
//    scene_path = cr_.base_dir + "scene_data/"+cr_.model_name;
//    boost::filesystem::path data_path;
//    std::string data_name = cr_.scene_name+ ".txt";
//
//    if(!td::find_file(scene_path,data_name,data_path))
//    {
//        std::cout<<"can't find pose file"<<std::endl;
//        exit(-1);
//    }
//    std::ifstream result_file;
//    result_file.open(data_path.string(), std::ifstream::in);
//    if(!result_file)
//    {
//        std::cerr<<"cannot open "<<data_path.string()<<std::endl;
//        exit(1);
//    }
//    std::string line;
//    while (std::getline(result_file, line) && !line.empty()) {
////        Eigen::Matrix4d check_result = Eigen::Matrix4d::Identity();
//        td::Vec6 check_result;
//        int candidate_view_index = 0;
//        std::istringstream line_data(line);
//        line_data >> candidate_view_index >> check_result(0) >> check_result(1) >> check_result(2) >> check_result(3) >>
//                  check_result(4) >> check_result(5);
//        CHECK_RESULT.push_back(check_result);
//    }
//
//}
bool CheckResult(const Eigen::Matrix4d &final_result) {
    Eigen::Matrix3d R = final_result.block(0,0,3,3);
    Eigen::Vector3d t = final_result.col(3).segment(0,3);
    Sophus::SE3d T(R,t);
    td::Vec6 v = T.log();
    double tao = (v-CHECK_RESULT[0]).norm()/std::min(v.norm(), CHECK_RESULT[0].norm() );
    std::cout<<"tao = "<<tao<<std::endl;
    for (int n = 0; n < CHECK_RESULT.size(); ++n) {
        if(v.isApprox(CHECK_RESULT[n],0.053)) // banana 0.053 joystic: sim1 0.0045   scene1 0.008
            return true;
    }
    return false;
}
void OutputResult(const Result::Ptr result, std::string file_path){

    std::cout<<"final result:\n"<<result->pose<<std::endl;
    bool right_result = false;
    right_result = CheckResult(result->pose);

    std::ofstream file;
    file.open(file_path,fstream::app | fstream::out);
    if(!file)
    {
        std::cerr<<"there is no "<<file_path<<std::endl;
    }
    Sophus::SE3d real_T = Sophus::SE3d::exp(CHECK_RESULT[0]);
    Eigen::Matrix3d result_R = result->pose.block(0,0,3,3);
    Eigen::Vector3d result_t = result->pose.col(3).segment(0,3);
    Sophus::SE3d result_T(result_R,  result_t);
    td::Vec6 se3_result = result_T.log();
    std::cout<<"final result se3:\n"<<se3_result.transpose()<<std::endl;

    Sophus::SE3d delta_T = real_T.inverse() * result_T;
    td::Vec6 delta_se3 = delta_T.log();
    double delta_se3_norm = delta_se3.norm();
    if(right_result)
    {
        file<<1<<" ";
        std::cout<<"right result"<<std::endl;
    }
    else
    {
        file<<0<<" ";
        std::cout<<"wrong result"<<std::endl;
        std::cout<<"result should be : \n";
        for(int i = 0; i < CHECK_RESULT.size(); i++)
            std::cout<<Sophus::SE3d::exp(CHECK_RESULT[i]).matrix()<<std::endl;
    }
//    Eigen::Matrix<double,3,4> pose = result->pose.block(0,0,3,4);
//    Eigen::VectorXd v_pose(Eigen::Map<Eigen::VectorXd>(pose.data(),pose.cols()*pose.rows()));
    file<<se3_result.transpose()<<" ";
    file<<result->final_error<<" "<<result->final_inlier_fraction<<" "<<result->error_decline_fraction<<" "<<result->solve_time<<" "<<delta_se3_norm<<std::endl;
    file.close();

}