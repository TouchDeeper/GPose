//
// Created by wang on 20-1-7.
//

#include "DataProcessor.hpp"

//void DataProcessor::LoadResult(std::vector<std::string> &paths) {
//    for (int k = 0; k < paths.size(); ++k) {
//        std::ifstream result_file;
//        result_file.open(paths[k],std::fstream::in);
//        if(!result_file.is_open())
//        {
//            std::cerr<<"result_file doesn't exist"<<std::endl;
//        }
//        std::string line;
//        int line_index = 0;
//        std::vector<Sophus::SE3d> Ts;
//        std::vector<bool> is_rights;
//        while (std::getline(result_file, line) && !line.empty()) {
//
//            std::istringstream ssResultData(line);
//            Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
//            double error;
//            double solve_time;
//            double inlier_fraction;
//            double error_decline_fraction;
////        Eigen::Vector4d point;//useless
//
//            bool is_right;
//            ssResultData >> is_right;
//            is_rights.push_back(is_right);
//            for (int i = 0; i < 4; ++i) {
//                for (int j = 0; j < 3; ++j) {
//                    ssResultData >> pose(j,i);
//                }
//            }
//
//            Eigen::Matrix3d R_part = pose.block(0,0,3,3);
//            Eigen::AngleAxisd aa(R_part);
//            R_part = aa.toRotationMatrix();
//            pose.block(0,0,3,3) = R_part;
//            Ts.push_back(Sophus::SE3d(pose));
//
//
//            ssResultData >> error >> inlier_fraction >> error_decline_fraction >> solve_time;
////        std::cout<<"reading line "<<line_index+1<<std::endl;
////        if(is_right)
////            std::cout<<"right pose  \n"<<pose<<std::endl;
////        else
////            std::cout<<"wrong pose  \n"<<pose<<std::endl;
////        std::cout<<"final_fitness = "<<error<<"  final_inlier_fraction = "<<inlier_fraction<<" error_decline_fraction = "<<error_decline_fraction<<" solve_time = "<<solve_time<<std::endl;
////        if(COLOR_STYLE == "print")
////        td::pclib::ShowAlignResult(scene_->cloud_with_normal,best_match_view,pose, "align result", VIEWSTYLE.scene_color,VIEWSTYLE.model_color,WINDOW_SIZE,VIEWSTYLE.point_size,VIEWSTYLE.background_color);
////        else
////            td::pclib::ShowAlignResult(best_match_view,scene_->cloud_with_normal,pose, "align result",WINDOW_SIZE,POINT_SIZE);
//
//            line_index ++;
//        }
//        result_file.close();
//        is_rightss_.push_back(is_rights);
//        Tss_.push_back(Ts);
//    }
//
//}
void DataProcessor::LoadResult(std::string type, std::string type2, std::string index) {

    std::string path = "../result/"+ model_name_ + "/" + type + "/" + type2 + "/" + index + ".txt";
    std::ifstream result_file;
    result_file.open(path, std::ifstream::in);
    if(!result_file)
    {
        std::cerr<<"cannot open "<<path<<std::endl;
        exit(1);
    }
    if(index == "time")
    {
        time_data_.SetInfo(type, type2, index);
        std::string time_line;
        std::string line_GA_icp;

        std::string GA_ICP = "../result/"+ model_name_ + "/" + type + "/" + type2 + "/" +  "GA_icp.txt";
        std::ifstream GA_ICP_file;
        GA_ICP_file.open(GA_ICP, std::ifstream::in);
        int data_num = 0;
        while (std::getline(result_file, time_line) && !time_line.empty() && std::getline(GA_ICP_file, line_GA_icp) && !line_GA_icp.empty()) {
            std::istringstream time_data(time_line);
            std::istringstream GA_icp_data(line_GA_icp);
            int is_right = 0;
            double temp_time_global;
            double temp_time_local;
            double temp_time_total;
            GA_icp_data >> is_right;
            time_data_.is_right.push_back(is_right);
//            if(is_right){
                time_data >> temp_time_global >> temp_time_local >> temp_time_total;
            time_data_.global_time.push_back(temp_time_global);
            time_data_.local_time.push_back(temp_time_local);
            time_data_.total_time.push_back(temp_time_total);
//                time_global += temp_time_global;
//                time_local += temp_time_local;
//                time_total += temp_time_total;
//                right_num ++;
//            }
            data_num ++;
        }


    }
    else {
        FineRegistrationData fine_reg_data;
        fine_reg_data.SetInfo(type, type2, index);
        std::string line;
        int data_num = 0;
        while (std::getline(result_file, line) && !line.empty()) {
            std::istringstream line_data(line);

//            Eigen::Vector4d point;//useless
            int is_right = 0;
            double final_error;
            double inlier_fraction;
            double error_decline_fraction;
            double solve_time;
            double delta_norm;
            line_data >> is_right;
            td::Vec6 se3;
            for (int i = 0; i < 6; ++i) {
                line_data >> se3(i);
            }
            line_data >> final_error >> inlier_fraction >> error_decline_fraction >> solve_time >> delta_norm;

//            if (is_right) {
//                right_num++;
//                final_error_sum += final_error;
//                inlier_fraction_sum += inlier_fraction;
//                error_decline_fraction_sum += error_decline_fraction;
//                solve_time_sum += solve_time;
//                delta_norm_sum += delta_norm;
//            }
            fine_reg_data.is_right.emplace_back(is_right);
            fine_reg_data.se3s.emplace_back(se3);
            fine_reg_data.errors.push_back(final_error);
            fine_reg_data.inlier_fractions.push_back(inlier_fraction);
            fine_reg_data.error_decline_fractions.push_back(error_decline_fraction);
            fine_reg_data.times.push_back(solve_time);
            fine_reg_data.delta_norms.push_back(delta_norm);

            data_num++;
        }
        datas_.push_back(fine_reg_data);
    }

}
//void DataProcessor::Average() {
//    for (int i = 0; i < Tss_.size(); ++i) {
//        Eigen::Vector3d so3;
//        Eigen::Vector3d t;
//        td::LieAverage(Tss_[i],so3,t);
//        std::cout<<"se3 = "<<t.transpose()<<so3.transpose()<<std::endl;
//    }
//
//}

//void DataProcessor::CorrectRate() {
//    for (int i = 0; i < Tss_[0].size(); ++i) {
//        td::Vec6 se3_GA_ICP_base;
//        se3_GA_ICP_base << 0.713218,  -1.21509, -0.605688,  -1.79862, -0.802502,  0.744604;
//        for (int k = 0; k < Tss_.size(); ++k) {
//            int right_num = 0;
//            double tao_sum = 0;
//            for (int j = 0; j < Tss_[k].size(); ++j) {
//                td::Vec6 se3_GA_ICP = Tss_[k][j].log();
//                double tao = (se3_GA_ICP-se3_GA_ICP).norm()/std::min(se3_GA_ICP.norm(), se3_GA_ICP_base.norm() );
//                if(se3_GA_ICP.isApprox(se3_GA_ICP_base,0.00111)){
//                    tao_sum +=tao;
//                    right_num++;
//                }
//            }
//            std::cout<<"correct rate = "<<right_num/Tss_[k].size()<<"  tao_average = "<<tao_sum/right_num<<std::endl;
//        }
//        std::cout<<"******* new base *******"<<std::endl;
//
//
//    }
//
//}
//void DataProcessor::process_data_global(std::string type, std::string type2, std::string index){
//    std::string path = "../result/"+ model_name_ + "/" + type + "/" + type2 + "/" + index + ".txt";
//    std::ifstream result_file;
//    result_file.open(path, std::ifstream::in);
//    if(!result_file)
//    {
//        std::cerr<<"cannot open "<<path<<std::endl;
//        exit(1);
//    }
//    if(index == "time")
//    {
//        std::string time_line;
//        std::string line_GA_icp;
//
//        std::string GA_ICP = "../result/"+ model_name_ + "/" + type + "/" + type2 + "/" +  "GA_icp.txt";
//        std::ifstream GA_ICP_file;
//        GA_ICP_file.open(GA_ICP, std::ifstream::in);
//        double time_global = 0;
//        double time_local = 0;
//        double time_total = 0;
//        int right_num = 0;
//        int data_num = 0;
//        while (std::getline(result_file, time_line) && !time_line.empty() && std::getline(GA_ICP_file, line_GA_icp) && !line_GA_icp.empty()) {
//            std::istringstream time_data(time_line);
//            std::istringstream GA_icp_data(line_GA_icp);
//            int is_right = 0;
//            double temp_time_global;
//            double temp_time_local;
//            double temp_time_total;
//            GA_icp_data >> is_right;
//            if(is_right){
//                time_data >> temp_time_global >> temp_time_local >> temp_time_total;
//                time_global += temp_time_global;
//                time_local += temp_time_local;
//                time_total += temp_time_total;
//                right_num ++;
//            }
//            data_num ++;
//        }
//        std::cout<<type<<"/"<<type2<<"/"<<index<<" result:"<<" right_percent = "<<double(right_num)/data_num<<"  global time="<<time_global/right_num
//                 <<"  local time="<<time_local/right_num
//                 <<"  total time = "<<time_total/right_num<<std::endl;
//
//    }
//    else
//    {
//        std::string line;
//        int data_num = 0;
//        int right_num = 0;
//        double final_error_sum = 0.0;
//        double inlier_fraction_sum = 0.0;
//        double error_decline_fraction_sum = 0.0;
//        double solve_time_sum = 0.0;
//        double delta_norm_sum = 0.0;
//        while (std::getline(result_file, line) && !line.empty()) {
//            std::istringstream line_data(line);
//
//            Eigen::Vector4d point;//useless
//            int is_right = 0;
//            double final_error;
//            double inlier_fraction;
//            double error_decline_fraction;
//            double solve_time;
//            double delta_norm;
//            line_data >> is_right;
//            td::Vec6 se3;
//            for (int i = 0; i < 6; ++i) {
//                line_data>>se3(i);
//            }
//            line_data>> final_error >> inlier_fraction >> error_decline_fraction >> solve_time >> delta_norm;
//
//            if(is_right)
//            {
//                right_num ++;
//                final_error_sum += final_error;
//                inlier_fraction_sum += inlier_fraction;
//                error_decline_fraction_sum += error_decline_fraction;
//                solve_time_sum += solve_time;
//                delta_norm_sum += delta_norm;
//            }
//
//            data_num ++;
//        }
//
//        std::cout<<type<<"/"<<type2<<"/"<<index<<" result:"<<" right_percent = "<<double(right_num)/data_num<<"  solve time="<<solve_time_sum/right_num
//                 <<"  delta_norm = "<<delta_norm_sum/right_num<<"  final_error="<<final_error_sum/right_num
//                 <<"  inlier fraction="<<inlier_fraction_sum/right_num<<"  error_decline_fraction="<<error_decline_fraction_sum/right_num
//                 <<std::endl;
////    std::cout<<type<<"/"<<type2<<"/"<<index<<" result:"<<" right_percent = "<<double(right_num)/data_num
////             <<"  solve time="<<solve_time_sum/right_num<<"  delta_norm = "<<delta_norm_sum/right_num<<std::endl;
//    }
//
//
//}

void DataProcessor::Load_whole_pipeline_data() {
    std::vector<std::string> index;
    index.emplace_back("pure_icp");
    index.emplace_back("pure_GA");
    index.emplace_back("GA_icp");
    index.emplace_back("time");

    for (int j = 0; j <index.size() ; ++j) {
        LoadResult("whole_pipe", scene_name_, index[j]);
    }

}
void DataProcessor::Load_fine_reg_data() {
    std::vector<std::string> index;
    index.emplace_back("pure_icp");
    index.emplace_back("pure_GA");
    index.emplace_back("GA_icp");

    for (int j = 0; j <index.size() ; ++j) {
        LoadResult("fine_registration", scene_name_, index[j]);
    }

}
void DataProcessor::ReComputeCorrectRate(double tao) {
    tao_ = tao;
    LoadCheckResult();
    for (int i = 0; i < datas_.size(); ++i) {
        for (int j = 0; j < datas_[i].se3s.size(); ++j) {
            CheckResult(datas_[i].se3s[j]) ? datas_[i].is_right[j] = true : datas_[i].is_right[j] = false;
        }
    }
    time_data_.is_right = datas_[2].is_right;

}

void DataProcessor::LoadCheckResult() {
    cr_.add_model_load_config("../config.ini");
    std::string scene_path;
    scene_path = cr_.base_dir + "scene_data/"+model_name_;
    boost::filesystem::path data_path;
    std::string data_name = scene_name_ + ".txt";

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
    int specified_view;
    while (std::getline(result_file, line) && !line.empty()) {
//        Eigen::Matrix4d check_result = Eigen::Matrix4d::Identity();
        td::Vec6 check_result;
        std::istringstream line_data(line);
        line_data >> specified_view >> check_result(0) >> check_result(1) >> check_result(2) >> check_result(3) >>
                  check_result(4) >> check_result(5);
        check_results_.push_back(check_result);
    }

}
bool DataProcessor::CheckResult(const td::Vec6& v) {
    double tao = (v-check_results_[0]).norm()/std::min(v.norm(), check_results_[0].norm() );
//    std::cout<<"tao = "<<tao<<std::endl;
    if(v.isApprox(check_results_[0],tao_)) // banana 0.053 joystic: sim1 0.0045   scene1 0.005
        return true;
    else
        return false;
}