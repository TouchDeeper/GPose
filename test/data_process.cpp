//
// Created by wang on 19-10-9.
//
#include <iostream>
#include <fstream>
#include <iomanip>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <thread>
#include <iomanip>
#include <TdLibrary/td_eigen/eigen_common_typedef.h>

#include <Eigen/Core>
void process_data(std::string model_name, std::string sim_real, std::string local_global,
        std::string type, std::string index){

    std::string path = "../result/"+ model_name + "/" + sim_real + "/" + local_global + "/" + type + "/" + index + ".txt";
    std::ifstream result_file;
    result_file.open(path, std::ifstream::in);
    if(!result_file)
    {
        std::cerr<<"cannot open "<<path<<std::endl;
        exit(1);
    }
    std::string line;
    int data_num = 0;
    int right_num = 0;
    double final_error_sum = 0.0;
    double inlier_fraction_sum = 0.0;
    double error_decline_fraction_sum = 0.0;
    double solve_time_sum = 0.0;
    while (std::getline(result_file, line) && !line.empty()) {
        std::istringstream line_data(line);

        Eigen::Vector4d point;//useless
        int is_right = 0;
        double final_error;
        double inlier_fraction;
        double error_decline_fraction;
        double solve_time;
        line_data >> is_right;
        Eigen::VectorXd T;
        T.resize(12);
        for (int i = 0; i < 12; ++i) {
            line_data>>T(i);
        }
        line_data>> final_error >> inlier_fraction >> error_decline_fraction >> solve_time;

        if(is_right)
        {
            right_num ++;
            final_error_sum += final_error;
            inlier_fraction_sum += inlier_fraction;
            error_decline_fraction_sum += error_decline_fraction;
            solve_time_sum += solve_time;
        }

        data_num ++;
    }

    std::cout<<type<<"/"<<index<<" result:"<<" right_percent = "<<double(right_num)/data_num<<"  final_error="<<final_error_sum/right_num
             <<"  inlier fraction="<<inlier_fraction_sum/right_num<<"  error_decline_fraction="<<error_decline_fraction_sum/right_num
             <<"  solve time="<<solve_time_sum/right_num<<std::endl;
}
void process_data(std::string model_name, std::string sim_real, std::string local_global,
                  std::string type, std::string type2, std::string index){

    std::string path = "../result/"+ model_name + "/" + sim_real + "/" + local_global + "/" + type + "/" + type2 + "/" + index + ".txt";
    std::ifstream result_file;
    result_file.open(path, std::ifstream::in);
    if(!result_file)
    {
        std::cerr<<"cannot open "<<path<<std::endl;
        exit(1);
    }
    std::string line;
    int data_num = 0;
    int right_num = 0;
    double final_error_sum = 0.0;
    double inlier_fraction_sum = 0.0;
    double error_decline_fraction_sum = 0.0;
    double solve_time_sum = 0.0;
    double delta_norm_sum = 0.0;
    while (std::getline(result_file, line) && !line.empty()) {
        std::istringstream line_data(line);

        Eigen::Vector4d point;//useless
        int is_right = 0;
        double final_error;
        double inlier_fraction;
        double error_decline_fraction;
        double solve_time;
        double delta_norm;
        line_data >> is_right;
        Eigen::VectorXd T;
        T.resize(12);
        for (int i = 0; i < 12; ++i) {
            line_data>>T(i);
        }
        line_data>> final_error >> inlier_fraction >> error_decline_fraction >> solve_time >> delta_norm;

        if(is_right)
        {
            right_num ++;
            final_error_sum += final_error;
            inlier_fraction_sum += inlier_fraction;
            error_decline_fraction_sum += error_decline_fraction;
            solve_time_sum += solve_time;
            delta_norm_sum += delta_norm;
        }

        data_num ++;
    }

    std::cout<<type<<"/"<<type2<<"/"<<index<<" result:"<<" right_percent = "<<double(right_num)/data_num<<"  solve time="<<solve_time_sum/right_num
            <<"  delta_norm = "<<delta_norm_sum/right_num<<"  final_error="<<final_error_sum/right_num
             <<"  inlier fraction="<<inlier_fraction_sum/right_num<<"  error_decline_fraction="<<error_decline_fraction_sum/right_num
             <<std::endl;
//    std::cout<<type<<"/"<<type2<<"/"<<index<<" result:"<<" right_percent = "<<double(right_num)/data_num
//             <<"  solve time="<<solve_time_sum/right_num<<"  delta_norm = "<<delta_norm_sum/right_num<<std::endl;

}
void process_data(std::string model_name, std::string type, std::string type2, std::string index){

    std::string path = "../result/"+ model_name + "/" + type + "/" + type2 + "/" + index + ".txt";
    std::ifstream result_file;
    result_file.open(path, std::ifstream::in);
    if(!result_file)
    {
        std::cerr<<"cannot open "<<path<<std::endl;
        exit(1);
    }
    std::string line;
    int data_num = 0;
    int right_num = 0;
    double final_error_sum = 0.0;
    double inlier_fraction_sum = 0.0;
    double error_decline_fraction_sum = 0.0;
    double solve_time_sum = 0.0;
    double delta_norm_sum = 0.0;
    while (std::getline(result_file, line) && !line.empty()) {
        std::istringstream line_data(line);

        Eigen::Vector4d point;//useless
        int is_right = 0;
        double final_error;
        double inlier_fraction;
        double error_decline_fraction;
        double solve_time;
        double delta_norm;
        line_data >> is_right;
        Eigen::VectorXd T;
        T.resize(12);
        for (int i = 0; i < 12; ++i) {
            line_data>>T(i);
        }
        line_data>> final_error >> inlier_fraction >> error_decline_fraction >> solve_time >> delta_norm;

        if(is_right)
        {
            right_num ++;
            final_error_sum += final_error;
            inlier_fraction_sum += inlier_fraction;
            error_decline_fraction_sum += error_decline_fraction;
            solve_time_sum += solve_time;
            delta_norm_sum += delta_norm;
        }

        data_num ++;
    }

    std::cout<<type<<"/"<<type2<<"/"<<index<<" result:"<<" right_percent = "<<double(right_num)/data_num<<"  solve time="<<solve_time_sum/right_num
             <<"  delta_norm = "<<delta_norm_sum/right_num<<"  final_error="<<final_error_sum/right_num
             <<"  inlier fraction="<<inlier_fraction_sum/right_num<<"  error_decline_fraction="<<error_decline_fraction_sum/right_num
             <<std::endl;
//    std::cout<<type<<"/"<<type2<<"/"<<index<<" result:"<<" right_percent = "<<double(right_num)/data_num
//             <<"  solve time="<<solve_time_sum/right_num<<"  delta_norm = "<<delta_norm_sum/right_num<<std::endl;

}
void process_data_global(std::string model_name, std::string type, std::string type2, std::string index){
    std::string path = "../result/"+ model_name + "/" + type + "/" + type2 + "/" + index + ".txt";
    std::ifstream result_file;
    result_file.open(path, std::ifstream::in);
    if(!result_file)
    {
        std::cerr<<"cannot open "<<path<<std::endl;
        exit(1);
    }
    if(index == "time")
    {
        std::string time_line;
        std::string line_GA_icp;

        std::string GA_ICP = "../result/"+ model_name + "/" + type + "/" + type2 + "/" +  "GA_icp.txt";
        std::ifstream GA_ICP_file;
        GA_ICP_file.open(GA_ICP, std::ifstream::in);
        double time_global = 0;
        double time_local = 0;
        double time_total = 0;
        int right_num = 0;
        int data_num = 0;
        while (std::getline(result_file, time_line) && !time_line.empty() && std::getline(GA_ICP_file, line_GA_icp) && !line_GA_icp.empty()) {
            std::istringstream time_data(time_line);
            std::istringstream GA_icp_data(line_GA_icp);
            int is_right = 0;
            double temp_time_global;
            double temp_time_local;
            double temp_time_total;
            GA_icp_data >> is_right;
            if(is_right){
                time_data >> temp_time_global >> temp_time_local >> temp_time_total;
                time_global += temp_time_global;
                time_local += temp_time_local;
                time_total += temp_time_total;
                right_num ++;
            }
            data_num ++;
        }
        std::cout<<type<<"/"<<type2<<"/"<<index<<" result:"<<" right_percent = "<<double(right_num)/data_num<<"  global time="<<time_global/right_num
                <<"  local time="<<time_local/right_num
                 <<"  total time = "<<time_total/right_num<<std::endl;

        }
    else
    {
        std::string line;
        int data_num = 0;
        int right_num = 0;
        double final_error_sum = 0.0;
        double inlier_fraction_sum = 0.0;
        double error_decline_fraction_sum = 0.0;
        double solve_time_sum = 0.0;
        double delta_norm_sum = 0.0;
        while (std::getline(result_file, line) && !line.empty()) {
            std::istringstream line_data(line);

            Eigen::Vector4d point;//useless
            int is_right = 0;
            double final_error;
            double inlier_fraction;
            double error_decline_fraction;
            double solve_time;
            double delta_norm;
            line_data >> is_right;
            Eigen::VectorXd T;
            T.resize(12);
            for (int i = 0; i < 12; ++i) {
                line_data>>T(i);
            }
            line_data>> final_error >> inlier_fraction >> error_decline_fraction >> solve_time >> delta_norm;

            if(is_right)
            {
                right_num ++;
                final_error_sum += final_error;
                inlier_fraction_sum += inlier_fraction;
                error_decline_fraction_sum += error_decline_fraction;
                solve_time_sum += solve_time;
                delta_norm_sum += delta_norm;
            }

            data_num ++;
        }

        std::cout<<type<<"/"<<type2<<"/"<<index<<" result:"<<" right_percent = "<<double(right_num)/data_num<<"  solve time="<<solve_time_sum/right_num
                 <<"  delta_norm = "<<delta_norm_sum/right_num<<"  final_error="<<final_error_sum/right_num
                 <<"  inlier fraction="<<inlier_fraction_sum/right_num<<"  error_decline_fraction="<<error_decline_fraction_sum/right_num
                 <<std::endl;
//    std::cout<<type<<"/"<<type2<<"/"<<index<<" result:"<<" right_percent = "<<double(right_num)/data_num
//             <<"  solve time="<<solve_time_sum/right_num<<"  delta_norm = "<<delta_norm_sum/right_num<<std::endl;
    }


}
int main(){
    std::vector<std::string> result_type;
//    result_type.emplace_back("uniform");
//    result_type.emplace_back("non_uniform");
//    result_type.emplace_back("mutate_by_chromosomes");
    result_type.emplace_back("dynamic_fusion");
    result_type.emplace_back("static_fusion");
    result_type.emplace_back("complete_model");
    result_type.emplace_back("no_fusion");
    std::vector<std::string> model_type;
    model_type.emplace_back("banana");
    model_type.emplace_back("joystick");

    std::vector<std::string> index;
    index.emplace_back("pure_icp");
    index.emplace_back("pure_GA");
    index.emplace_back("GA_icp");
//    index.emplace_back("time");
    for (int i = 0; i < result_type.size(); ++i) {
        for (int j = 0; j < index.size(); ++j) {
            process_data("banana", "sim", "local", "dynamic_fusion",result_type[i], index[j]);
        }
    }
//    std::vector<std::string> sim_or_real;
//    sim_or_real.emplace_back("sim1");
//    sim_or_real.emplace_back("scene1");

    // whole pipeline
//    for (int i = 0; i < model_type.size(); ++i) {
//        for (int k = 0; k < sim_or_real.size(); ++k) {
//            for (int j = 0; j <index.size() ; ++j) {
//                process_data_global(model_type[i],"whole_pipe", sim_or_real[k], index[j]);
//            }
//        }
//    }
//    for (int i = 0; i < model_type.size(); ++i) {
//        for (int k = 0; k < sim_or_real.size(); ++k) {
//            for (int j = 0; j <index.size() ; ++j) {
//                process_data(model_type[i],"fine_registration", sim_or_real[k], index[j]);
//            }
//        }
//    }
    // fine registration

//    for (int i = 0; i < result_type.size(); ++i) {
//        for (int j = 0; j < index.size(); ++j) {
//            process_data("joystick", "sim", "local", "mutation",result_type[i], index[j]);
//        }
//    }
//    for (int j = 1; j < 6; ++j) {
//        process_data("F0",std::to_string(j));
//    }
//    process_data("dynamic_threading","true");
//    process_data("dynamic_threading","false");

//    process_data("set_threads","true");
//    process_data("set_threads","false");
//
//    process_data("register_method", "fine_icp_path_uniform");
//    process_data("register_method", "fine_icp_path_non_uniform");
//    process_data("register_method", "fine_icp_path_mutate_by_chromosomes");
//    process_data("register_method", "pure_GA_uniform");
//    process_data("register_method", "pure_GA_non_uniform");
//    process_data("register_method", "pure_GA_mutate_by_chromosomes");
//    process_data("register_method", "pure_icp");
//    for (int j = 0; j < 16; ++j) {
//        int index = 50 + j * 10;
//        process_data("population_local",std::to_string(index));
//
//    }
}
