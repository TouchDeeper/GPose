//
// Created by wang on 20-1-7.
//

#ifndef OBJECT_POSE_ESTIMATION_DATAPROCESSOR_HPP
#define OBJECT_POSE_ESTIMATION_DATAPROCESSOR_HPP

#include <string>
#include <fstream>
#include <iostream>
#include <Eigen/Core>
#include <TdLibrary/td_eigen/eigen_common_typedef.h>
#include <TdLibrary/FileOperation/FileOperation.h>
#include "config_reader.h"
#include <boost/filesystem.hpp>
#include <sophus/se3.hpp>
class DataProcessor {

public:
    DataProcessor(std::string model_name, std::string scene_name){
        model_name_ = std::move(model_name);
        scene_name_ = std::move(scene_name);
    }
    void LoadResult(std::string type, std::string type2, std::string index);
    void Load_whole_pipeline_data();
    void ReComputeCorrectRate(double tao);
    void LoadCheckResult();
    void Load_fine_reg_data();
    bool CheckResult(const td::Vec6& v);
    void OutputWholePipeResult(){
        OutputFineRegData();
        OutputTime();
    }
private:
    struct DataBase{
        std::vector<bool> is_right;
        std::string type;
        std::string type2;
        std::string index;
        int right_num=-1;
        float correct_rate;
        void ComputeRightNum(){
            right_num = 0;
            for (int i = 0; i < is_right.size(); ++i) {
                if(is_right[i])
                    right_num++;
            }
        }
        void ComputeCorrectRate(){
            if(right_num < 0)
                ComputeRightNum();
            correct_rate = float(right_num) / is_right.size();
        }
        void SetInfo(std::string _type, std::string _type2, std::string _index){
            type = std::move(_type);
            type2 = std::move(_type2);
            index = std::move(_index);
        }
        template <typename data_type>
        void ComputeAve_right(std::vector<data_type>& Vec, data_type& ave){
            assert(Vec.size() == is_right.size());
            data_type sum = 0;
            if(right_num<0)
                ComputeRightNum();
            for (int i = 0; i < Vec.size(); ++i) {
                if(is_right[i])
                    sum += Vec[i];
            }
            ave = sum / right_num;
        }
        template <typename data_type>
        void ComputeAve_all(std::vector<data_type>& Vec, data_type& ave){
            assert(Vec.size() == is_right.size());
            data_type sum = 0;
            for (int i = 0; i < Vec.size(); ++i) {
                    sum += Vec[i];
            }
            ave = sum / right_num;

        }
    };

    struct TimeData:DataBase{
        std::vector<double> global_time;
        std::vector<double> local_time;
        std::vector<double> total_time;
        double total_time_ave;
        double global_time_ave;
        double local_time_ave;
        void ComputeTimeAve(){
            ComputeAve_right(global_time, global_time_ave);
            ComputeAve_right(local_time, local_time_ave);
            ComputeAve_right(total_time, total_time_ave);
        }


    };
    struct FineRegistrationData:DataBase{
        td::VecVec6 se3s;
        std::vector<double> times;
        std::vector<double> delta_norms;
        std::vector<double> errors;
        std::vector<double> inlier_fractions;
        std::vector<double> error_decline_fractions;
        double time_ave;
        double delta_norm_ave;
        double error_ave;
        double inlier_fraction_ave;
        double error_decline_ave;
        void ComputeTimeAve(){
            ComputeAve_right(times, time_ave);
        }
        void ComputeDeltaNormsAve(){
            ComputeAve_right(delta_norms, delta_norm_ave);
        }
        void ComputeErrorAve(){
            ComputeAve_right(errors,error_ave);
        }
        void ComputeInlierAve(){
            ComputeAve_right(inlier_fractions, inlier_fraction_ave);
        }
        void ComputeErrorDeclineAve(){
            ComputeAve_all(error_decline_fractions, error_decline_ave);
        }
        void ComputeFineRegAve(){
            ComputeTimeAve();
            ComputeDeltaNormsAve();
            ComputeErrorAve();
            ComputeInlierAve();
            ComputeErrorDeclineAve();
        }
    };
    void OutputTime(){
        time_data_.ComputeCorrectRate();
        time_data_.ComputeTimeAve();
        std::cout<<time_data_.type<<"/"<<time_data_.type2<<"/"<<time_data_.index<<" result:"<<" right_percent = "<<time_data_.correct_rate<<"  global time="<<time_data_.global_time_ave
                 <<"  local time="<<time_data_.local_time_ave
                 <<"  total time = "<<time_data_.total_time_ave<<std::endl;
    }
    void OutputFineRegData(){
        for (int i = 0; i < datas_.size(); ++i) {
            datas_[i].ComputeCorrectRate();
            datas_[i].ComputeFineRegAve();
            std::cout<<datas_[i].type<<"/"<<datas_[i].type2<<"/"<<datas_[i].index<<" result:"<<" right_percent = "<<datas_[i].correct_rate<<"  solve time="<<datas_[i].time_ave
                     <<"  delta_norm = "<<datas_[i].delta_norm_ave<<"  final_error="<<datas_[i].error_ave
                     <<"  inlier fraction="<<datas_[i].inlier_fraction_ave<<"  error_decline_fraction="<<datas_[i].error_decline_ave
                     <<std::endl;
        }

    }


    std::vector<FineRegistrationData> datas_;
    TimeData time_data_;
    std::string model_name_;
    std::string scene_name_;
    Config_Reader cr_;
    td::VecVec6 check_results_;
    double tao_;
};


#endif //OBJECT_POSE_ESTIMATION_DATAPROCESSOR_HPP
