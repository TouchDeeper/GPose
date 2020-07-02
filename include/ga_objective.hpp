//
// Created by wang on 19-5-21.
//

#ifndef LOCALPIPELINE_GA_OBJECTIVE_H
#define LOCALPIPELINE_GA_OBJECTIVE_H
#include "sample_consensus_prerejective.h"
#include <Eigen/Core>
// Eigen 几何模块
#include <Eigen/Geometry>
#include "Matching.h"
template <typename T>
class ga_objective {
public:

    // objective function example : Rosenbrock function
    // minimizing f(x,y) = (1 - x)^2 + 100 * (y - x^2)^2
    static std::vector<T> Objective(const std::vector<T>& x)
    {
        Eigen::Vector3d o1_euler_angle_on(x[3],x[4],x[5]);
        Eigen::Matrix3d o1_R_on = (Eigen::AngleAxisd(o1_euler_angle_on[0],Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(o1_euler_angle_on[1],Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(o1_euler_angle_on[2],Eigen::Vector3d::UnitX())).matrix();
        Eigen::Isometry3d o1_T_on = Eigen::Isometry3d::Identity();
        o1_T_on.rotate(o1_R_on);
        o1_T_on.pretranslate(Eigen::Vector3d(x[0],x[1],x[2]));

        Eigen::Matrix4d s_T_on = s_T_o1_ * o1_T_on.matrix();
        double inlier_fraction;
        float fitness_score;
        int inlier_size;
        aligner_.getFitness_ga(inlier_fraction,inlier_size,fitness_score,s_T_on);
//        std::cout<<"final_inlier_fraction = "<<final_inlier_fraction<<std::endl;
        T obj;
        if (inlier_fraction < 0.5)
        {
//            std::cout<<"reject this pose"<<std::endl;
            obj = outlier_fitness_;
        }
        else
        {
//            std::cout<<"accept this pose"<<std::endl;
            obj = -1000 * fitness_score;
        }

        return {obj};
    }
    // NB: GALGO maximize by default so we will maximize -f(x,y)
    void SetStaticData(pcl::SampleConsensusPrerejective<td::pclib::PointN, td::pclib::PointN, td::pclib::FpfhDescriptor, double>& aligner, Eigen::Matrix4d s_T_o1, double first_transformation_fitness){
        aligner_ = aligner;
        s_T_o1_ = s_T_o1;
        outlier_fitness_ = -5 * 1000* first_transformation_fitness;
    }
private:
    static pcl::SampleConsensusPrerejective<td::pclib::PointN, td::pclib::PointN, td::pclib::FpfhDescriptor, double> aligner_;
    static Eigen::Matrix4d s_T_o1_;
    static double outlier_fitness_ ;

};
template <typename T>
pcl::SampleConsensusPrerejective<td::pclib::PointN, td::pclib::PointN, td::pclib::FpfhDescriptor, double> ga_objective<T>::aligner_;
template <typename T>
Eigen::Matrix4d ga_objective<T>::s_T_o1_ = Eigen::Isometry3d::Identity().matrix();
template <typename T>
double ga_objective<T>::outlier_fitness_ = 0;
#endif //LOCALPIPELINE_GA_OBJECTIVE_HPP
