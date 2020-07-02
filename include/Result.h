//
// Created by wang on 19-11-27.
//

#ifndef OBJECT_POSE_ESTIMATION_RESULT_H
#define OBJECT_POSE_ESTIMATION_RESULT_H

#include <TdLibrary/PCL/common_typedef.h>
struct Result{
    Result(){
        fusion_partial_cloud = td::pclib::PointNCloudPtr(new td::pclib::PointNCloud);
    }
    Eigen::Matrix4d pose;
    double final_fitness;
    double final_error;
    double final_inlier_fraction;
    double init_error;
    double error_decline_fraction;
    double init_inlier_fraction;
    td::pclib::PointNCloudPtr fusion_partial_cloud;
    void compute_error_decline_fraction(){
        error_decline_fraction = (init_error - final_error)/init_error;
    }
    double solve_time;
public:
    typedef boost::shared_ptr< ::Result> Ptr;
    typedef boost::shared_ptr< ::Result const> ConstPtr;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
#endif //OBJECT_POSE_ESTIMATION_RESULT_H
