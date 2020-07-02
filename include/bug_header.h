//
// Created by wang on 20-1-7.
//

#ifndef OBJECT_POSE_ESTIMATION_DATAPROCESSOR_H
#define OBJECT_POSE_ESTIMATION_DATAPROCESSOR_H

#include <string>
#include <fstream>
#include <iostream>
#include <Eigen/Core>
typedef Eigen::Matrix<double, 6, 1> Vec6;
typedef std::vector<Vec6, Eigen::aligned_allocator<Vec6> > VecVec6;
struct FineRegistrationData{
    VecVec6 se3s;
};
class DataProcessor {
public:
    std::vector<FineRegistrationData> datas_;
    void LoadResult();

};


#endif //OBJECT_POSE_ESTIMATION_DATAPROCESSOR_H
