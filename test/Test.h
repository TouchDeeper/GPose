//
// Created by wang on 20-1-6.
//

#ifndef OBJECT_POSE_ESTIMATION_TEST_H
#define OBJECT_POSE_ESTIMATION_TEST_H

#include <sophus/se3.hpp>
#include <TdLibrary/PCL/common_typedef.h>
class Test{
public:
    void fun(Sophus::SE3d pose_temp)
    {
        pose = pose_temp;
        std::cout<<"fun done"<<std::endl;

    }
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    Sophus::SE3d pose;
public:
    typedef boost::shared_ptr< ::Test> Ptr;
    typedef boost::shared_ptr< ::Test const> ConstPtr;

};

#endif //OBJECT_POSE_ESTIMATION_TEST_H
