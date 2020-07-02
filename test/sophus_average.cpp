//
// Created by wang on 19-12-30.
//
#include <sophus/se3.hpp>
#include "Test.h"
#include <memory>
//class Test{
//public:
//    void fun(Sophus::SE3d pose_temp)
//    {
//        pose = pose_temp;
//        std::cout<<"fun done"<<std::endl;
//
//    }
//
//private:
//    Sophus::SE3d pose;
//
//};

int main(int argc, char** argv){
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> vec_o_T_s1_;
    std::vector<Test::Ptr> testers;
    int thread_num = 3;

    for (int i = 0; i < thread_num; ++i) {
        Test test;
        Test::Ptr test_ptr = Test::Ptr(new Test);
//        std::cout<<"make shared_ptr success"<<std::endl;
        testers.push_back(test_ptr);
        vec_o_T_s1_.push_back(Eigen::Matrix4d::Identity());

    }
    omp_set_num_threads(thread_num);
#pragma omp parallel
    {
#pragma omp for
        for (int i = 0; i < thread_num; ++i) {
            int threads_index = omp_get_thread_num();
            Test::Ptr tester = testers[threads_index];
            Sophus::SE3d temp(vec_o_T_s1_[i].block(0, 0, 3, 3), vec_o_T_s1_[i].col(3).segment(0, 3));
            std::cout << "temp = \n" << temp.matrix3x4() << std::endl;

            tester->fun(temp);
        }

    }
}