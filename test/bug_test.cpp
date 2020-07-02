//
// Created by wang on 20-1-6.
//
#include "Optimization.h"
#include "Test.h"
#include <memory>
int main(int argc, char** argv){
//    std::vector<std::shared_ptr<Optimization>> optimizers;
//
//    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> vec_o_T_s1_;
//    for (int i = 0; i < 3; ++i) {
//        Optimization optimizer;
//        std::shared_ptr<Optimization> optimizer_ptr = std::make_shared<Optimization>(optimizer);
////        std::cout<<"make shared_ptr success"<<std::endl;
//        optimizers.push_back(optimizer_ptr);
//        vec_o_T_s1_.push_back(Eigen::Matrix4d::Identity());
//
//    }
//
////    std::cout<<"done"<<std::endl;
//
////    td::TicToc timer;
//
//    omp_set_num_threads(3);
//#pragma omp parallel
//    {
//#pragma omp for
//        for (int k = 0; k < 3; ++k) {
//            int threads_index = omp_get_thread_num();
//    //            std::cout<<"threads "<<threads_index<<std::endl;
//    //            Optimization optimizer;
//            //TODO use share_ptr defined in class
//            std::shared_ptr<Optimization> optimizer = optimizers[threads_index];
//    //#pragma omp critical
//    //            {
//    //                optimizers.push_back(optimizer);
//    //            }
//
//    //            std::vector<MySolution> init_genes_manually;
//    //            init_genes_manually = init_genes_[k];
//            Sophus::SE3d o_T_s1_sop(vec_o_T_s1_[k].block(0, 0, 3, 3), vec_o_T_s1_[k].col(3).segment(0, 3));
//    //            if(set_threads_)
//    //                optimizer->SetThreads(6/GA_size);
//            //注意避免使用引用，防止数据竞争和内存共享问题
//            std::cout << "o_T_s1 = \n" << o_T_s1_sop.matrix3x4() << std::endl;
//            optimizer->fun(o_T_s1_sop);
//        }
//    }

    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> vec_o_T_s1_;
    std::vector<std::shared_ptr<Test>> testers;
    for (int i = 0; i < 3; ++i) {
        Test test;
        std::shared_ptr<Test> test_ptr = std::make_shared<Test>(test);
//        std::cout<<"make shared_ptr success"<<std::endl;
        testers.push_back(test_ptr);
        vec_o_T_s1_.push_back(Eigen::Matrix4d::Identity());

    }
    omp_set_num_threads(3);
#pragma omp parallel
    {
#pragma omp for
        for (int i = 0; i < 3; ++i) {
            int threads_index = omp_get_thread_num();
            std::shared_ptr<Test> tester = testers[threads_index];
            Sophus::SE3d temp(vec_o_T_s1_[i].block(0, 0, 3, 3), vec_o_T_s1_[i].col(3).segment(0, 3));
            std::cout << "temp = \n" << temp.matrix3x4() << std::endl;

            tester->fun(temp);
        }

    }
}
