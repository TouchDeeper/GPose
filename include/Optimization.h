//
// Created by wang on 19-9-12.
//

#ifndef OBJECT_POSE_ESTIMATION_OPTIMIZATION_H
#define OBJECT_POSE_ESTIMATION_OPTIMIZATION_H

#include <iostream>
#include <vector>
#include <iomanip>
#include <string>

#include "config_reader.h"
#include "parameters.h"
#include "sample_consensus_prerejective.h"
//#include "common_typedef.h"
//#include "match/LocalPipe.h"
#include <TdLibrary/td_eigen/eigen_common_typedef.h>
#include <TdLibrary/openGA/src/openga.hpp>
#include <sophus/se3.hpp>


double pose_fitness(double inlier_fraction, double fitness_score);

class LocalPipe;
struct MySolution
{
    std::vector<double> x;

    std::string to_string() const
    {
        std::ostringstream out;
        out<<"{";
        for(unsigned long i=0;i<x.size();i++)
            out<<(i?",":"")<<std::setprecision(10)<<x[i];
        out<<"}";
        return out.str();
    }
};

struct MyMiddleCost
{
    // This is where the results of simulation
    // is stored but not yet finalized.
    double cost;
};
typedef EA::Genetic<MySolution,MyMiddleCost> GA_Type;
typedef EA::GenerationType<MySolution,MyMiddleCost> Generation_Type;

class Optimization {
public:
    Optimization(){
        std::cout<<"this is default constructor"<<std::endl;
//        cr_->add_path_config("../config.ini");
        cr_ = Config_Reader::GetInstance();
        cr_->add_GA_parameters("../config.ini");
        if(cr_->mutation_method == "non_uniform"){
            mutation_method_ = MutationMethod::non_uniform;
            std::cout<<"non_uniform mutation method"<<std::endl;
        } else
        {
            if(cr_->mutation_method == "mutate_by_chromosomes"){
                mutation_method_ = MutationMethod::mutate_by_chromosomes;
                std::cout<<"mutate_by_chromosomes mutation method"<<std::endl;
            } else{
                mutation_method_ = MutationMethod::uniform;
                std::cout<<"uniform mutation method"<<std::endl;
            }


        }
        if(cr_->fitness_method == "hard_code_inlier_fraction")
        {
            fitness_method_ = FitnessMethod::hard_code_inlier_fraction;
            std::cout<<"hard_code_inlier_fraction fitness method"<<std::endl;

        } else{
            fitness_method_ = FitnessMethod::no_hard_code;
            std::cout<<"no_hard_code fitness method"<<std::endl;
        }

        generation_max = cr_->generation_max;
        F0_ = cr_->F0;
        std::cout
            <<"population = "<<cr_->population<<"\n"
            <<"generation_max = "<<cr_->generation_max<<"\n"
            <<"best_stall_max = "<<cr_->best_stall_max<<"\n"
            <<"average_stall_max = "<<cr_->average_stall_max<<"\n"
            <<"tol_stall_best = "<<cr_->tol_stall_best<<"\n"
            <<"tol_stall_average = "<<cr_->tol_stall_average<<"\n"
            <<"crossover_fraction = "<<cr_->crossover_fraction<<"\n"
            <<"elite_count = "<<cr_->elite_count<<"\n"
            <<"mutation_rate = "<<cr_->mutation_rate<<"\n"
            <<"multi_threading = "<<cr_->multi_threading<<"\n"
            <<"dynamic_threading = "<<cr_->dynamic_threading<<"\n"
            <<"mutation_method = "<<std::to_string(mutation_method_)<<"\n"
            <<"fitness_method = "<<std::to_string(fitness_method_)<<"\n"
            <<"F0 = "<<F0_<<"\n"
            ;
//        output_file.open(cr_->report_path,fstream::trunc | fstream::out);
//    }
//    ~Optimization(){
//        output_file.close();
    }
    bool SolveGA(MySolution &best_chromosomes, double &error);
    bool SolveGA(MySolution &best_chromosomes, double &error, std::string mutation_method);
    void transfer_data(double inlier_threshold, Sophus::SE3d &tr_T_sr1_sop,double outlier_fitness,std::vector<std::vector<double>> genes_range,td::pclib::PointNCloudPtr &fusion_partial_model,td::pclib::PointNCloudPtr &source_cloud){
        input_source_ = *source_cloud;
        tg_T_sr0_sop_ = tr_T_sr1_sop;
        outlier_fitness_ = outlier_fitness;
        genes_range_ = genes_range;

        target_cloud_ = *fusion_partial_model;
        target_cloud_kdtree_.setInputCloud(fusion_partial_model);

        target_cloud_size_ = target_cloud_.size();
//        inlier_squared_threshold_ = inlier_threshold * inlier_threshold;
        //TODO 这里不要平方效果会好很多
        inlier_squared_threshold_ = inlier_threshold;
    }
    void set_init_genes_manually(std::vector<MySolution> init_genes);
    void SetThreads(int n_threads){
        ga_obj_.N_threads = n_threads;
    }
    double get_current_best_fitness(){
        ga_obj_.get_best_total_cost();
    }
    double StopGA(){
        ga_obj_.user_request_stop = true;
    }

private:
    void init_genes(MySolution& p,const std::function<double(void)> &rnd01);
    MySolution mutate(
            const Generation_Type& last_generation,
            const MySolution& X_base,
            const std::function<double(void)> &rnd01,
            double shrink_scale);
    MySolution crossover(
            const MySolution& X1,
            const MySolution& X2,
            const std::function<double(void)> &rnd01);
    double default_shrink_scale(int n_generation,const std::function<double(void)> &rnd01);
//    MySolution mutate_uniform(
//            const MySolution& X_base,
//            const std::function<double(void)> &rnd01,
//            double shrink_scale);
    void CheckGenesRange(const MySolution &p);
    void getFitnessWithFusionView(double& inlier_fraction, int& inlier_size,
                                  float& fitness_score, Eigen::Matrix4d& population_transformation);
    enum MutationMethod{
        non_uniform,
        uniform,
        mutate_by_chromosomes
    };
    enum FitnessMethod{
        hard_code_inlier_fraction,
        no_hard_code
    };
private:
    MutationMethod mutation_method_;
    FitnessMethod fitness_method_;
    bool eval_solution (const MySolution &p, MyMiddleCost &c);
    //TODO 设置检查init_genes_manually的机制
    //GA parameters
    GA_Type ga_obj_;
    std::vector<MySolution> init_genes_manually_;
    pcl::SampleConsensusPrerejective<td::pclib::PointN, td::pclib::PointN, td::pclib::FpfhDescriptor, double> aligner_;
    Sophus::SE3d tg_T_sr0_sop_; //the Sopuhs SE3 base transformation from source to target
    double outlier_fitness_;
    std::vector<std::vector<double>> genes_range_;
//    std::ofstream output_file;
    Config_Reader* cr_;
    int target_cloud_size_;
//    int source_cloud_size_;
    int generation_max;
    td::pclib::PointNCloud target_cloud_;
    td::pclib::PointNCloud input_source_;
    double inlier_squared_threshold_;
    pcl::search::KdTree<pcl::PointNormal> target_cloud_kdtree_;
    float F0_;
public:
    typedef boost::shared_ptr< ::Optimization> Ptr;
    typedef boost::shared_ptr< ::Optimization const> ConstPtr;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


#endif //OBJECT_POSE_ESTIMATION_OPTIMIZATION_H
