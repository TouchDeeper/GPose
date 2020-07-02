//
// Created by wang on 19-9-12.
//


#include <vector>
#include <string>
#include <iomanip>

#include <TdLibrary/tool/random_tool.hpp>
#include <TdLibrary/tool/random_tool.h>
#include <TdLibrary/slam_tool/motion_transformation.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include "Optimization.h"
#include "match/LocalPipe.h"

std::ofstream output_file;


//bool eval_solution(
//        const MySolution& p,
//        MyMiddleCost &c)
//{
//    constexpr double pi=3.141592653589793238;
//    c.cost=10*double(p.x.size());
//    for(unsigned long i=0;i<p.x.size();i++)
//        c.cost+=p.x[i]*p.x[i]-10.0*cos(2.0*pi*p.x[i]);
//    return true;
//}

MySolution Optimization::crossover(
        const MySolution& X1,
        const MySolution& X2,
        const std::function<double(void)> &rnd01)
{
    MySolution X_new;
    for(unsigned long i=0;i<X1.x.size();i++)
    {
        double r=rnd01();
        X_new.x.push_back(r*X1.x[i]+(1.0-r)*X2.x[i]);

    }
    for (int j = 0; j < X_new.x.size(); ++j) {
        assert(X_new.x[j]>=genes_range_[j][0] && X_new.x[j]<=genes_range_[j][1]);
    }
    return X_new;
}
double calculate_SO_total_fitness(const GA_Type::thisChromosomeType &X)
{
    // finalize the cost
    return X.middle_costs.cost;
}
void SO_report_generation(
        int generation_number,
        const EA::GenerationType<MySolution,MyMiddleCost> &last_generation,
        const MySolution& best_genes)
{
    std::cout
            <<"Generation ["<<generation_number<<"], "
            <<"Best="<<last_generation.best_total_cost<<", "
            <<"Average="<<last_generation.average_cost<<", "
            <<"Best genes=("<<best_genes.to_string()<<")"<<", "
            <<"Exe_time="<<last_generation.exe_time
            <<std::endl;

    output_file
            <<generation_number<<"\t"
            <<last_generation.average_cost<<"\t"
            <<last_generation.best_total_cost<<"\t";
    for (int i = 0; i < best_genes.x.size(); ++i) {
        output_file<<best_genes.x[i]<<"\t";
    }
    output_file<<"\n";
}
double Optimization::default_shrink_scale(int n_generation,const std::function<double(void)> &rnd01)
{

    double lambda = exp(1-double(generation_max)/(generation_max+1-n_generation));
    double scale = F0_ * pow(2,lambda);
//    double scale=(n_generation<=5?1.0:1.0/sqrt(n_generation-5+1));
//    if(rnd01()<0.4)
//        scale*=scale;
//    else if(rnd01()<0.1)
//        scale=1.0;
//    std::cout<<"my shrink_scale = "<<scale<<std::endl;
    return scale;
}
MySolution Optimization::mutate(
        const Generation_Type& last_generation,
        const MySolution& X_base,
        const std::function<double(void)> &rnd01,
        double shrink_scale)
{
    switch (mutation_method_){
        case MutationMethod::non_uniform:
        {
            MySolution X_new;
            bool out_of_range;
            std::vector<std::vector<double>> genes_sigma;
            genes_sigma.resize(X_base.x.size());
            for (int j = 0; j < X_base.x.size(); ++j) {
                double genes_left_sigma = X_base.x[j] - shrink_scale * genes_range_[j][0];
                double genes_right_sigma = shrink_scale * genes_range_[j][1] - X_base.x[j];
                genes_sigma[j].push_back(genes_left_sigma);
                genes_sigma[j].push_back(genes_right_sigma);

            }

            do{
                out_of_range=false;
                X_new=X_base;

                for(unsigned long i=0;i<X_new.x.size();i++)
                {

                    if(td::BernouliSampling(0.5))//    td::pclib::ShowAlignResult(input_source_,input_source_,first_two_transformation_[0].first,"first_candidate");
//                X_new.x[i] += std::abs(td::NormalSampling(0,1.25*genes_sigma[i][1]*sqrt(shrink_scale)));
                        X_new.x[i] += std::abs(td::NormalSampling(0,genes_sigma[i][1]));
                    else
//                X_new.x[i] -= std::abs(td::NormalSampling(0,1.25*genes_sigma[i][0]*sqrt(shrink_scale)));
                        X_new.x[i] -= std::abs(td::NormalSampling(0,genes_sigma[i][0]));
                    if(X_new.x[i] < genes_range_[i][0] || X_new.x[i] > genes_range_[i][1])
                        out_of_range=true;
                }
            } while(out_of_range);
            return X_new;
        }
        case MutationMethod::uniform :{
            MySolution X_new;
            bool out_of_range;

            do{
                out_of_range=false;
                X_new=X_base;

                for(unsigned long i=0;i<X_new.x.size();i++)
                {
                    X_new.x[i] = td::UniformSampling<double>(shrink_scale * genes_range_[i][0], shrink_scale * genes_range_[i][1]);
                    if(X_new.x[i] < genes_range_[i][0] || X_new.x[i] > genes_range_[i][1])
                        out_of_range=true;
                }
            } while(out_of_range);
            return X_new;
        }
        case MutationMethod::mutate_by_chromosomes:{

            MySolution X_best = last_generation.chromosomes[last_generation.best_chromosome_index].genes;
            double fitness_best = last_generation.best_total_cost;
            double fitness_worst = last_generation.worst_total_cost;
            //TODO deal with the inf
//            std::cout<<"fitness_worst = "<<fitness_worst<<std::endl;
//            assert(std::isinf(fitness_worst)==0);
//            if(std::isinf(fitness_worst) == 0){
//                std::cout<<"fitness_worst/fitness_best = "<<fitness_worst/fitness_best<<std::endl;
//            }
//    std::cout<<"fitness_best  = "<<fitness_best<<"  "<<"fitness_worst = "<<fitness_worst<<std::endl;
            MyMiddleCost X_base_cost;
            eval_solution(X_base,X_base_cost);
            double fitness_X_base = X_base_cost.cost;
//    std::cout<<"fitness_X_base = "<<fitness_X_base<<std::endl;
            double gma1;
            if(fitness_X_base >= fitness_worst || std::isinf(fitness_X_base) == 1)
                gma1 = 1;
            else
                gma1 = (fitness_X_base - fitness_best)/(fitness_worst - fitness_best);
            assert(gma1 <= 1);
            double gma2 = 1 - gma1;
//    std::cout<<"fitness_best = "<<fitness_best<<std::endl;
            MySolution X_new;
//    std::cout<<"mutation start"<<std::endl;
//    std::cout<<"X_base:"<<X_base.to_string()<<std::endl;
            int out_of_rang_num = -1;
            double raw_gma2 = gma2;
            bool out_of_range;
            do{
//                std::cout<<"mutate out of range"<<std::endl;
//                std::cout<<"shrink_scale = "<<shrink_scale<<std::endl;
//                std::cout<<"gma1 = "<<gma1<<"  "<<"gma2 = "<<gma2<<std::endl;
                double delta = M_PI_2 / 6;
                if(out_of_rang_num >= 0)
                    gma2 = raw_gma2 * cos(out_of_rang_num * delta);
                out_of_range=false;
                X_new=X_base;
                int index2 ;
                int index3;
//        std::cout<<"random choose in "<<last_generation.chromosomes.size()<<std::endl;

                do{
                    index2 = int(td::UniformSampling<float>(0,last_generation.chromosomes.size()));
                    index3 = int(td::UniformSampling<float>(0,last_generation.chromosomes.size()));
                }while (index2 == index3);
//        std::cout<<"index2 = "<<index2<<"  "<<"index3 = "<<index3<<std::endl;
                MySolution X_r2 = last_generation.chromosomes[index2].genes;
                MySolution X_r3 = last_generation.chromosomes[index3].genes;
//        std::cout<<"X_r2 : "<<X_r2.to_string()<<std::endl;
//        std::cout<<"X_r3 : "<<X_r3.to_string()<<std::endl;

//        std::cout<<"generate new genes"<<std::endl;
                for(unsigned long i=0;i<X_new.x.size();i++)
                {
                    double random_part = X_r2.x[i] - X_r3.x[i];
                    double to_best_part = X_best.x[i] - X_base.x[i];

//                    std::cout<<"random_part = "<<random_part<<"  to best part = "<<to_best_part<<std::endl;
                    X_new.x[i] += (shrink_scale * gma2 * random_part + shrink_scale * gma1 * to_best_part);
                    if(X_new.x[i] < genes_range_[i][0] || X_new.x[i] > genes_range_[i][1])
                    {
                        out_of_range=true;
//                        if(out_of_rang_num > 6)
//                        {
//                            std::cout<<"shrink_scale = "<<shrink_scale<<"  gma1 = "<<gma1<<"  X_base i = "<<X_base.x[i]<<"   X_best i = "<<X_best.x[i]<<std::endl;
//                        }


                    }

                }
                out_of_rang_num ++;
//                assert(out_of_rang_num < 7);
                if(out_of_rang_num>6)
                {
                    std::cerr<<"gma2 = "<<gma2<<", still out of range"<<std::endl;
                }


            } while(out_of_range);
            return X_new;
        }
    }

}

void Optimization::init_genes(MySolution& p,const std::function<double(void)> &rnd01)
{

    for(int i=0;i<genes_range_.size();i++)
        p.x.push_back(td::UniformSampling<double>(genes_range_[i][0],genes_range_[i][1]));
    for (int j = 0; j < p.x.size(); ++j) {
        assert(p.x[j]>=genes_range_[j][0] && p.x[j]<=genes_range_[j][1]);
    }

}

bool Optimization::eval_solution (const MySolution &p, MyMiddleCost &c){
//    Eigen::Vector3d tr_euler_angle(p.x[3],p.x[4],p.x[5]);
//    Eigen::Matrix3d tr_R = (Eigen::AngleAxisd(tr_euler_angle[0],Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(tr_euler_angle[1],Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(tr_euler_angle[2],Eigen::Vector3d::UnitX())).matrix();
//    Eigen::Isometry3d o1_T_on = Eigen::Isometry3d::Identity();
//    std::cout<<"tr_R:\n"<<tr_R<<std::endl;
//    std::cout<<"o1_t_on:\n"<<Eigen::Vector3d(p.x[0],p.x[1],p.x[2]).transpose()<<std::endl;
//    o1_T_on.rotate(tr_R);
//    o1_T_on.pretranslate(Eigen::Vector3d(p.x[0],p.x[1],p.x[2]));
//    std::cout<<"o1_T_on:\n"<<o1_T_on.matrix()<<std::endl;
//    std::cout<<"vec_o_T_s1_\n"<<vec_o_T_s1_.matrix()<<std::endl;
//    Eigen::Matrix4d tg_T_srn = vec_o_T_s1_ * o1_T_on.matrix();
    Eigen::Matrix4d tg_T_srn = Eigen::Matrix4d::Identity();//不用sophus会有概率报错
//    std::cout<<"s_T_o1_sop\n"<<s_T_o1_sop.matrix()<<std::endl;
    Sophus::SE3d tg_T_sop = td::EulerTranslatetoSE3(p.x);
//    std::cout<<"tg_T_sop\n"<<tg_T_sop.matrix()<<std::endl;
    Sophus::SE3d tg_T_srn_sop = tg_T_sop * tg_T_sr0_sop_ ;
    tg_T_srn = tg_T_srn_sop.matrix();

//    std::cout<<"tg_T_srn:\n"<<tg_T_srn<<std::endl;

    double inlier_fraction;
    int inlier_size;
    float error;
//    aligner_.getFitnessWithFusionView(final_inlier_fraction,inlier_size, error,tg_T_srn,target_cloud_);
    getFitnessWithFusionView(inlier_fraction, inlier_size, error, tg_T_srn);
//    std::cout<<"final_inlier_fraction = "<<final_inlier_fraction<<"  inlier_size = "<<inlier_size<<"  error = "<<error<<std::endl;
//    aligner_.getFitness_ga(final_inlier_fraction,inlier_size, error,tg_T_srn);

//        std::cout<<"final_inlier_fraction = "<<final_inlier_fraction<<std::endl;
    switch (fitness_method_){
        case FitnessMethod::hard_code_inlier_fraction:{
            //TODO check the inlier fraction
            if (inlier_fraction < 0.5)
            {
                //            std::cout<<"reject this pose"<<std::endl;
                c.cost = 1000 * error + tan((0.5 - inlier_fraction) * M_PI);
            }
            else
            {
                //            std::cout<<"accept this pose"<<std::endl;
                c.cost = 1000 * error;
            }
            return true;
        }
        case FitnessMethod::no_hard_code:{
            c.cost = pose_fitness(inlier_fraction, error);
//            if(c.cost > 10000)
//                std::cout<<"target_cloud_size_ = "<<target_cloud_size_<<"  "<<"final_inlier_fraction = "<<final_inlier_fraction<<"  inlier_size = "<<inlier_size<<"  fiteness_score = "<<error<<std::endl;
            return true;
        }
    }
}
double pose_fitness(double inlier_fraction, double fitness_score){
    double cost;
    double lambda = 2.0;
//            double overlapping_coef = target_cloud_size_ / (inlier_size * pow(inlier_fraction,1+lambda));
    double overlapping_coef = 1 /  pow(inlier_fraction,1+lambda);
//            std::cout<<"overlapping_coef = "<<overlapping_coef<<"  fitness_score = "<<fitness_score<<std::endl;
    cost = overlapping_coef * 1e6 * fitness_score;
    return cost;

}
void Optimization::getFitnessWithFusionView(double& inlier_fraction, int& inlier_size,
                                            float& fitness_score, Eigen::Matrix4d& population_transformation) {
    // Initialize variables
    std::vector<int> inliers;
    inliers.reserve (input_source_.size ());
    fitness_score = 0.0f;

    // Transform the input dataset using the population transformation
    td::pclib::PointNCloudPtr source_transformed(new td::pclib::PointNCloud);
//    source_transformed->resize (target_cloud_->size ());
    transformPointCloud (input_source_, *source_transformed, population_transformation);

    // For each point in the source dataset
    for (size_t i = 0; i < source_transformed->points.size (); ++i)
    {
        // Find its nearest neighbor in the target
        std::vector<int> nn_indices (1);
        std::vector<float> nn_dists (1);
        target_cloud_kdtree_.nearestKSearch (source_transformed->points[i], 1, nn_indices, nn_dists);

        // Check if point is an inlier
        if (nn_dists[0] < inlier_squared_threshold_)
        {
            // Update inliers
            inliers.push_back (static_cast<int> (i));

            // Update fitness score
            fitness_score += nn_dists[0];
        }
    }

    // Calculate MSE
    if (inliers.size () > 0)
        fitness_score /= static_cast<float> (inliers.size ());
    else
        fitness_score = std::numeric_limits<float>::max ();
    inlier_fraction = static_cast<float> (inliers.size ()) / static_cast<float> (input_source_.size ());
    inlier_size = inliers.size();
}
bool Optimization::SolveGA(MySolution &best_chromosomes, double &error){
    if(!output_file.is_open()){
        output_file.open(k_ga_report_path,std::fstream::app | std::fstream::out);
        if(!output_file){
            std::cout<<"can't open "<<k_ga_report_path<<std::endl;
            exit(-1);
        }

    }
//    output_file
//            <<"step"<<"\t"
//            <<"cost_avg"<<"\t"
//            <<"cost_best"<<"\t";
//    for (int i = 0; i < genes_range_.size(); ++i) {
//        std::string x_best = "x_best" + std::to_string(i);
//        output_file<<x_best<<"\t";
//    }
//    output_file<<"\n";
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    using std::placeholders::_4;
    std::function<void(MySolution&, const std::function<double(void)> &rnd01)> init_genes = std::bind(&Optimization::init_genes,this,_1,_2);
    std::function<bool(const MySolution &p, MyMiddleCost &c)> eval_pose_solution = std::bind(&Optimization::eval_solution,this,_1,_2);
    std::function<MySolution(
            const Generation_Type& last_generation,
            const MySolution& X_base,
            const std::function<double(void)> &rnd01,
            double shrink_scale)> mutate = std::bind(&Optimization::mutate,this,_1,_2,_3,_4);
    std::function<MySolution(
            const MySolution& X1,
            const MySolution& X2,
            const std::function<double(void)> &rnd01)> crossover = std::bind(&Optimization::crossover,this,_1,_2,_3);
//    std::function<MySolution(const MySolution& X_base,
//                             const std::function<double(void)> &rnd01,
//                             double shrink_scale)> mutate_uniform = std::bind(&Optimization::mutate_uniform,this,_1,_2,_3);
    std::function<double(int n_generation,const std::function<double(void)> &rnd01)> default_shrink_scale = std::bind(&Optimization::default_shrink_scale,this,_1,_2);
    EA::Chronometer timer;
    timer.tic();


    ga_obj_.SetInitPopulationManually(init_genes_manually_);
    ga_obj_.problem_mode=EA::GA_MODE::SOGA;
    ga_obj_.multi_threading=true;
    ga_obj_.dynamic_threading=cr_->dynamic_threading;
    ga_obj_.idle_delay_us=0; // switch between threads quickly
    ga_obj_.verbose=false;
    ga_obj_.population=cr_->population;
    ga_obj_.generation_max=generation_max;
    ga_obj_.calculate_SO_total_fitness=calculate_SO_total_fitness;
    ga_obj_.init_genes=init_genes;
    ga_obj_.eval_solution=eval_pose_solution;
//    switch (mutation_method_){
//        case MutationMethod::non_uniform:
            ga_obj_.mutate=mutate;
//            break;
//        case MutationMethod::uniform:
//            ga_obj_.mutate=mutate_uniform;
//    }
    if(mutation_method_ == MutationMethod::mutate_by_chromosomes)
        ga_obj_.get_shrink_scale=default_shrink_scale;
    ga_obj_.crossover=crossover;
    ga_obj_.SO_report_generation=SO_report_generation;
    //TODO check the best_stall_max
    ga_obj_.best_stall_max=cr_->best_stall_max;
    ga_obj_.average_stall_max=cr_->average_stall_max;
    ga_obj_.tol_stall_best=cr_->tol_stall_best;
    ga_obj_.tol_stall_average=cr_->tol_stall_average;
    ga_obj_.elite_count=cr_->elite_count;
    ga_obj_.crossover_fraction=cr_->crossover_fraction;
    ga_obj_.mutation_rate=cr_->mutation_rate;
    ga_obj_.verbose= false;
    std::cout<<"use "<<ga_obj_.N_threads<<" threads"<<std::endl;
    std::string stop_reason = ga_obj_.stop_reason_to_string(ga_obj_.solve());
    if( stop_reason == "Maximum generation reached" || stop_reason == "Average stalled" || stop_reason == "Best stalled"){
        //-0.001798665336,-0.001009620523,-0.0004813350692,-0.03048475668,0.001770099587,0.06871385661
//0.003447213115,0.002631358808,-0.0004589082073,-0.002568362806,-0.0789223138,-0.09657628683
        std::cout<<"The problem is optimized in "<<timer.toc()<<" seconds."<<std::endl;

        output_file.close();
        best_chromosomes = ga_obj_.last_generation.chromosomes[ga_obj_.last_generation.best_chromosome_index].genes;
        error = ga_obj_.last_generation.best_total_cost;
    }

    return ga_obj_.user_request_stop;

}
bool Optimization::SolveGA(MySolution &best_chromosomes, double &error, std::string mutation_method){
    std::string ga_mutation_path = "../result/banana/report_" + mutation_method +".txt";
    if(!output_file.is_open()){
        output_file.open(ga_mutation_path,std::fstream::app | std::fstream::out);
        if(!output_file){
            std::cout<<"can't open "<<ga_mutation_path<<std::endl;
            exit(-1);
        }

    }
//    output_file
//            <<"step"<<"\t"
//            <<"cost_avg"<<"\t"
//            <<"cost_best"<<"\t";
//    for (int i = 0; i < genes_range_.size(); ++i) {
//        std::string x_best = "x_best" + std::to_string(i);
//        output_file<<x_best<<"\t";
//    }
//    output_file<<"\n";
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    using std::placeholders::_4;
    std::function<void(MySolution&, const std::function<double(void)> &rnd01)> init_genes = std::bind(&Optimization::init_genes,this,_1,_2);
    std::function<bool(const MySolution &p, MyMiddleCost &c)> eval_pose_solution = std::bind(&Optimization::eval_solution,this,_1,_2);
    std::function<MySolution(
            const Generation_Type& last_generation,
            const MySolution& X_base,
            const std::function<double(void)> &rnd01,
            double shrink_scale)> mutate = std::bind(&Optimization::mutate,this,_1,_2,_3,_4);
    std::function<MySolution(
            const MySolution& X1,
            const MySolution& X2,
            const std::function<double(void)> &rnd01)> crossover = std::bind(&Optimization::crossover,this,_1,_2,_3);
//    std::function<MySolution(const MySolution& X_base,
//                             const std::function<double(void)> &rnd01,
//                             double shrink_scale)> mutate_uniform = std::bind(&Optimization::mutate_uniform,this,_1,_2,_3);
    std::function<double(int n_generation,const std::function<double(void)> &rnd01)> default_shrink_scale = std::bind(&Optimization::default_shrink_scale,this,_1,_2);
    EA::Chronometer timer;
    timer.tic();


    ga_obj_.SetInitPopulationManually(init_genes_manually_);
    ga_obj_.problem_mode=EA::GA_MODE::SOGA;
    ga_obj_.multi_threading=true;
    ga_obj_.dynamic_threading=cr_->dynamic_threading;
    ga_obj_.idle_delay_us=0; // switch between threads quickly
    ga_obj_.verbose=false;
    ga_obj_.population=cr_->population;
    ga_obj_.generation_max=generation_max;
    ga_obj_.calculate_SO_total_fitness=calculate_SO_total_fitness;
    ga_obj_.init_genes=init_genes;
    ga_obj_.eval_solution=eval_pose_solution;
//    switch (mutation_method_){
//        case MutationMethod::non_uniform:
    if(mutation_method == "non_uniform"){
        mutation_method_ = MutationMethod::non_uniform;
        std::cout<<"non_uniform mutation method"<<std::endl;
    } else
    {
        if(mutation_method == "mutate_by_chromosomes"){
            mutation_method_ = MutationMethod::mutate_by_chromosomes;
            std::cout<<"mutate_by_chromosomes mutation method"<<std::endl;
        } else{
            mutation_method_ = MutationMethod::uniform;
            std::cout<<"uniform mutation method"<<std::endl;
        }


    }
    ga_obj_.mutate=mutate;
//            break;
//        case MutationMethod::uniform:
//            ga_obj_.mutate=mutate_uniform;
//    }
    if(mutation_method_ == MutationMethod::mutate_by_chromosomes)
        ga_obj_.get_shrink_scale=default_shrink_scale;
    ga_obj_.crossover=crossover;
    ga_obj_.SO_report_generation=SO_report_generation;
    //TODO check the best_stall_max
    ga_obj_.best_stall_max=cr_->best_stall_max;
    ga_obj_.average_stall_max=cr_->average_stall_max;
    ga_obj_.tol_stall_best=cr_->tol_stall_best;
    ga_obj_.tol_stall_average=cr_->tol_stall_average;
    ga_obj_.elite_count=cr_->elite_count;
    ga_obj_.crossover_fraction=cr_->crossover_fraction;
    ga_obj_.mutation_rate=cr_->mutation_rate;
    ga_obj_.verbose= false;
    std::cout<<"use "<<ga_obj_.N_threads<<" threads"<<std::endl;
    std::string stop_reason = ga_obj_.stop_reason_to_string(ga_obj_.solve());
    if( stop_reason == "Maximum generation reached" || stop_reason == "Average stalled" || stop_reason == "Best stalled"){
        //-0.001798665336,-0.001009620523,-0.0004813350692,-0.03048475668,0.001770099587,0.06871385661
//0.003447213115,0.002631358808,-0.0004589082073,-0.002568362806,-0.0789223138,-0.09657628683
        std::cout<<"The problem is optimized in "<<timer.toc()<<" seconds."<<std::endl;

        output_file.close();
        best_chromosomes = ga_obj_.last_generation.chromosomes[ga_obj_.last_generation.best_chromosome_index].genes;
        error = ga_obj_.last_generation.best_total_cost;
    }

    return ga_obj_.user_request_stop;

}
void Optimization::set_init_genes_manually(const std::vector<MySolution> init_genes){
    init_genes_manually_ = init_genes;
    for (int j = 0; j < init_genes_manually_.size(); ++j) {
        for (int i = 0; i < init_genes_manually_[j].x.size(); ++i) {
//            std::cout<<"init_genes_manually_["<<j<<"]"<<".x["<<i<<"] = "<<init_genes_manually_[j].x[i]<<std::endl;
//            std::cout<<"genes_range_["<<i<<"][0] = "<<genes_range_[i][0]<<std::endl;
            assert(init_genes_manually_[j].x[i]>=genes_range_[i][0]);
            assert(init_genes_manually_[j].x[i]<=genes_range_[i][1]);
        }
    }
}

void Optimization::CheckGenesRange(const MySolution &p) {
    for (int i = 0; i < p.x.size(); ++i) {
        assert(p.x[i]>=genes_range_[i][0]);
        assert(p.x[i]<=genes_range_[i][1]);
    }
}


