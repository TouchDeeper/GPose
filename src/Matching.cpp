
#include "Matching.h"

#include <TdLibrary/PCL/filter.h>
#include "ga_objective.hpp"
#include <TdLibrary/PCL/tools.h>
#include <TdLibrary/slam_tool/motion_transformation.h>
#include <TdLibrary/PCL/filter.hpp>

//#include "include/trimmed_icp.h"
#include <pcl/registration/icp.h>
#include <pcl/recognition/ransac_based/auxiliary.h>
#include <pcl/recognition/ransac_based/trimmed_icp.h>
#include <pcl/features/fpfh.h>
#include <TdLibrary/PCL/Viewer.h>
#include <TdLibrary/FileOperation/FileOperation.h>
#include <boost/filesystem.hpp>
#include <algorithm>
#include <TdLibrary/tool/tic_toc.h>
#include <TdLibrary/PCL/enveloping.hpp>
#include <nlohmann/json.hpp>
/**
 * load the scene image of dataset
 */
void Matching::LoadSceneImage(){
    std::string data_name = scene_->scene_name + ".png";
    std::string scene_path = cr_->scene_data_path + "depth/" + data_name;
    std::string mask_path = cr_->scene_data_path + "mask_visib/" +"000001_000000.png";
//    boost::filesystem::path data_path;
//    if(!td::find_file(scene_path,data_name,data_path))
//    {
//        std::cout<<"can't find scene data"<<std::endl;
//        exit(-1);
//    }
    cv::Mat depth = cv::imread(scene_path, CV_LOAD_IMAGE_UNCHANGED);
    cv::Mat mask = cv::imread(mask_path, CV_LOAD_IMAGE_UNCHANGED);
    if(depth.empty()){
        std::cout<<"can not read "<<scene_path<<std::endl;
        exit(-1);
    }
    if(mask.empty()){
        std::cout<<"can not read "<<mask_path<<std::endl;
        exit(-1);
    }
    std::cout<<depth<<std::endl;
    cv::namedWindow("depth");
    cv::imshow("depth",depth);
    cv::namedWindow("mask");
    cv::imshow("mask",mask);
    cv::waitKey();
    cv::destroyAllWindows();
    std::string scene_camera_json_path = cr_->scene_data_path + "scene_camera.json";
    std::ifstream scene_camera_json(scene_camera_json_path);
    if(!scene_camera_json)
    {
        std::cout<<"can not find "<<scene_camera_json_path<<std::endl;
        exit(-1);
    }
    nlohmann::json j;
    scene_camera_json >> j;
    auto vec_cam_K = j["1"]["cam_K"].get<std::vector<double>>();

    double fx = vec_cam_K[0];
    double cx = vec_cam_K[2];
    double fy = vec_cam_K[4];
    double cy = vec_cam_K[5];
    auto depth_scale = j["1"]["depth_scale"].get<double>(); //depth to meter
    std::cout<<"fx = "<<fx<<" cx = "<<cx<<" fy = "<<fy<<" cy = "<<cy<<" depth_scale = "<<depth_scale<<std::endl;
    td::pclib::PointCloudPtr scene(new td::pclib::PointCloud);
    depthToPointCloud(depth,mask,scene,fx,fy,cx,cy,depth_scale);
    td::pclib::quickShowCloud<pcl::PointXYZ>(scene,0.3);
    scene_->cloud = scene;
    td::pclib::DownsamplingSquareLeaf(scene_->cloud,LEAF_SIZE);


}
void Matching::depthToPCD(){
    std::string scene_path = cr_->scene_data_path + "depth/";
    std::string mask_path = cr_->scene_data_path + "mask_visib/";
    std::string pcd_visib_path = cr_->scene_data_path + "pcd_visib/";
    boost::filesystem::path p(pcd_visib_path);
    if(!boost::filesystem::exists(p)){
        // Path does not exist, create directory
        if (!boost::filesystem::create_directory(p))
        {
            std::stringstream ss;
            ss << "ERROR: Could not create directory " << p << "!\n\n";
            pcl::console::print_error(ss.str().c_str());
            std::exit (EXIT_FAILURE);
        }
    }
    for (int i = 1; ; ++i) {
        //read depth image
        std::string depth_path = scene_path;
        std::string depth_file_name;
        std::string depth_index = std::to_string(i);
        depth_file_name.append(6-depth_index.size(),'0');
        depth_file_name += depth_index;
//        depth_file_name = (i<=9? "00000" + std::to_string(i): "0000" + std::to_string(i)) ;
        depth_path += depth_file_name;
        cv::Mat depth = cv::imread(depth_path + + ".png", CV_LOAD_IMAGE_UNCHANGED);

        if(depth.empty()){
            std::cout<<"can not read "<<depth_path<<std::endl;
            break;
        }
        std::cout<<"now read "<<depth_file_name<<" .png"<<std::endl;
        // read camera config
        std::string scene_camera_json_path = cr_->scene_data_path + "scene_camera.json";
        std::ifstream scene_camera_json(scene_camera_json_path);
        if(!scene_camera_json)
        {
            std::cout<<"can not find "<<scene_camera_json_path<<std::endl;
            exit(-1);
        }
        nlohmann::json js;
        scene_camera_json >> js;
        auto vec_cam_K = js["1"]["cam_K"].get<std::vector<double>>();

        double fx = vec_cam_K[0];
        double cx = vec_cam_K[2];
        double fy = vec_cam_K[4];
        double cy = vec_cam_K[5];
        auto depth_scale = js["1"]["depth_scale"].get<double>() * 0.001; //depth to meter
        std::cout<<"fx = "<<fx<<" cx = "<<cx<<" fy = "<<fy<<" cy = "<<cy<<" depth_scale = "<<depth_scale<<std::endl;

        //read mask
        for (int j = 0; ; ++j) {
            std::string mask_file_name = depth_file_name + "_";
            std::string mask_index = std::to_string(j);
            mask_file_name.append(6-mask_index.size(), '0');
            mask_file_name += mask_index;
//            mask_file_name += j<=9? "_00000" + std::to_string(j): "_0000"  + std::to_string(j);
            std::string mask_file_path= mask_path + mask_file_name;
            cv::Mat mask = cv::imread(mask_file_path + ".png", CV_LOAD_IMAGE_UNCHANGED);
            if(mask.empty()){
                std::cout<<"can not read "<<mask_file_path<<std::endl;
                break;
            }
            td::pclib::PointCloudPtr scene_object(new td::pclib::PointCloud);
            depthToPointCloud(depth, mask, scene_object, fx, fy, cx, cy, depth_scale);
//            td::pclib::quickShowCloud<pcl::PointXYZ>(scene_object,0.3);
            processScene(scene_object);
//            td::pclib::quickShowCloud<pcl::PointXYZ>(scene_object,0.3);
            std::string pcd_file = pcd_visib_path + mask_file_name + ".pcd";
            pcl::io::savePCDFileASCII(pcd_file, *scene_object);
        }


    }
}

void Matching::depthToPointCloud(cv::Mat& depth, cv::Mat& mask, td::pclib::PointCloudPtr& point_cloud, double fx, double fy, double cx, double cy, double scale){
    assert(depth.size == mask.size);

    for (size_t y = 0; y < depth.rows; ++y) {
        for (size_t x = 0; x < depth.cols; ++x) {
            unsigned char mask_data = mask.ptr<unsigned char>(y)[x];
            if(mask_data)
            {
                unsigned int depth_data = depth.ptr<unsigned short>(y)[x];
                if(depth_data>0){
                    pcl::PointXYZ p;
                    p.x = (double(x)-cx)/fx * depth_data * scale;
                    p.y = (double(y)-cy)/fy * depth_data * scale;
                    p.z = depth_data * scale;
                    point_cloud->push_back(p);
                }

            }
        }
    }
}
void Matching::processScene(td::pclib::PointCloudPtr& scene){
    td::pclib::DownsamplingSquareLeaf(scene,cr_->leaf_size);
    td::pclib::StatisticalFilter(scene,cr_->mean_k,cr_->std_dev_mul_thresh);
    td::pclib::smoothCloud<pcl::PointXYZ>(scene,cr_->search_radius_mls);
    td::pclib::euclideanClusterExtraction<pcl::PointXYZ>(scene,cr_->cluster_tolerance, cr_->min_cluster_size, cr_->max_cluster_size);

}
/**
 * load the scene model, downsampling
 */
void Matching::LoadScene() {
    std::string scene_path;
//    scene_path = SCENE_DATA_PATH + "/"+model_->name;
    scene_path = cr_->scene_data_path + "scene_data/"+model_->name;
    boost::filesystem::path data_path;
    std::string data_name = scene_->scene_name + ".pcd";
    if(!td::find_file(scene_path,data_name,data_path)){
        std::cout<<"can't find scene data"<<std::endl;
        exit(-1);
    }
    std::cout<<"loading scene:"<<scene_path<<std::endl;
    td::pclib::PointCloudPtr cloud(new td::pclib::PointCloud);

    scene_->cloud = cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(data_path.string(), *(scene_->cloud)) != 0)
    {
        std::cout<<"can't load scene model"<<std::endl;
        exit(-1);
    }
//    std::cout<<"transform the units from mm to m"<<std::endl;
    // Transformation matrix object, initialized to the identity matrix
//    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
//    transformation(0,0) = 0.001;
//    transformation(1,1) = 0.001;
//    transformation(2,2) = 0.001;
//    pcl::transformPointCloud(*scene_model_, *scene_model_, transformation);//convert the mm to m;

//    std::cout<<"downsample,set the size of every voxel to be 3mmx3mmx3mm"<<std::endl;
    // downsampling
    td::pclib::DownsamplingSquareLeaf(scene_->cloud,LEAF_SIZE);
    LoadCheckResult();

}
void Matching::LoadCheckResult() {
    std::string scene_path;
    scene_path = cr_->base_dir + "scene_data/"+cr_->model_name;
    boost::filesystem::path data_path;
    std::string data_name = cr_->scene_name+ ".txt";

    if(!td::find_file(scene_path,data_name,data_path))
    {
        std::cout<<"can't find pose file"<<std::endl;
        exit(-1);
    }
    std::ifstream result_file;
    result_file.open(data_path.string(), std::ifstream::in);
    if(!result_file)
    {
        std::cerr<<"cannot open "<<data_path.string()<<std::endl;
        exit(1);
    }
    std::string line;
    while (std::getline(result_file, line) && !line.empty()) {
//        Eigen::Matrix4d check_result = Eigen::Matrix4d::Identity();
        td::Vec6 check_result;
        std::istringstream line_data(line);
        line_data >> scene_->specified_view >> check_result(0) >> check_result(1) >> check_result(2) >> check_result(3) >>
                  check_result(4) >> check_result(5);
        CHECK_RESULT.push_back(check_result);
    }

}
/**
 * load the model cloud with normal, local descriptors, global descriptors
 * @param model_path the scene_path
 */
void Matching::LoadModel() {

    int i = 0;
    bool load = true;
    std::cout<<"loading "<<cr_->model_name<<std::endl;
//    std::string base_dir_ = "../data/model_data/";
    ModelData::Ptr model_data(new ModelData);
    model_data->name = cr_->model_name;
    td::pclib::EsfDescriptorCloudPtr esf_descriptors(new td::pclib::EsfDescriptorCloud);
    model_data->esf_descriptors = esf_descriptors;
    while (load)
    {
        std::string view_N_original_pose_path = cr_->model_data_path + "/"+ model_data->name + "/views_original_pose/view" + std::to_string(i) + ".pcd";
        td::pclib::PointNCloudPtr point_normal(new td::pclib::PointNCloud);
        if(pcl::io::loadPCDFile<td::pclib::PointN>(view_N_original_pose_path, *point_normal)!=0)
            load  = false;
        else
            model_data->cloud_N_original_pose_vec.push_back(point_normal);

        std::string view_N_cam_path = cr_->model_data_path+ "/"+ model_data->name + "/views/view" + std::to_string(i) + ".pcd";
        td::pclib::PointNCloudPtr view_N_cam(new td::pclib::PointNCloud);
        if(pcl::io::loadPCDFile<td::pclib::PointN>(view_N_cam_path, *view_N_cam)!=0)
            load  = false;
        else
            model_data->cloud_N_cam_vec.push_back(view_N_cam);
//        td::pclib::quickShowCloud<pcl::PointNormal>(view_N_cam,0.3);
        std::string model_fpfh_descripors_path = cr_->model_data_path+ "/" + model_data->name + "/local_features/view" + std::to_string(i) + ".pcd";
        td::pclib::FpfhDescriptorCloudPtr fpfh_descripor(new td::pclib::FpfhDescriptorCloud);
        if(pcl::io::loadPCDFile<td::pclib::FpfhDescriptor>(model_fpfh_descripors_path, *fpfh_descripor)!=0)
            load  = false;
        else
            model_data->fpfh_descriptors_vector.push_back(fpfh_descripor);

        i++;
    }

    std::cout<<"loading " << model_data->name<<" global esf feature"<<std::endl;
    std::string model_esf_path = cr_->model_data_path + "/" + model_data->name + "/global_features.pcd";
    if (pcl::io::loadPCDFile<td::pclib::EsfDescriptor>(model_esf_path, *(model_data->esf_descriptors)) != 0)
    {
        std::cout<<"can't load global esf feature"<<std::endl;
        exit(-1);
    }
    model_data->view_graph->load_graph(model_data->name);
    if(fusion_level_ > 0)
    {
        std::vector<int> indice;
        ac_->load_fusion_model_views(model_data->name,model_data->fusion_cloud,indice,fusion_level_);
    } else{
        model_data->fusion_cloud = model_data->cloud_N_original_pose_vec;
    }

    model_ = std::move(model_data);
    ac_->load_complete_model(model_->name,model_->complete_model);
//    td::pclib::BoundingBoxSize<pcl::PointXYZ>(model_->complete_model,model_->bound_box,true);
    pcl::PointXYZ min_p1, max_p1;   //点云的最大值与最小值点
    pcl::getMinMax3D(*model_->complete_model, min_p1, max_p1 );
    model_->bound_box = max_p1.getVector3fMap() - min_p1.getVector3fMap();
//    std::cout<<"x_width = "<<model_->bound_box(0)<<"  y_width = "<<model_->bound_box(1)<<"  z_width = "<<model_->bound_box(2)<<std::endl;
    std::cout<<model_->name<<" load done"<<std::endl;
}
/**
 * compute the scene descriptors
 */
void Matching::SceneLocalDescriptorsTrain() {
    if(local_descriptors_type_ == "fpfh")
    {
        std::cout<<"estimate the"<< typeid(td::pclib::FpfhDescriptor).name() << "descriptor"<<std::endl;
        // Estimate the scene normals.
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
        normal_estimation.setInputCloud(scene_->cloud);
        normal_estimation.setViewPoint(0,0,0);
        normal_estimation.setKSearch(cr_->k_search_normals);
//        normal_estimation.setRadiusSearch(0.03);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr normal_kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
        normal_estimation.setSearchMethod(normal_kdtree);
        normal_estimation.compute(*scene_->normal);


        pcl::copyPointCloud(*(scene_->cloud), *(scene_->cloud_with_normal));
        pcl::copyPointCloud(*scene_->normal, *(scene_->cloud_with_normal));
//        PointNormalVisual(scene_->cloud_with_normal);


        td::pclib::LocalDescriptorEstimation(scene_->cloud_with_normal,scene_->fpfh_descriptors_ptr,cr_->pose_feature_radius_search);
    }
    if (local_descriptors_type_ == "shot")
    {

    }


}
void Matching::LocalPipeMatch(){

//    std::cout<<"estimate the"<< typeid(td::pclib::FpfhDescriptor).name() << "descriptor"<<std::endl;
//
//    // Estimate the scene normals.
//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
//    normal_estimation.setInputCloud(scene_->cloud);
//    normal_estimation.setViewPoint(0,0,0);
//    normal_estimation.setKSearch(cr_->k_search_normals);
////        normal_estimation.setRadiusSearch(0.03);
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr normal_kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
//    normal_estimation.setSearchMethod(normal_kdtree);
//    normal_estimation.compute(*scene_->normal);
//
//
//    pcl::copyPointCloud(*(scene_->cloud), *(scene_->cloud_with_normal));
//    pcl::copyPointCloud(*scene_->normal, *(scene_->cloud_with_normal));
////    if(VISUAL)
////        PointNormalVisual(scene_->cloud_with_normal);
    SceneLocalDescriptorsTrain();
    // 计算boundary
    scene_->TrainKdtree();
    scene_->GetBoundary(cr_->pose_feature_radius_search-0.002); //0.005
//    local_pipe_.DescriptorsTrain(scene_->cloud_with_normal, scene_->fpfh_descriptors_ptr, cr_->pose_feature_radius_search);
    for (int k = 0; k < model_->correspondences.size(); ++k) {
        LocalPipe local_pipe_;
        int view_candidate_indice = model_->correspondences[k].index_query;

//        td::pclib::PointNCloudPtr partial_model = model_->cloud_N_original_pose_vec[view_candidate_indice];
//        pcl::visualization::PCLVisualizer viewer("candidate view");
//        viewer.addPointCloud<td::pclib::PointN>(partial_model,"candidate_view");
//        viewer.spin();
//        CandidateAddNeighbors(view_candidate_indice, partial_model);

//        td::pclib::FpfhDescriptorCloudPtr descriptors_candidate = model_->fpfh_descriptors_vector[view_candidate_indice];
        if (VERBOSE)
            pcl::console::print_highlight("Estimating pose in correspondence view %d\n",view_candidate_indice);
        td::pclib::PointNCloudPtr object_aligned(new td::pclib::PointNCloud);
//        td::pclib::PointNCloudPtr fusion_partial_model;
//        fusion_partial_model = model_->fusion_cloud[view_candidate_indice];
//        local_pipe_.SetFusionPartialCloud(fusion_partial_model);
//        local_pipe_.SetFusionPartialCloud(partial_model);
//        td::TicToc transfer_timer;
        local_pipe_.TransferData(model_, scene_, view_candidate_indice);
//        std::cout<<"transfer take "<<transfer_timer.toc()<<" ms"<<std::endl;
//        local_pipe_.EstimatePose(partial_model, descriptors_candidate, scene_->cloud_with_normal,
//                                 scene_->fpfh_descriptors_ptr, object_aligned);
        local_pipe_.EstimatePose(object_aligned);
        Result::Ptr ga_result(new Result);
        Result::Ptr ga_icp_result(new Result);
        Result::Ptr icp_result(new Result);
        local_pipe_.GetResult(icp_result,ga_result,ga_icp_result);
        GA_results_.push_back(ga_result);
        GA_ICP_results_.push_back(ga_icp_result);
        ICP_results_.push_back(icp_result);
//        std::cout<<"this views final fitness = "<<ga_result->final_fitness<<std::endl;
        if(GA_results_.size() == 2)
        {
            int better_index = GA_results_[0]->final_fitness < GA_results_[1]->final_fitness ? 0 : 1;
            double relative_error = (GA_results_[1-better_index]->final_fitness - GA_results_[better_index]->final_fitness)/GA_results_[1-better_index]->final_fitness;
            if(relative_error > 0.4)
            {
                m_T_s_ = GA_ICP_results_[better_index]->pose;
                OutputResult(ICP_results_[better_index], ICP_PATH);
                OutputResult(GA_results_[better_index], PURE_GA_PATH);
                OutputResult(GA_ICP_results_[better_index], GA_ICP_PATH);

                if(VISUAL)
                    td::pclib::ShowAlignResult(scene_->cloud_with_normal, GA_results_[better_index]->fusion_partial_cloud, m_T_s_,"global final result", VIEWSTYLE.model_color, VIEWSTYLE.scene_color,WINDOW_SIZE, VIEWSTYLE.point_size, VIEWSTYLE.background_color);
                std::cout<<"omit remaining view"<<std::endl;
                return;
            }
        }
    }
    int final_index = 0;
    double fitness = GA_results_[0]->final_fitness;
    for (int i = 0; i < GA_results_.size(); ++i) {
        if(GA_results_[i]->final_fitness < fitness)
        {
            fitness = GA_results_[i]->final_fitness;
            final_index = i;
        }
    }
    m_T_s_ = GA_ICP_results_[final_index]->pose;
    if(!LOCAL_FLAG){
        OutputResult(ICP_results_[final_index], ICP_PATH);
        OutputResult(GA_results_[final_index], PURE_GA_PATH);
        OutputResult(GA_ICP_results_[final_index], GA_ICP_PATH);

    }
    if(VISUAL)
        td::pclib::ShowAlignResult(scene_->cloud_with_normal, GA_results_[final_index]->fusion_partial_cloud, m_T_s_,"global final result", VIEWSTYLE.model_color, VIEWSTYLE.scene_color,WINDOW_SIZE, VIEWSTYLE.point_size, VIEWSTYLE.background_color);



}
void Matching::CandidateAddNeighbors(const int &view_candidate_indice, td::pclib::PointNCloudPtr partial_model) {
    pcl::visualization::PCLVisualizer viewer("partial_model view");
    viewer.addPointCloud<td::pclib::PointN>(partial_model,"partial_model");
    std::vector<int> neighbor_index = model_->view_graph->get_neighbors(view_candidate_indice);
    for (int i = 0; i < neighbor_index.size(); ++i) {
        std::cout<<"neighbor_index = "<<neighbor_index[i]<<std::endl;
        td::pclib::PointNCloudPtr neighbor_view = model_->cloud_N_original_pose_vec[neighbor_index[i]];
        *partial_model += *neighbor_view;
        std::string neighbor = "partial_model" + std::to_string(i);
        auto r = td::UniformSampling<double>(0,255);
        auto g = td::UniformSampling<double>(0,255);
        auto b = td::UniformSampling<double>(0,255);
        viewer.addPointCloud<td::pclib::PointN>(neighbor_view,td::pclib::ColorHandlerPointN(neighbor_view,r,g,b),neighbor);
    }


    viewer.spin();


}
//show the candidate euler angle as point cloud
void Matching::ShowCandidate(){

    td::pclib::PointNCloudPtr best_match_view = model_->cloud_N_original_pose_vec[3];

    Eigen::Vector3d s_euler_angle_o1 = Eigen::Vector3d(0.000549608,-3.1334,1.23193);
    Eigen::Vector3d s_euler_angle_o2 = Eigen::Vector3d(0.0222646,-3.10948,1.31032);
    Eigen::Vector3d s_t_o1 = Eigen::Vector3d(0.0148442,0.0453362,0.342611);
    Eigen::Vector3d s_t_o2 = Eigen::Vector3d(0.0213847,0.0488918,0.341841);

    Sophus::SE3d s_T_o1 = td::EulerTranslatetoSE3(s_euler_angle_o1,s_t_o1);
    Sophus::SE3d s_T_o2 = td::EulerTranslatetoSE3(s_euler_angle_o2,s_t_o2);
    Sophus::SE3d o1_T_o2 = s_T_o1.inverse() * s_T_o2;
    Eigen::Matrix3d o1_R_o2 = o1_T_o2.rotationMatrix();
    std::cout<<"o1_R_o2\n"<<o1_R_o2<<std::endl;
    Eigen::Vector3d o1_euler_angle_o2_my = td::RotationToEulerAngle(o1_R_o2);

    std::cout<<"o1_euler_angle_o2 by me = "<<o1_euler_angle_o2_my.transpose()<<std::endl;
    Eigen::Matrix3d o1_R_o2_my = (Eigen::AngleAxisd(o1_euler_angle_o2_my[0],Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(o1_euler_angle_o2_my[1],Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(o1_euler_angle_o2_my[2],Eigen::Vector3d::UnitX())).matrix();
    std::cout<<"o1_R_o2_my\n"<<o1_R_o2_my<<std::endl;

    Eigen::AngleAxisd o1_angle_axis_o2;
    o1_angle_axis_o2.fromRotationMatrix(o1_R_o2);
    Eigen::Vector3d o1_euler_angle_o2 = o1_R_o2.eulerAngles(2,1,0);
    std::cout<<"o1_euler_angle_o2:\n"<<o1_euler_angle_o2<<std::endl;
    std::cout<<"o1_angle_axis_o2:\n"<<o1_angle_axis_o2.angle()<<std::endl<<o1_angle_axis_o2.axis()<<std::endl;
    td::pclib::PointNCloudPtr best_match_view_transformed1(new td::pclib::PointNCloud);
    td::pclib::PointNCloudPtr best_match_view_transformed2(new td::pclib::PointNCloud);

    pcl::transformPointCloud(*best_match_view,*best_match_view_transformed1,s_T_o1.matrix());
    pcl::transformPointCloud(*best_match_view,*best_match_view_transformed2,s_T_o2.matrix());

    pcl::visualization::PCLVisualizer viewer ("candidate pose");
//    if(COLOR_STYLE == "print")
        viewer.setBackgroundColor(VIEWSTYLE.background_color[0],VIEWSTYLE.background_color[1],VIEWSTYLE.background_color[2]);
    viewer.addPointCloud<td::pclib::PointN>(best_match_view_transformed1,td::pclib::ColorHandlerPointN (best_match_view_transformed1, 0.0, 0.0, 255.0),"first candidate");
    viewer.addPointCloud<td::pclib::PointN>(best_match_view_transformed2,td::pclib::ColorHandlerPointN (best_match_view_transformed2, 0.0, 255.0, 0.0),"second candidate");
    viewer.spin();
}
void Matching::CheckResults(std::string result_file_path){

    td::pclib::PointNCloudPtr best_match_view = model_->cloud_N_original_pose_vec[48];
    std::ifstream result_file;
    result_file.open(result_file_path,ios::in);
    if(!result_file.is_open())
    {
        std::cerr<<"result_file doesn't exist"<<std::endl;
    }
    std::string line;
    int line_index = 0;
    while (std::getline(result_file, line) && !line.empty()) {

        std::istringstream ssResultData(line);
        bool is_right;
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        double error;
        double solve_time;
        double inlier_fraction;
        double error_decline_fraction;
//        Eigen::Vector4d point;//useless
        ssResultData >> is_right;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 3; ++j) {
                ssResultData >> pose(j,i);
            }

        }

        ssResultData >> error >> inlier_fraction >> error_decline_fraction >> solve_time;
        std::cout<<"reading line "<<line_index+1<<std::endl;
        if(is_right)
            std::cout<<"right pose  \n"<<pose<<std::endl;
        else
            std::cout<<"wrong pose  \n"<<pose<<std::endl;
        std::cout<<"final_fitness = "<<error<<"  final_inlier_fraction = "<<inlier_fraction<<" error_decline_fraction = "<<error_decline_fraction<<" solve_time = "<<solve_time<<std::endl;
//        if(COLOR_STYLE == "print")
            td::pclib::ShowAlignResult(scene_->cloud_with_normal,best_match_view,pose, "align result", VIEWSTYLE.scene_color,VIEWSTYLE.model_color,WINDOW_SIZE,VIEWSTYLE.point_size,VIEWSTYLE.background_color);
//        else
//            td::pclib::ShowAlignResult(best_match_view,scene_->cloud_with_normal,pose, "align result",WINDOW_SIZE,POINT_SIZE);

        line_index ++;
    }
    result_file.close();
//    Eigen::Vector3d s_euler_angle_o1 = Eigen::Vector3d(0.000549608,-3.1334,1.23193);
//    Eigen::Vector3d s_euler_angle_o2 = Eigen::Vector3d(0.0222646,-3.10948,1.31032);
//    Eigen::Vector3d s_t_o1 = Eigen::Vector3d(0.0148442,0.0453362,0.342611);
//    Eigen::Vector3d s_t_o2 = Eigen::Vector3d(0.0213847,0.0488918,0.341841);
//
//    Sophus::SE3d s_T_o1 = td::EulerTranslatetoSE3(s_euler_angle_o1,s_t_o1);
//    Sophus::SE3d s_T_o2 = td::EulerTranslatetoSE3(s_euler_angle_o2,s_t_o2);
//    Sophus::SE3d o1_T_o2 = s_T_o1.inverse() * s_T_o2;
//    Eigen::Matrix3d o1_R_o2 = o1_T_o2.rotationMatrix();
//    std::cout<<"o1_R_o2\n"<<o1_R_o2<<std::endl;
//    Eigen::Vector3d o1_euler_angle_o2_my = td::RotationToEulerAngle(o1_R_o2);
//
//    std::cout<<"o1_euler_angle_o2 by me = "<<o1_euler_angle_o2_my.transpose()<<std::endl;
//    Eigen::Matrix3d o1_R_o2_my = (Eigen::AngleAxisd(o1_euler_angle_o2_my[0],Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(o1_euler_angle_o2_my[1],Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(o1_euler_angle_o2_my[2],Eigen::Vector3d::UnitX())).matrix();
//    std::cout<<"o1_R_o2_my\n"<<o1_R_o2_my<<std::endl;
//
//    Eigen::AngleAxisd o1_angle_axis_o2;
//    o1_angle_axis_o2.fromRotationMatrix(o1_R_o2);
//    Eigen::Vector3d o1_euler_angle_o2 = o1_R_o2.eulerAngles(2,1,0);
//    std::cout<<"o1_euler_angle_o2:\n"<<o1_euler_angle_o2<<std::endl;
//    std::cout<<"o1_angle_axis_o2:\n"<<o1_angle_axis_o2.angle()<<std::endl<<o1_angle_axis_o2.axis()<<std::endl;
//    PointNCloudPtr best_match_view_transformed1(new PointNCloud);
//    PointNCloudPtr best_match_view_transformed2(new PointNCloud);
//
//    pcl::transformPointCloud(*best_match_view,*best_match_view_transformed1,s_T_o1.matrix());
//    pcl::transformPointCloud(*best_match_view,*best_match_view_transformed2,s_T_o2.matrix());
//
//    pcl::visualization::PCLVisualizer viewer ("candidate pose");
//    viewer.addPointCloud<PointN>(best_match_view_transformed1,ColorHandlerPointN (best_match_view_transformed1, 0.0, 0.0, 255.0),"first candidate");
//    viewer.addPointCloud<PointN>(best_match_view_transformed2,ColorHandlerPointN (best_match_view_transformed2, 0.0, 255.0, 0.0),"second candidate");
//    viewer.spin();
}
void Matching::CheckViewGraph() {
    int graph_size = model_->view_graph->get_size();
//    Render_Synthetic_Views render;
    ac_->load_complete_model(model_->name,model_->complete_model);

    graph_viewer(model_->view_graph,model_->complete_model);
//    td::pclib::PointNCloudPtr complete_model(new td::pclib::PointNCloud);
//    for (int i = 0; i < graph_size; ++i) {
//
////        pcl::transformPointCloud(*(model_->cloud_N_original_pose_vec[i]),*cn_view,cn_T_w_sop.inverse().matrix());
//        *complete_model+=*(model_->cloud_N_original_pose_vec[i]);
//    }
//    pcl::visualization::PCLVisualizer viewer ("complete model");
//    viewer.addCoordinateSystem(0.1);
//    viewer.addPointCloud<td::pclib::PointN>(complete_model,"complete_model");
//    viewer.spin();

//    complete_model->clear();
//    pcl::visualization::PCLVisualizer viewer2 ("complete model2");
//    for (int i = 0; i < graph_size; ++i) {
//        td::pclib::PointNCloudPtr cn_view(new td::pclib::PointNCloud);
//        td::pclib::Point view_point = model_->view_graph->get_viewpoint(i);
//        Eigen::Vector3d w_t_cn(view_point.x,view_point.y,view_point.z);
//        Sophus::SE3d w_T_cn_sop(model_->view_graph->get_rotation(i),w_t_cn);
//
//        pcl::transformPointCloud(*(model_->cloud_N_cam_vec[i]),*cn_view,w_T_cn_sop.matrix());
//        *complete_model+=*cn_view;
//    }
//    viewer2.addPointCloud<td::pclib::PointN>(complete_model,"complete_model2");
//    viewer2.spin();

}
void
Matching::graph_viewer (View_Graph::Ptr graph, td::pclib::PointCloudPtr complete_model)
{
    // Add graph to viewer
    pcl::visualization::PCLVisualizer viewer ("Viewer");
    viewer.setBackgroundColor(VIEWSTYLE.background_color[0],VIEWSTYLE.background_color[1],VIEWSTYLE.background_color[2]);
    graph->add_graph_to_viewer (viewer, 0.005, 0, false, false);

    // Add complete model to viewer
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(complete_model,VIEWSTYLE.model_rgb[0],VIEWSTYLE.model_rgb[1],VIEWSTYLE.model_rgb[2]);
    viewer.addPointCloud<pcl::PointXYZ> (complete_model,rgb);
    viewer.addCoordinateSystem(0.05);
    std::cout << "Press Q to continue..." << std::endl;
    viewer.spin ();
}



void Matching::FusionViews(std::vector<int> &multi_view_indexs) {
    td::pclib::PointNCloudPtr complete_model(new td::pclib::PointNCloud);
    pcl::visualization::PCLVisualizer viewer ("fusion views");
//    for (int i = 0; i < multi_view_indexs.size(); ++i) {
        td::pclib::PointNCloudPtr fusion_view0 = model_->cloud_N_original_pose_vec[multi_view_indexs[0]];
//        pcl::transformPointCloud(*(model_->cloud_N_original_pose_vec[i]),*cn_view,cn_T_w_sop.inverse().matrix());
        *complete_model+=*fusion_view0;
        std::string point_cloud_id = "view0";
//        auto r = td::UniformSampling<double>(0,255);
//        auto g = td::UniformSampling<double>(0,255);
//        auto b = td::UniformSampling<double>(0,255);
        viewer.addPointCloud<td::pclib::PointN>(fusion_view0,td::pclib::ColorHandlerPointN (fusion_view0, 0, 0, 255),point_cloud_id);
    td::pclib::PointNCloudPtr fusion_view1 = model_->cloud_N_original_pose_vec[multi_view_indexs[1]];
    std::string point_cloud_id1 = "view1";
    viewer.addPointCloud<td::pclib::PointN>(fusion_view1,td::pclib::ColorHandlerPointN (fusion_view0, 0, 255, 0),point_cloud_id1);

//    }

    viewer.addCoordinateSystem(0.1);

    viewer.spin();
//    complete_model->clear();
//    pcl::visualization::PCLVisualizer viewer2 ("complete model2");
//    for (int i = 0; i < graph_size; ++i) {
//        td::pclib::PointNCloudPtr cn_view(new td::pclib::PointNCloud);
//        td::pclib::Point view_point = model_->view_graph->get_viewpoint(i);
//        Eigen::Vector3d w_t_cn(view_point.x,view_point.y,view_point.z);
//        Sophus::SE3d w_T_cn_sop(model_->view_graph->get_rotation(i),w_t_cn);
//
//        pcl::transformPointCloud(*(model_->cloud_N_cam_vec[i]),*cn_view,w_T_cn_sop.matrix());
//        *complete_model+=*cn_view;
//    }
//    viewer2.addPointCloud<td::pclib::PointN>(complete_model,"complete_model2");
//    viewer2.spin();

}
/**
 * compute scene global descriptor
 */
void Matching::SceneGlobalDescriptorTrain(){
    if(global_descriptors_type_ == "esf")
    {
        td::pclib::GlobalDescriptorEstimation(scene_->cloud, scene_->global_feature);
    }
}

///**
// * compute model global descriptor
// */
//void Matching::ModelGlobalDescriptorTrain(){
//    if(global_descriptors_type_ == "esf")
//    {
//        td::pclib::GlobalDescriptorEstimation(model_, model_esf_descriptor_);
//    }
//}

/**
 * match the descriptors
 */
void Matching::MatchLocalDescriptors() {
    float MaxCorrespondence = 0;
    for (int j = 0; j < model_->fpfh_descriptors_vector.size(); ++j) {
        // A kd-tree object that uses the FLANN library for fast search of nearest neighbors.
        // pcl::KdTreeFLANN<pcl::SHOT352> matching;
        pcl::KdTreeFLANN<td::pclib::FpfhDescriptor> matching;
        matching.setInputCloud(model_->fpfh_descriptors_vector[j]);
        // A Correspondence object stores the indices of the query and the match,and the distance/weight.
        pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
        float squareDistancesSum = 0;
        // Check every descriptor computed for the scene.
        for (size_t i = 0; i < scene_->fpfh_descriptors_ptr->size(); ++i)
        {
            std::vector<int> neighbors(1);
            std::vector<float> square_distances(1);
            // Find the nearest neighbor (in descriptor space)...
            int neighborCount = matching.nearestKSearch(scene_->fpfh_descriptors_ptr->at(i), 1, neighbors, square_distances);
            // ...and add a new correspondence if the distance is less than a threshold
            // (SHOT distances are between 0 and 1, other descriptors use different metrics).
//            cout<<"neighborCount="<<neighborCount<<","<<"squaredDistances="<<square_distances[0]<<endl;

            squareDistancesSum += square_distances[0];
            if (neighborCount == 1 && square_distances[0] < 0.25f)
            {
                pcl::Correspondence correspondence(neighbors[0], static_cast<int>(i), square_distances[0]);
                correspondences->push_back(correspondence);
            }
        }
        float squareDistanceAverage = squareDistancesSum / scene_->fpfh_descriptors_ptr->size();
        std::cout<<"sceneDescriptors->size = "<<scene_->fpfh_descriptors_ptr->size()<<std::endl;

        std::cout << "Found " << correspondences->size() << " correspondences." << std::endl;
        if(correspondences->size() > MaxCorrespondence)
        {
            MaxCorrespondence = correspondences->size();
            best_match_model_snapshots_ = j;
        }
        std::cout<<"best_match_model_snapshots id = "<<best_match_model_snapshots_<<std::endl<<MaxCorrespondence<<" correspondence"<<std::endl;
    }

}
/**
 * match the global descriptors
 */
void Matching::MatchGlobalDescriptors()
{
//    for (int i = 0; i < model_->size(); i++)
//    {
    // L1 norm
    std::vector<pcl::Correspondence> model_cluster_corrs;
    for (int j = 0; j < model_->esf_descriptors->points.size(); j++)
    {
        float diff = l1_norm (scene_->global_feature, model_->esf_descriptors->points[j]);
//        cout<<"distance of view"<<j<<" = "<<diff<<std::endl;
        pcl::Correspondence corr (j, static_cast<int> (1), diff);
        model_cluster_corrs.push_back (corr);
        std::cout<<"candidate view index = "<<corr.index_query<<"  distance = "<<corr.distance<<std::endl;

    }
    if(model_cluster_corrs.empty())
    {
        std::cout<<"can't find a view, please capture a new snapshots in a new viewpoint"<<std::endl;
        return;
    } else{
        std::sort(model_cluster_corrs.begin(), model_cluster_corrs.end(),less_than_distance_struct());
        model_cluster_corrs.resize(20);
        // Sort model_cluster_corrs in ascending order with respect to the query
//        std::sort(model_cluster_corrs.begin(), model_cluster_corrs.end(), less_than_query());
        model_->correspondences.TransferData(model_cluster_corrs, model_->view_graph, model_->complete_model) ;
//        model_->correspondences.sort_index_query();
//        model_->ShowCorrespondenceFilter();
        model_->correspondences.NetCluster();
        for (int i = 0; i < model_->correspondences.size(); ++i) {
            std::cout<<"view " <<model_->correspondences[i].index_query <<" distance = "<<model_->correspondences[i].distance<<std::endl;
        }
//        if(VISUAL)

//        model_->correspondences.NetVertexFilter();

//        GlobalCorrespondenceFilter();

//        std::cout<<"best match is view "<<model_->correspondences[0].index_query<<" distance = "<<model_->correspondences[0].distance<<std::endl;

    }


}

void Matching::GlobalCorrespondenceFilter(){
    std::vector<int> neighbor_num_in_correpsondences;
    neighbor_num_in_correpsondences.resize(model_->correspondences.size(),0);
    std::vector<double> neighbor_average_distance;
    neighbor_average_distance.resize(model_->correspondences.size(),0);
    for (int k = 0; k < model_->correspondences.size(); ++k) {
        neighbor_average_distance[k] += model_->correspondences[k].distance;
    }

    for (int i = 0; i < model_->correspondences.size(); ++i) {
        std::cout<<"candidate view index = "<<model_->correspondences[i].index_query<<std::endl;
        std::vector<int> neighbor = model_->view_graph->get_neighbors(model_->correspondences[i].index_query);
        for (int j = 0; j < neighbor.size(); ++j) {
            std::cout<<"    neighbor view"<<neighbor[j];
            auto it = std::lower_bound(model_->correspondences.correspondences_index_query_.begin(), model_->correspondences.correspondences_index_query_.end(),neighbor[j]);
//            for (int k = 0; k < model_->correspondences.size(); ++k) {
//                if(neighbor[j] == model_->correspondences[k].index_query)
//                    neighbor_num_in_correpsondences[i]++;
//            }
            if(it != model_->correspondences.correspondences_index_query_.end() && *it == neighbor[j]){
                neighbor_num_in_correpsondences[i]++;
                neighbor_average_distance[i] += model_->correspondences[it - model_->correspondences.correspondences_index_query_.begin()].distance;
                std::cout<<" has neighbor"<<model_->correspondences.correspondences_index_query_[it - model_->correspondences.correspondences_index_query_.begin()]<<" in candidate views"<<std::endl;
            } else{
                std::cout<<"  has no neighbor in candidate views"<<std::endl;
            }

        }
    }
    for (int m = 0; m < neighbor_num_in_correpsondences.size(); ++m) {
        neighbor_average_distance[m] /= neighbor_num_in_correpsondences[m];

    }
    int most_neighbor_corr_index = -1;
    int max_neighbor_corr = 0;
    for (int l = 0; l < neighbor_num_in_correpsondences.size(); ++l) {
        if(neighbor_num_in_correpsondences[l] > max_neighbor_corr || (neighbor_num_in_correpsondences[l] == max_neighbor_corr && neighbor_average_distance[l] < neighbor_average_distance[most_neighbor_corr_index]))
        {
            max_neighbor_corr = neighbor_num_in_correpsondences[l];
            most_neighbor_corr_index = l;
        }
//        if(neighbor_num_in_correpsondences[l] == max_neighbor_corr)
//        {
//            if(neighbor_average_distance[l] < neighbor_average_distance[most_neighbor_corr_index])
//            {
//                max_neighbor_corr = neighbor_num_in_correpsondences[l];
//                most_neighbor_corr_index = l;
//            }
//        }

    }

    pcl::Correspondence final_view;
    if(most_neighbor_corr_index == -1)//all have no neighbor in candidate view
    {
        // Sort model_cluster_corrs in ascending order with respect to the distance
        model_->correspondences.sort_by_distance();
        final_view = model_->correspondences[0];
    } else{
        final_view = model_->correspondences[most_neighbor_corr_index];

    }
    model_->correspondences.clear();
    model_->correspondences.push_back(final_view);

}
void Matching::SetSpecifiedGlobalResult(){


    pcl::Correspondence specified_cor;
    specified_cor.index_query = scene_->specified_view;
    model_->correspondences.push_back(specified_cor);
    LOCAL_FLAG = true;

}

/**
  Computes the l1-norm of the difference between two histograms
  @param f1 The first histogram
  @param f2 the second histogram
*/
float Matching::l1_norm (td::pclib::EsfDescriptor f1, td::pclib::EsfDescriptor f2)
{
    float sum = 0.0f;
    for (int i = 0; i < 640; i++)
    {
        sum += std::abs (f1.histogram[i] - f2.histogram[i]);
    }
    return sum;
}

/**
 * get the poses of snapshots
 * @return vector for storing the pose of snapshot
 */
td::VecMat4 Matching::GetPoses(){
    return poses_;
}
/**
 * get the camera position in the tessellated sphere
 * @return camera position in the tessellated sphere
 */
td::VecMat3 Matching::GetCamPos(){
    return cam_pos_;
}
/**
 * get vector for storing the model with normal
 * @return vector for storing the model with normal
 */
std::vector<td::pclib::PointNCloudPtr> Matching::GetModelWithNormals(){
    return model_->cloud_N_original_pose_vec;
}
/**
 * get the best match snapshot id
 * @return the best match snapshot id
 */
int Matching::GetBestMatchSnapshotId(){
    return best_match_model_snapshots_;
}
/**
 * show th point cloud with normal
 * @param pointNormal point cloud with normal
 */
void Matching::PointNormalVisual(td::pclib::PointNCloudPtr pointNormal)
{
    // Visualize them.
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
//    if(COLOR_STYLE == "print")
        viewer->setBackgroundColor(VIEWSTYLE.background_color[0], VIEWSTYLE.background_color[1], VIEWSTYLE.background_color[2]);
    if(!(WINDOW_SIZE[0] == 0||WINDOW_SIZE[1] == 0))
        viewer->setSize(WINDOW_SIZE[0],WINDOW_SIZE[1]);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> single_color(pointNormal, 0, 255, 0);
    viewer->addPointCloud<pcl::PointNormal>(pointNormal, single_color, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.005, "cloud");
    // Display one normal
    viewer->addPointCloudNormals<pcl::PointNormal>(pointNormal, 2, 0.01, "normals");
    viewer->addCoordinateSystem(0.05);
//        Eigen::Matrix4f pose;
//        pose = poses[l].inverse();
//        viewer->setCameraPosition(0,0,0,0,0,0,pose(0,0),pose(1,0),pose(2,0));
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }
}

/**
	Estimates the pose of an object in a scene
	@param transformation[out] The transformation (pose) of the object
	@param all_transformations[out] Vector with all transformations with inliers above inlier_fraction
	@return Vector of all inliers
 */
//std::vector<int> Matching::estimate_pose(Eigen::Matrix4d* transformation,
//                                         td::VecMat4* all_transformations, std::vector<double>* inlier_ratio) {
//
//
//    for (int k = 0; k < model_->correspondences.size(); ++k) {
//        int view_candidate_indice = model_->correspondences[k].index_query;
//        int print_mode = 1;
//        if(print_mode == 1)
//            pcl::console::print_highlight("Estimating pose...\n");
//        td::pclib::PointNCloudPtr object_aligned (new td::pclib::PointNCloud);
//
//        if(print_mode == 1)
//            pcl::console::print_info("\t Starting alignment...\n");
//
//        td::pclib::PointNCloudPtr view_candidate = model_->cloud_N_original_pose_vec[view_candidate_indice];
//        pcl::SampleConsensusPrerejective<td::pclib::PointN, td::pclib::PointN, td::pclib::FpfhDescriptor, double> aligner;
//        aligner.setInputSource(view_candidate);
//        aligner.setSourceFeatures(model_->fpfh_descriptors_vector[view_candidate_indice]);
//        aligner.setInputTarget(scene_->cloud_with_normal);
//        aligner.setTargetFeatures(scene_->fpfh_descriptors_ptr);
//        aligner.setMaximumIterations(cr_->pose_max_iterations);
//        aligner.setNumberOfSamples(3);
//        aligner.setCorrespondenceRandomness(cr_->pose_correspondence_randomness);
//        aligner.setSimilarityThreshold(cr_->pose_similarity_threshold);
//        aligner.setMaxCorrespondenceDistance(cr_->pose_max_correspondence_distance);
//        aligner.setInlierFraction(cr_->pose_inlier_fraction);
//        aligner.setNumberOfTransformations(10);
//        if(print_mode == 1) {
//            pcl::ScopeTime t("\t Alignment");
//            aligner.align (*object_aligned);
//        } else {
//            aligner.align (*object_aligned);
//        }
//
//        aligner.getTransformations(all_transformations,inlier_ratio);
//        std::vector<std::pair<std::vector<std::vector<int>>,float>> correspondence;
//        aligner.getCorrespondences(&correspondence);
//
//
//        for (int i = 0; i < all_transformations->size(); ++i) {
//            std::cout<<(*all_transformations)[i]<<std::endl;
//            std::cout<<(*inlier_ratio)[i]<<std::endl;
//        }
//
//        *transformation = aligner.getFinalTransformation();
//        std::cout<<"final transformation\n"<<*transformation<<std::endl;
//        if(aligner.hasConverged()) {
//            if(print_mode == 1)
//                pcl::console::print_info("\t Alignment successful!\n");
//            td::pclib::Viewer viewer("aligner pose", WINDOW_SIZE);
//            pcl::console::print_info ("Inliers: %i/%i\n", aligner.getInliers ().size (), model_->cloud_N_original_pose_vec[view_candidate_indice]->size ());
//            for (int i = 0; i < correspondence.size(); ++i) {
//                auto transformation_temp = (*all_transformations)[i];
////                if(COLOR_STYLE == "print")
//                    viewer.ShowCorrespondence(&correspondence[i],view_candidate, scene_->cloud_with_normal,transformation_temp, VIEWSTYLE.scene_color,VIEWSTYLE.model_color,VIEWSTYLE.line_rgb,VIEWSTYLE.point_size,VIEWSTYLE.background_color);
//                    viewer.ClearView();
////                else
////                    viewer.ShowCorrespondence(&correspondence[i],view_candidate, scene_->cloud_with_normal,transformation_temp,WINDOW_SIZE,POINT_SIZE);
//
//            }
//
//            //trimmed ICP
//            td::pclib::PointCloudPtr view_candidate_temp(new td::pclib::PointCloud);
//            pcl::copyPointCloud(*view_candidate,*view_candidate_temp);
//            TrimmedIcp(all_transformations, 0.7f, view_candidate_temp, scene_->cloud);
////                RunIcp(candidate_tg_T_sr_, view_candidate_temp, scene_->cloud);
//            for (int j = 0; j < correspondence.size(); ++j) {
//                auto transformation_temp = (*all_transformations)[j];
//                viewer.ShowCorrespondence(&correspondence[j],view_candidate, scene_->cloud_with_normal, transformation_temp,VIEWSTYLE.scene_color,VIEWSTYLE.model_color,VIEWSTYLE.line_rgb,VIEWSTYLE.point_size,VIEWSTYLE.background_color);//show the align result after the ICP
//                viewer.ClearView();
//
//            }
//            return aligner.getInliers();
//        } else {
//            if(print_mode == 1)
//                pcl::console::print_info("\t Alignment failed!\n");
//            return aligner.getInliers();
//        }
//    }
//
//
//}
/**
 * process the  ICP
 * @param guess_and_result object for storing the initial guess transformations and the result
 * @param source is the point cloud to be registered to the target.
 * @param target is target point cloud
 */
void Matching::RunIcp(td::VecMat4* guess_and_result, td::pclib::PointCloudPtr source, td::pclib::PointCloudPtr target)
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, double> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.setMaxCorrespondenceDistance(0.012);
    icp.setMaximumIterations(1000);
    icp.setRANSACIterations(1000);
    icp.setRANSACOutlierRejectionThreshold(0.0006);
    std::cout<<"-------- ICP ---------"<<std::endl;
    for (int i = 0; i < guess_and_result->size(); ++i) {
        pcl::PointCloud<pcl::PointXYZ> Final;
        icp.align(Final, (*guess_and_result)[i]);
        std::cout << "has converged:" << icp.hasConverged() << " score: " <<
                  icp.getFitnessScore() << std::endl;
        (*guess_and_result)[i] = icp.getFinalTransformation();
        std::cout << (*guess_and_result)[i] << std::endl;

    }
}
/**
 * process the trimmed ICP(handle the partially overlapping point sets registration problem)
 * @param guess_and_result object for storing the initial guess transformations and the result
 * @param init_ratio the initial ratio of the overlap
 * @param source is the point cloud to be registered to the target.
 * @param target is target point cloud
 */
void Matching::TrimmedIcp(td::VecMat4* guess_and_result, float init_ratio, td::pclib::PointCloudPtr source, td::pclib::PointCloudPtr target){
    pcl::recognition::TrimmedICP<td::pclib::Point,double> trimmed_icp;
    trimmed_icp.init(target);
    int num_source_points_to_use = int(init_ratio*source->size());
    std::cout<<"-------- Trimmed ICP ---------"<<std::endl;
    for (int i = 0; i < guess_and_result->size(); ++i) {
        std::cout<<"in candidate pose "<<i<<std::endl<<(*guess_and_result)[i]<<std::endl;
        trimmed_icp.align(*source, num_source_points_to_use, (*guess_and_result)[i]);
        std::cout<<"the pose after trimmed icp is\n"<<(*guess_and_result)[i]<<std::endl;
    }

}

Matching::Matching() {
    scene_ = SceneData::Ptr(new SceneData);
    model_ = ModelData::Ptr(new ModelData);
    ac_ = Access_Model_Data::GetInstance();
    cr_ = Config_Reader::GetInstance();
    cr_->add_model_load_config ("../config.ini");
    cr_->add_descriptors_config("../config.ini");
    cr_->system_load_config("../config.ini");
    cr_->add_GA_parameters("../config.ini");
    cr_->add_path_config("../config.ini");
    base_dir_ = cr_->base_dir;
    scene_->scene_name = cr_->scene_name;
    local_descriptors_type_ = cr_->local_descriptors_type_;
    //pointer initialization
    td::pclib::PointNCloudPtr cloud_with_normal(new td::pclib::PointNCloud);
    scene_->cloud_with_normal = cloud_with_normal;
    td::pclib::FpfhDescriptorCloudPtr fpfh_depscriptors_ptr(new td::pclib::FpfhDescriptorCloud);
    scene_->fpfh_descriptors_ptr = fpfh_depscriptors_ptr;
    fusion_level_ = cr_->fusion_level_ga;
}

void Matching::PointPick() {
//    td::pclib::PassFilter<td::pclib::PointN>(scene_->cloud_with_normal,"z",0,0.5);
    pcl::visualization::PCLVisualizer viewer("cloud viewer");
    viewer.addPointCloud<td::pclib::PointN>(scene_->cloud_with_normal);
    scene_->TrainKdtree();
    //view boundary
    scene_->GetBoundary(cr_->pose_feature_radius_search-0.0052);
    viewer.addPointCloud<pcl::PointNormal>(scene_->boundaries_points,td::pclib::ColorHandlerPointN(scene_->boundaries_points, 0,0,255),"boundary");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2*VIEWSTYLE.point_size,"boundary");

    viewer.registerPointPickingCallback (&Matching::pointPickingEventOccurred, *this, (void*)&viewer);
    viewer.spin();
}
void Matching::pointPickingEventOccurred(const pcl::visualization::PointPickingEvent& event,void* viewer_void)
{

    std::cout << "[INFO] Point picking event occurred." << std::endl;
    td::pclib::PointN choosed_point;

    if (event.getPointIndex () == -1)
    {
        return;
    }
    event.getPoint(choosed_point.x, choosed_point.y, choosed_point.z);
    int choosed_index = event.getPointIndex();
    std::vector<int> pointIndices;
    std::vector<float> squaredDistances;
    scene_->kdtree->radiusSearch(choosed_point,cr_->pose_feature_radius_search,pointIndices,squaredDistances);
    bool isboundary = std::binary_search(scene_->boundaries_indices->indices.begin(), scene_->boundaries_indices->indices.end(), choosed_index);

    std::cout << "[INFO] Point coordinate ( " << choosed_point.x << ", " << choosed_point.y << ", " << choosed_point.z << ")" << "  has "<<pointIndices.size()<<" neighbor"
    <<"  boundary = "<<isboundary<<std::endl;
}
/**
	Estimates the pose of an object in a scene
	@param transformation[out] The transformation (pose) of the object
	@param all_transformations[out] Vector with all transformations with inliers above inlier_fraction
	@return Vector of all inliers
 */
//std::vector<int> Matching::estimate_pose_ga(Eigen::Matrix4d* transformation,
//                                            td::VecMat4* all_transformations, std::vector<double>* inlier_ratio) {
//
//
//    for (int k = 0; k < model_->correspondences.size(); ++k) {
//        int view_candidate_indice = model_->correspondences[k].index_query;
//        int print_mode = 1;
//        if(print_mode == 1)
//            pcl::console::print_highlight("Estimating pose...\n");
//        td::pclib::PointNCloudPtr object_aligned (new td::pclib::PointNCloud);
//
//        if(print_mode == 1)
//            pcl::console::print_info("\t Starting alignment...\n");
//
//        td::pclib::PointNCloudPtr view_candidate = model_->cloud_N_original_pose_vec[view_candidate_indice];
//        pcl::SampleConsensusPrerejective<td::pclib::PointN, td::pclib::PointN, td::pclib::FpfhDescriptor, double> aligner;
//        aligner.setInputSource(view_candidate);
//        aligner.setSourceFeatures(model_->fpfh_descriptors_vector[view_candidate_indice]);
//        aligner.setInputTarget(scene_->cloud_with_normal);
//        aligner.setTargetFeatures(scene_->fpfh_descriptors_ptr);
//        aligner.setMaximumIterations(cr_->pose_max_iterations);
//        aligner.setNumberOfSamples(3);
//        aligner.setCorrespondenceRandomness(cr_->pose_correspondence_randomness);
//        aligner.setSimilarityThreshold(cr_->pose_similarity_threshold);
//        aligner.setMaxCorrespondenceDistance(cr_->pose_max_correspondence_distance);
//        aligner.setInlierFraction(cr_->pose_inlier_fraction);
//        aligner.setNumberOfTransformations(10);
//        if(print_mode == 1) {
//            pcl::ScopeTime t("\t Alignment");
//            aligner.align (*object_aligned);
//        } else {
//            aligner.align (*object_aligned);
//        }
//        // GA algorithm
//        // compute the ranges of the parameter
//        aligner.get_first_two_transformation_ga(&first_two_transformation_);
//        ShowRangeOfGaParameter(&first_two_transformation_);
//        Eigen::Matrix4d o1_T_o2 = first_two_transformation_[0].first.inverse() * first_two_transformation_[1].first;
//        Eigen::Matrix3d o1_R_o2 = o1_T_o2.block(0,0,3,3);
//        Eigen::Vector3d o1_euler_angle_o2 = o1_R_o2.eulerAngles(2,1,0);
//        double t0_min,t0_max,t1_min,t1_max,t2_min,t2_max,af_min,af_max,bt_min,bt_max,gm_min,gm_max;
//        if(o1_T_o2(0,3) > 0)
//        {
//            t0_min = -3 * o1_T_o2(0,3);
//            t0_max = 3 * o1_T_o2(0,3);
//        }
//        else{
//            t0_min = 3 * o1_T_o2(0,3);
//            t0_max = -3 * o1_T_o2(0,3);
//        }
//        if(o1_T_o2(1,3) > 0)
//        {
//            t1_min = -3 * o1_T_o2(1,3);
//            t1_max = 3 * o1_T_o2(1,3);
//        }
//        else{
//            t1_min = 3 * o1_T_o2(1,3);
//            t1_max = -3 * o1_T_o2(1,3);
//        }
//        if(o1_T_o2(2,3) > 0)
//        {
//            t2_min = -3 * o1_T_o2(2,3);
//            t2_max = 3 * o1_T_o2(2,3);
//        }
//        else{
//            t2_min = 3 * o1_T_o2(2,3);
//            t2_max = -3 * o1_T_o2(2,3);
//        }
//        if(o1_euler_angle_o2(0) > 0)
//        {
//            af_min = -3 * o1_euler_angle_o2(0);
//            af_max = 3 * o1_euler_angle_o2(0);
//        }
//        else{
//            af_min = 3 * o1_euler_angle_o2(0);
//            af_max = -3 * o1_euler_angle_o2(0);
//        }
//        if(o1_euler_angle_o2(1) > 0)
//        {
//            bt_min = -3 * o1_euler_angle_o2(1);
//            bt_max = 3 * o1_euler_angle_o2(1);
//        }
//        else{
//            bt_min = 3 * o1_euler_angle_o2(1);
//            bt_max = -3 * o1_euler_angle_o2(1);
//        }
//        if(o1_euler_angle_o2(2) > 0)
//        {
//            gm_min = -3 * o1_euler_angle_o2(2);
//            gm_max = 3 * o1_euler_angle_o2(2);
//        }
//        else{
//            gm_min = 3 * o1_euler_angle_o2(2);
//            gm_max = -3 * o1_euler_angle_o2(2);
//        }
//        int N_t1 = std::ceil(std::log((t1_max - t1_min)/0.0001 + 1) / std::log(2));
//        int N_t2 = std::ceil(std::log((t2_max - t2_min)/0.0001 + 1) / std::log(2));
//        int N_t0 = std::ceil(std::log((t0_max - t0_min)/0.0001 + 1) / std::log(2));
//        int N_af = std::ceil(std::log((af_max - af_min)/0.0001 + 1) / std::log(2));
//        int N_bt = std::ceil(std::log((bt_max - bt_min)/0.0001 + 1) / std::log(2));
//        int N_gm = std::ceil(std::log((gm_max - gm_min)/0.0001 + 1) / std::log(2));
//        galgo::Parameter<double,64> t1({t1_min,t1_max});
//        galgo::Parameter<double,64> t2({t2_min,t2_max});
//        galgo::Parameter<double,64> t0({t0_min,t0_max});
//        galgo::Parameter<double,64> af({af_min,af_max});
//        galgo::Parameter<double,64> bt({bt_min,bt_max});
//        galgo::Parameter<double,64> gm({gm_min,gm_max});
//        ga_objective<double> object;
//        object.SetStaticData(aligner,first_two_transformation_[0].first,first_two_transformation_[0].second);
//        std::cout<<"first_transformation final_fitness = "<<1000 * first_two_transformation_[0].second<<std::endl;
//        galgo::GeneticAlgorithm<double> ga(ga_objective<double>::Objective,100,50,true,t0,t1,t2,af,bt,gm);
//        ga.run();
//        galgo::CHR<double> best_CHR = ga.result();
//        std::vector<double> best_o1_T_on_vector = best_CHR->getParam();
//        Eigen::Vector3d best_o1_t_on(best_o1_T_on_vector[0],best_o1_T_on_vector[1],best_o1_T_on_vector[2]);
//        Eigen::Matrix3d best_o1_R_on = (Eigen::AngleAxisd(best_o1_T_on_vector[3],Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(best_o1_T_on_vector[4],Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(best_o1_T_on_vector[5],Eigen::Vector3d::UnitX())).matrix();
//        Eigen::Isometry3d best_o1_T_on = Eigen::Isometry3d::Identity();
//        best_o1_T_on.rotate(best_o1_R_on);
//        best_o1_T_on.pretranslate(best_o1_t_on);
//
//        Eigen::Matrix4d best_s_T_on = first_two_transformation_[0].first * best_o1_T_on.matrix();
////        if(COLOR_STYLE == "print")
//            td::pclib::ShowAlignResult(view_candidate, scene_->cloud_with_normal, best_s_T_on,"align",VIEWSTYLE.scene_color,VIEWSTYLE.model_color,WINDOW_SIZE,VIEWSTYLE.point_size,VIEWSTYLE.background_color);
////        else
////            td::pclib::ShowAlignResult(view_candidate, scene_->cloud_with_normal, best_s_T_on,"align",WINDOW_SIZE,POINT_SIZE);
//
//
//        aligner.getTransformations(all_transformations, inlier_ratio);
//        std::vector<std::pair<std::vector<std::vector<int>>,float>> correspondence;
//        aligner.getCorrespondences(&correspondence);
//
//
//        for (int i = 0; i < all_transformations->size(); ++i) {
//            std::cout<<(*all_transformations)[i]<<std::endl;
//            std::cout<<(*inlier_ratio)[i]<<std::endl;
//        }
//        *transformation = aligner.getFinalTransformation();
//        std::cout<<"final transformation\n"<<*transformation<<std::endl;
//        if(aligner.hasConverged()) {
//            if(print_mode == 1)
//                pcl::console::print_info("\t Alignment successful!\n");
//            pcl::console::print_info ("Inliers: %i/%i\n", aligner.getInliers ().size (), model_->cloud_N_original_pose_vec[view_candidate_indice]->size ());
//            td::pclib::Viewer viewer;
//            for (int i = 0; i < correspondence.size(); ++i) {
//                auto transformation_temp = (*all_transformations)[i];
////                if(COLOR_STYLE == "print")
//                    viewer.ShowCorrespondence(&correspondence[i],view_candidate, scene_->cloud_with_normal, transformation_temp, VIEWSTYLE.scene_color,VIEWSTYLE.model_color, WINDOW_SIZE,VIEWSTYLE.point_size, VIEWSTYLE.background_color);
////                else
////                    viewer.ShowCorrespondence(&correspondence[i],view_candidate, scene_->cloud_with_normal, transformation_temp,WINDOW_SIZE,POINT_SIZE);
//            }
//
//            return aligner.getInliers();
//        } else {
//            if(print_mode == 1)
//                pcl::console::print_info("\t Alignment failed!\n");
//            return aligner.getInliers();
//        }
//    }
//
//}
/**
	Estimates the pose of an object in a scene
	@param transformation[out] The transformation (pose) of the object
	@param all_transformations[out] Vector with all transformations with inliers above inlier_fraction
	@return Vector of all inliers
 */
//std::vector<int> Matching::estimate_pose_openGA(Eigen::Matrix4d* transformation,
//                                                td::VecMat4* all_transformations, std::vector<double>* inlier_ratio) {
//
//    for (int k = 0; k < model_->correspondences.size(); ++k) {
//        int view_candidate_indice = model_->correspondences[k].index_query;
//        int print_mode = 1;
//        if(print_mode == 1)
//            pcl::console::print_highlight("Estimating pose...\n");
//        td::pclib::PointNCloudPtr object_aligned (new td::pclib::PointNCloud);
//
//        if(print_mode == 1)
//            pcl::console::print_info("\t Starting alignment...\n");
//
//        td::pclib::PointNCloudPtr view_candidate = model_->cloud_N_original_pose_vec[view_candidate_indice];
//        pcl::SampleConsensusPrerejective<td::pclib::PointN, td::pclib::PointN, td::pclib::FpfhDescriptor, double> aligner;
//        aligner.setInputSource(view_candidate);
//        aligner.setSourceFeatures(model_->fpfh_descriptors_vector[view_candidate_indice]);
//        aligner.setInputTarget(scene_->cloud_with_normal);
//        aligner.setTargetFeatures(scene_->fpfh_descriptors_ptr);
//        aligner.setMaximumIterations(cr_->pose_max_iterations);
//        aligner.setNumberOfSamples(3);
//        aligner.setCorrespondenceRandomness(cr_->pose_correspondence_randomness);
//        aligner.setSimilarityThreshold(cr_->pose_similarity_threshold);
//        aligner.setMaxCorrespondenceDistance(cr_->pose_max_correspondence_distance);
//        aligner.setInlierFraction(cr_->pose_inlier_fraction);
//        aligner.setNumberOfTransformations(10);
//        if(print_mode == 1) {
//            pcl::ScopeTime t("\t Alignment");
//            aligner.align (*object_aligned);
//        } else {
//            aligner.align (*object_aligned);
//        }
//        // GA algorithm
//        // compute the ranges of the parameter
//        aligner.get_first_two_transformation_ga(&first_two_transformation_);
//        ShowRangeOfGaParameter(&first_two_transformation_);
//        Eigen::Matrix4d o1_T_o2 = first_two_transformation_[0].first.inverse() * first_two_transformation_[1].first;
//        Eigen::Matrix3d o1_R_o2 = o1_T_o2.block(0,0,3,3);
//        Eigen::Vector3d o1_euler_angle_o2 = o1_R_o2.eulerAngles(2,1,0);
//        double t0_min,t0_max,t1_min,t1_max,t2_min,t2_max,af_min,af_max,bt_min,bt_max,gm_min,gm_max;
//        double range_scale = 3;
//        std::vector<std::vector<double>> genes_range_;
//        genes_range_.resize(6);
//        if(o1_T_o2(0,3) > 0)
//        {
//            t0_min = -range_scale * o1_T_o2(0,3);
//            t0_max = range_scale * o1_T_o2(0,3);
//
//
//        }
//        else{
//            t0_min = range_scale * o1_T_o2(0,3);
//            t0_max = -range_scale * o1_T_o2(0,3);
//
//        }
//        genes_range_[0].push_back(t0_min);
//        genes_range_[0].push_back(t0_max);
//        if(o1_T_o2(1,3) > 0)
//        {
//            t1_min = -range_scale * o1_T_o2(1,3);
//            t1_max = range_scale * o1_T_o2(1,3);
//        }
//        else{
//            t1_min = range_scale * o1_T_o2(1,3);
//            t1_max = -range_scale * o1_T_o2(1,3);
//        }
//        genes_range_[1].push_back(t1_min);
//        genes_range_[1].push_back(t1_max);
//        if(o1_T_o2(2,3) > 0)
//        {
//            t2_min = -range_scale * o1_T_o2(2,3);
//            t2_max = range_scale * o1_T_o2(2,3);
//        }
//        else{
//            t2_min = range_scale * o1_T_o2(2,3);
//            t2_max = -range_scale * o1_T_o2(2,3);
//        }
//        genes_range_[2].push_back(t2_min);
//        genes_range_[2].push_back(t2_max);
//        if(o1_euler_angle_o2(0) > 0)
//        {
//            af_min = -range_scale * o1_euler_angle_o2(0);
//            af_max = range_scale * o1_euler_angle_o2(0);
//        }
//        else{
//            af_min = range_scale * o1_euler_angle_o2(0);
//            af_max = -range_scale * o1_euler_angle_o2(0);
//        }
//        genes_range_[3].push_back(af_min);
//        genes_range_[3].push_back(af_max);
//        if(o1_euler_angle_o2(1) > 0)
//        {
//            bt_min = -range_scale * o1_euler_angle_o2(1);
//            bt_max = range_scale * o1_euler_angle_o2(1);
//        }
//        else{
//            bt_min = range_scale * o1_euler_angle_o2(1);
//            bt_max = -range_scale * o1_euler_angle_o2(1);
//        }
//        genes_range_[4].push_back(bt_min);
//        genes_range_[4].push_back(bt_max);
//        if(o1_euler_angle_o2(2) > 0)
//        {
//            gm_min = -range_scale * o1_euler_angle_o2(2);
//            gm_max = range_scale * o1_euler_angle_o2(2);
//        }
//        else{
//            gm_min = range_scale * o1_euler_angle_o2(2);
//            gm_max = -range_scale * o1_euler_angle_o2(2);
//        }
//        genes_range_[5].push_back(gm_min);
//        genes_range_[5].push_back(gm_max);
//        std::vector<std::string> genes_name;
//        genes_name.push_back("t0");
//        genes_name.push_back("t1");
//        genes_name.push_back("t2");
//        genes_name.push_back("af");
//        genes_name.push_back("bt");
//        genes_name.push_back("gm");
//
//        for (int l = 0; l < genes_range_.size(); ++l) {
//            std::cout<<genes_name[l]<<" : min="<<genes_range_[l][0]<<" max="<<genes_range_[l][1]<<std::endl;
//        }
//        openga_class openga;
//        openga.SetStaticData(genes_range_, aligner,first_two_transformation_, o1_T_o2, o1_euler_angle_o2);
//        std::cout<<"first_transformation final_fitness = "<<1000 * first_two_transformation_[0].second<<std::endl;
//
//        std::vector<double> best_o1_T_on_vector = openga.solve();;
//        Eigen::Vector3d best_o1_t_on(best_o1_T_on_vector[0],best_o1_T_on_vector[1],best_o1_T_on_vector[2]);
//        Eigen::Matrix3d best_o1_R_on = (Eigen::AngleAxisd(best_o1_T_on_vector[3],Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(best_o1_T_on_vector[4],Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(best_o1_T_on_vector[5],Eigen::Vector3d::UnitX())).matrix();
//        Eigen::Isometry3d best_o1_T_on = Eigen::Isometry3d::Identity();
//        best_o1_T_on.rotate(best_o1_R_on);
//        best_o1_T_on.pretranslate(best_o1_t_on);
//
//        Eigen::Matrix4d best_s_T_on = first_two_transformation_[0].first * best_o1_T_on.matrix();
////        if(COLOR_STYLE == "print")
//            td::pclib::ShowAlignResult(view_candidate, scene_->cloud_with_normal, best_s_T_on,"align", VIEWSTYLE.scene_color, VIEWSTYLE.model_color,WINDOW_SIZE,VIEWSTYLE.point_size,VIEWSTYLE.background_color);
////        else
////            td::pclib::ShowAlignResult(view_candidate, scene_->cloud_with_normal, best_s_T_on,"align",WINDOW_SIZE,POINT_SIZE);
//
//
//        aligner.getTransformations(all_transformations, inlier_ratio);
//        std::vector<std::pair<std::vector<std::vector<int>>,float>> correspondence;
//        aligner.getCorrespondences(&correspondence);
//
//
//        for (int i = 0; i < all_transformations->size(); ++i) {
//            std::cout<<(*all_transformations)[i]<<std::endl;
//            std::cout<<(*inlier_ratio)[i]<<std::endl;
//        }
//        *transformation = aligner.getFinalTransformation();
//        std::cout<<"final transformation\n"<<*transformation<<std::endl;
//        if(aligner.hasConverged()) {
//            if(print_mode == 1)
//                pcl::console::print_info("\t Alignment successful!\n");
//            pcl::console::print_info ("Inliers: %i/%i\n", aligner.getInliers ().size (), model_->cloud_N_original_pose_vec[view_candidate_indice]->size ());
//            td::pclib::Viewer viewer;
//            for (int i = 0; i < correspondence.size(); ++i) {
//                auto transformation_temp = (*all_transformations)[i];
////                if(COLOR_STYLE == "print")
//                    viewer.ShowCorrespondence(&correspondence[i],view_candidate, scene_->cloud_with_normal, transformation_temp, VIEWSTYLE.scene_color, VIEWSTYLE.model_color,WINDOW_SIZE, VIEWSTYLE.point_size, VIEWSTYLE.background_color);
////                else
////                    viewer.ShowCorrespondence(&correspondence[i],view_candidate, scene_->cloud_with_normal, transformation_temp,WINDOW_SIZE,POINT_SIZE);
//
//            }
//            return aligner.getInliers();
//        } else {
//            if(print_mode == 1)
//                pcl::console::print_info("\t Alignment failed!\n");
//            return aligner.getInliers();
//        }
//    }
//
//}

//void Matching::ShowRangeOfGaParameter(const td::VectorPairMatrix4dDouble* first_two_tansformation){
//
//    Eigen::Matrix4d o1_T_o2 = (*first_two_tansformation)[0].first.inverse() * (*first_two_tansformation)[1].first;
//    Eigen::Matrix3d o1_R_o2 = o1_T_o2.block(0,0,3,3);
//    Eigen::Vector3d o1_euler_angle_o2 = o1_R_o2.eulerAngles(2,1,0);
//    double t0_min,t0_max,t1_min,t1_max,t2_min,t2_max,af_min,af_max,bt_min,bt_max,gm_min,gm_max;
//    if(o1_T_o2(0,3) > 0)
//    {
//        t0_min = -3 * o1_T_o2(0,3);
//        t0_max = 3 * o1_T_o2(0,3);
//    }
//    else{
//        t0_min = 3 * o1_T_o2(0,3);
//        t0_max = -3 * o1_T_o2(0,3);
//    }
//    if(o1_T_o2(1,3) > 0)
//    {
//        t1_min = -3 * o1_T_o2(1,3);
//        t1_max = 3 * o1_T_o2(1,3);
//    }
//    else{
//        t1_min = 3 * o1_T_o2(1,3);
//        t1_max = -3 * o1_T_o2(1,3);
//    }
//    if(o1_T_o2(2,3) > 0)
//    {
//        t2_min = -3 * o1_T_o2(2,3);
//        t2_max = 3 * o1_T_o2(2,3);
//    }
//    else{
//        t2_min = 3 * o1_T_o2(2,3);
//        t2_max = -3 * o1_T_o2(2,3);
//    }
//    if(o1_euler_angle_o2(0) > 0)
//    {
//        af_min = -3 * o1_euler_angle_o2(0);
//        af_max = 3 * o1_euler_angle_o2(0);
//    }
//    else{
//        af_min = 3 * o1_euler_angle_o2(0);
//        af_max = -3 * o1_euler_angle_o2(0);
//    }
//    if(o1_euler_angle_o2(1) > 0)
//    {
//        bt_min = -3 * o1_euler_angle_o2(1);
//        bt_max = 3 * o1_euler_angle_o2(1);
//    }
//    else{
//        bt_min = 3 * o1_euler_angle_o2(1);
//        bt_max = -3 * o1_euler_angle_o2(1);
//    }
//    if(o1_euler_angle_o2(2) > 0)
//    {
//        gm_min = -3 * o1_euler_angle_o2(2);
//        gm_max = 3 * o1_euler_angle_o2(2);
//    }
//    else{
//        gm_min = 3 * o1_euler_angle_o2(2);
//        gm_max = -3 * o1_euler_angle_o2(2);
//    }
//    pcl::visualization::PCLVisualizer viewer("sampling pose");
////    if(COLOR_STYLE == "print")
//        viewer.setBackgroundColor(VIEWSTYLE.background_color[0],VIEWSTYLE.background_color[1],VIEWSTYLE.background_color[2]);
//    viewer.addCoordinateSystem(0.003);
//    for (int i = 0; i < 50; ++i) {
//        auto t0_temp = td::UniformSampling<double>(t0_min,t0_max);
//        auto t1_temp = td::UniformSampling<double>(t1_min,t1_max);
//        auto t2_temp = td::UniformSampling<double>(t2_min,t2_max);
//        auto af_temp = td::UniformSampling<double>(af_min,af_max);
//        auto bt_temp = td::UniformSampling<double>(bt_min,bt_max);
//        auto gm_temp = td::UniformSampling<double>(gm_min,gm_max);
//        Eigen::Vector3d o1_euler_angle_on(af_temp,bt_temp,gm_temp);
//        Eigen::Matrix3d o1_R_on = (Eigen::AngleAxisd(o1_euler_angle_on[0],Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(o1_euler_angle_on[1],Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(o1_euler_angle_on[2],Eigen::Vector3d::UnitX())).matrix();
//        Eigen::Isometry3d o1_T_on = Eigen::Isometry3d::Identity();
//        o1_T_on.rotate(o1_R_on);
//        o1_T_on.pretranslate(Eigen::Vector3d(t0_temp,t1_temp,t2_temp));
//
//        Eigen::Vector3d x_vertex_transformed, y_vertex_transformed, z_vertex_transformed,o_vertex_transformed;
//        x_vertex_transformed = o1_T_on * Eigen::Vector3d(0.001,0,0);
//        y_vertex_transformed = o1_T_on * Eigen::Vector3d(0,0.001,0);
//        z_vertex_transformed = o1_T_on * Eigen::Vector3d(0,0,0.001);
//        o_vertex_transformed = o1_T_on * Eigen::Vector3d(0,0,0);
//        td::pclib::Point X_vertex(x_vertex_transformed(0),x_vertex_transformed(1),x_vertex_transformed(2));
//        td::pclib::Point Y_vertex(y_vertex_transformed(0),y_vertex_transformed(1),y_vertex_transformed(2));
//        td::pclib::Point Z_vertex(z_vertex_transformed(0),z_vertex_transformed(1),z_vertex_transformed(2));
//        td::pclib::Point O_vertex(o_vertex_transformed(0),o_vertex_transformed(1),o_vertex_transformed(2));
//
//        std::stringstream X_axis;
//        X_axis << "X"<<i;
//        viewer.addLine<pcl::PointXYZ, pcl::PointXYZ> (X_vertex, O_vertex, 255, 0, 0, X_axis.str ());
//        std::stringstream Y_axis;
//        Y_axis << "Y"<<i;
//        viewer.addLine<pcl::PointXYZ, pcl::PointXYZ> (Y_vertex, O_vertex, 0, 255, 0, Y_axis.str ());
//        std::stringstream Z_axis;
//        Z_axis << "Z"<<i;
//        viewer.addLine<pcl::PointXYZ, pcl::PointXYZ> (Z_vertex, O_vertex, 0, 0, 255, Z_axis.str ());
//    }
//    while (!viewer.wasStopped())
//        viewer.spin();
//
//}




