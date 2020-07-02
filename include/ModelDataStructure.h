//
// Created by wang on 19-10-25.
//

#ifndef OBJECT_POSE_ESTIMATION_MODELDATASTRUCTURE_H
#define OBJECT_POSE_ESTIMATION_MODELDATASTRUCTURE_H

#include <TdLibrary/td_eigen/eigen_common_typedef.h>
#include "view_graph/view_graph.h"
#include "GlobalCorrespondence.h"
/*
A structure for storing the data associated to each model
*/
struct ModelData
{
    ModelData(){
        complete_model = td::pclib::PointCloudPtr(new td::pclib::PointCloud);
        view_graph = View_Graph::Ptr(new View_Graph);
    }
    std::string name;
    td::pclib::PointCloudPtr complete_model;
//        View_Graph graph;
    GlobalCorrespondence correspondences;
//    std::vector<pcl::Correspondence> correspondences;
//    std::vector<int> correspondences_index_query;
    std::vector<td::pclib::PointNCloudPtr> cloud_N_original_pose_vec;
    std::vector<td::pclib::PointNCloudPtr> cloud_N_cam_vec;
    std::vector<td::pclib::PointNCloudPtr> fusion_cloud;
    std::vector<td::pclib::FpfhDescriptorCloudPtr>  fpfh_descriptors_vector;
    td::pclib::EsfDescriptorCloudPtr esf_descriptors;
    View_Graph::Ptr view_graph;
    Eigen::Vector3f bound_box;
//    Eigen::Vector3f bound_box_den;
//    void set_bound_boox_den(){
//        int max_index;
//        int min_index;
//        for (int i = 0; i < bound_box.size(); ++i) {
//            if()
//        }
//    }
    void ShowCorrespondenceFilter(){
        correspondences.ShowGlobalCorrespondenceInViewGraph();
        correspondences.NetCluster();
        if(VISUAL)
            correspondences.ShowNetClusterResult();
        correspondences.NetVertexFilter();
    }
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    typedef boost::shared_ptr< ::ModelData> Ptr;
    typedef boost::shared_ptr< ::ModelData const> ConstPtr;
};
#endif //OBJECT_POSE_ESTIMATION_MODELDATASTRUCTURE_H
