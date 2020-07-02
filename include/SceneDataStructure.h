//
// Created by wang on 19-11-18.
//

#ifndef OBJECT_POSE_ESTIMATION_SCENEDATASTRUCTURE_H
#define OBJECT_POSE_ESTIMATION_SCENEDATASTRUCTURE_H
#include <TdLibrary/td_eigen/eigen_common_typedef.h>
/*
A structure for storing the data associated to the cluster
*/
struct SceneData
{
    SceneData(){
        normal = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
        kdtree = pcl::search::KdTree<td::pclib::PointN>::Ptr(new pcl::search::KdTree<td::pclib::PointN>);
        boundaries_points = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
        non_boundaries_points = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
        boundaries_indices = pcl::PointIndicesPtr(new pcl::PointIndices);
        non_boundaries_indices = pcl::PointIndicesPtr(new pcl::PointIndices);
    }
    std::string scene_name;
    int index;
    td::pclib::PointCloudPtr cloud;
    td::pclib::EsfDescriptor global_feature;
    td::pclib::FpfhDescriptorCloudPtr fpfh_descriptors_ptr;
    td::pclib::PointNCloudPtr cloud_with_normal;
    pcl::PointCloud<pcl::Normal>::Ptr normal;
    std::vector<std::string> similar_models;
    pcl::search::KdTree<td::pclib::PointN>::Ptr kdtree;
    pcl::PointCloud<pcl::Boundary> boundaries;
    pcl::PointCloud<pcl::PointNormal>::Ptr boundaries_points;
    pcl::PointCloud<pcl::PointNormal>::Ptr non_boundaries_points;
    pcl::PointIndicesPtr boundaries_indices;
    pcl::PointIndicesPtr non_boundaries_indices;
    pcl::BoundaryEstimation<pcl::PointNormal,pcl::Normal, pcl::Boundary> boundary_est;
    int specified_view;//for local pipeline, we will give the corresponding view.
public:
    typedef boost::shared_ptr< ::SceneData> Ptr;
    typedef boost::shared_ptr< ::SceneData const> ConstPtr;
    void TrainKdtree(){
        kdtree->setInputCloud(cloud_with_normal);
    }
    void GetBoundary(float radius_search){
        boundary_est.setInputCloud(cloud_with_normal);
        boundary_est.setInputNormals(normal);
        boundary_est.setRadiusSearch(radius_search);
        boundary_est.setSearchMethod(kdtree);
        boundary_est.compute(boundaries);
        for (int i = 0; i < cloud_with_normal->size(); ++i) {
            uint8_t x = boundaries.points[i].boundary_point;
            int  a = static_cast<int>(x);
            if (a == 1)
            {
                boundaries_points->push_back(cloud_with_normal->points[i]);
                boundaries_indices->indices.push_back(i);
            } else
            {
                non_boundaries_points->push_back(cloud_with_normal->points[i]);
                non_boundaries_indices->indices.push_back(i);
            }

        }

        std::cout<<"boundary's size = "<<boundaries_points->size()<<std::endl;
    }

};
#endif //OBJECT_POSE_ESTIMATION_SCENEDATASTRUCTURE_H
