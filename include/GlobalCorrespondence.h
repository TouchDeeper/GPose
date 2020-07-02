//
// Created by wang on 19-11-19.
//

#ifndef OBJECT_POSE_ESTIMATION_GLOBALCORRESPONDENCE_H
#define OBJECT_POSE_ESTIMATION_GLOBALCORRESPONDENCE_H

#include <vector>
#include "view_graph/view_graph.h"
#include <TdLibrary/td_eigen/eigen_common_typedef.h>
#include <TdLibrary/tool/random_tool.hpp>
#include "parameters.h"
extern bool VISUAL;
class GlobalCorrespondence {
public:
    GlobalCorrespondence(){
        view_graph_ = View_Graph::Ptr(new View_Graph);


    }
    void TransferData(std::vector<pcl::Correspondence> &correspondences, View_Graph::Ptr view_graph, td::pclib::PointCloudPtr complete_model);
    int size(){
        return correspondences_.size();
    }
    std::vector<pcl::Correspondence,std::allocator<pcl::Correspondence>>::iterator begin(){
        return correspondences_.begin();
    }
    std::vector<pcl::Correspondence,std::allocator<pcl::Correspondence>>::iterator end(){
        return correspondences_.end();
    }
    pcl::Correspondence& operator[](int index){
        return correspondences_[index];
    }
    void clear(){
        correspondences_.clear();
        correspondences_index_query_.clear();
    }
    void push_back(pcl::Correspondence &cor){
        correspondences_.push_back(cor);
        correspondences_index_query_.push_back(cor.index_query);
    }
    void sort_index_query(){
        std::sort(correspondences_.begin(), correspondences_.end(), less_than_query());
        correspondences_index_query_.clear();
        for (int i = 0; i < correspondences_.size(); ++i) {
            correspondences_index_query_.push_back(correspondences_[i].index_query);
        }
    }
    void sort_by_distance(){
        std::sort(correspondences_.begin(), correspondences_.end(), less_than_distance_struct());
        correspondences_index_query_.clear();
        for (int i = 0; i < correspondences_.size(); ++i) {
            correspondences_index_query_.push_back(correspondences_[i].index_query);
        }
    }
    void NetCluster();
    void FarAwayVertex(std::vector<pcl::Correspondence> &most_neighbor_groups, std::vector<pcl::Correspondence> &far_away_neighbor_groups);
    std::vector<pcl::Correspondence> correspondences_;
    std::vector<int> correspondences_index_query_;

    void ShowNetClusterResult();
    void ShowCorrespondences();
    void ShowGlobalCorrespondenceInViewGraph();
    void AddCorrespondenceNeighborLine(pcl::visualization::PCLVisualizer &visu);
    void NetVertexFilter();

//    double RelativeDist(pcl::Correspondence &smaller_cor, pcl::Correspondence larger_cor);

private:
    struct NeighborGroup{
        NeighborGroup(){
            average_distance = 0;
        }
        std::vector<pcl::Correspondence> correspondences;
        double average_distance;
        void push_back(pcl::Correspondence &correspondence){
            correspondences.push_back(correspondence);
        }
        pcl::Correspondence get_correspondence(int index){
            return correspondences[index];
        }
        int size(){
            return correspondences.size();
        }
        pcl::Correspondence& operator[](int index){
            return correspondences[index];
        }
        void ComputeAverageDistance(){
            for (const auto &item : correspondences) {
                average_distance += item.distance;
            }
            average_distance /= correspondences.size();
        }
    };
    struct NetClustering{
        std::vector<NeighborGroup> neighbor_groups;
        std::vector<NeighborGroup> most_neighbor_groups;
        double most_neighbor_groups_average_distance;
        std::vector<pcl::Correspondence> most_neighbor_vertex; //the object to store the most_neighbor_groups[i][0]
        std::vector<pcl::Correspondence> far_away_vertexs;
        void push_back(NeighborGroup &neighbor_group){
            neighbor_groups.push_back(neighbor_group);
        }
        void ComputeAverageDistances(){
            for (auto &item : neighbor_groups) {
                item.ComputeAverageDistance();

            }
        }
        void FindMostNeighborGroups(){
            int most_neighbor_count = -1;
            for (auto &item : neighbor_groups) {
                if(item.size() > most_neighbor_count)
                {
                    most_neighbor_groups.clear();
                    most_neighbor_groups.push_back(item);
                    most_neighbor_count = item.size();
                    std::cout<<"most neighbor count = "<<most_neighbor_count<<"  index = "<<item[0].index_query<<std::endl;
                    std::cout<<"most neighbor group size = "<<most_neighbor_groups.size()<<std::endl;
                }
                if(item.size() == most_neighbor_count)
                    most_neighbor_groups.push_back(item);
            }
            SortMostNeighborGroups();
            if(most_neighbor_count < 3)
            {
                NeighborGroup choosed_group = most_neighbor_groups[0];
                most_neighbor_groups.clear();
                most_neighbor_groups.push_back(choosed_group);
            }

        }
        struct less_than_average_distance{
            inline bool operator() (const NeighborGroup g1, const NeighborGroup g2)
            {
                return (g1.average_distance < g2.average_distance);
            }
        };
        void SortMostNeighborGroups(){
            most_neighbor_vertex.clear();
            std::sort(most_neighbor_groups.begin(),most_neighbor_groups.end(),less_than_average_distance());
            for (const auto &item : most_neighbor_groups) {
                most_neighbor_vertex.push_back(item.correspondences[0]);
            }

        }
        void ComputeMostNeighborGroupsAverageDistance(){
            most_neighbor_groups_average_distance = 0;
            for (const auto &item : most_neighbor_groups) {
                most_neighbor_groups_average_distance += item.average_distance;
            }
            most_neighbor_groups_average_distance /= most_neighbor_groups.size();
        }

//        void ComputeViewAngle(pcl::Correspondence &c1, pcl::Correspondence &c2)
//        {
//            Eigen::Vector3d c1_point =
//        }
//        void ComputeCandidateCorrespondence(){
//            ComputeAverageDistances();
//            FindMostNeighborGroups();
//        }
    };
//    struct Node{
//        int row_;
//        int index_;
//        Node(int row, int index){
//            row_ = row;
//            index_ = index;
//        }
//    };
    struct Edge{
        Edge(int i1, int i2){
            index1 = i1;
            index2 = i2;
        }
        int index1;
        int index2;
    };
    void edge_search(int i, std::vector<pcl::Correspondence> &col, std::vector<Edge> &edges, std::vector<int> &cor_index_query);
    double ComputeViewAngle(int query1, int query2);
    std::vector<std::vector<pcl::Correspondence>> net_vertex_;
    std::vector<std::vector<Edge>> net_edge_;
    View_Graph::Ptr view_graph_;
    td::pclib::PointCloudPtr complete_model_;
    std::vector<pcl::Correspondence> far_away_vertexs_;
    pcl::visualization::PCLVisualizer net_viewer_;

    /*
  Structure for sorting vectors with pcl::Correspondences by index_query
*/
    struct less_than_query
    {
        inline bool operator() (const pcl::Correspondence corr1, const pcl::Correspondence corr2)
        {
            return (corr1.index_query < corr2.index_query);
        }
    };
    /*
  Structure for sorting vectors with pcl::Correspondences by distance
*/
    struct less_than_distance_struct
    {
        inline bool operator() (const pcl::Correspondence corr1, const pcl::Correspondence corr2)
        {
            return (corr1.distance < corr2.distance);
        }
    };
    /*
Structure for sorting vectors with pair<int, pcl::Correspondences> by distance
*/
    struct less_than_distance_pair
    {
        inline bool operator() (const std::pair<int,pcl::Correspondence> corr1, const std::pair<int,pcl::Correspondence> corr2)
        {
            return (corr1.second.distance < corr2.second.distance);
        }
    };
/*
    Structure for sorting vectors with NetClustering by distance
    */
    struct less_than_most_neighbor_groups_average_distance
    {
        inline bool operator() (const NetClustering net1, const NetClustering net2)
        {
            return (net1.most_neighbor_groups_average_distance < net2.most_neighbor_groups_average_distance);
        }
    };
    /*
    Structure for sorting vectors with NeighborGroups by average_distance
    */
    struct less_than_average_distance
    {
        inline bool operator() (const NeighborGroup ng1, const NeighborGroup ng2)
        {
            return (ng1.average_distance < ng2.average_distance);
        }
    };


    bool find_cmp(int c1, pcl::Correspondence &c2){
        return c1 == c2.index_query;
    }
    // comparision function for min_element
//    bool less_than_distance =[](const pcl::Correspondence &a, const pcl::Correspondence &b){
//        return a.distance < b.distance;
//    };
public:
    void ShowMostNeighborVertex(std::vector<NetClustering> &net_clusterings);

private:
    double RelativeDist(pcl::Correspondence &smaller_cor, pcl::Correspondence &larger_cor);
    double RelativeAverageDist(NeighborGroup &smaller_neighbor_group, NeighborGroup &larger_neighbor_group);
};


#endif //OBJECT_POSE_ESTIMATION_GLOBALCORRESPONDENCE_H
