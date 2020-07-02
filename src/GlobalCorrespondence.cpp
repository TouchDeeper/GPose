//
// Created by wang on 19-11-19.
//

#include <TdLibrary/tool/tic_toc.h>
#include "GlobalCorrespondence.h"

void GlobalCorrespondence::TransferData(std::vector<pcl::Correspondence> &correspondences, View_Graph::Ptr view_graph, td::pclib::PointCloudPtr complete_model) {
    correspondences_ = correspondences;
    for (int i = 0; i < correspondences_.size(); ++i) {
        correspondences_index_query_.push_back(correspondences_[i].index_query);
    }
    view_graph_ = view_graph;
    complete_model_ = complete_model;
    if(VISUAL){
        net_viewer_.setWindowName("viewer");
        net_viewer_.setSize(WINDOW_SIZE[0], WINDOW_SIZE[1]);
        net_viewer_.setBackgroundColor(VIEWSTYLE.background_color[0],VIEWSTYLE.background_color[1],VIEWSTYLE.background_color[2]);
        double sphere_raidus = 0.02;
        view_graph_->add_graph_to_viewer (net_viewer_, sphere_raidus/2, 0, false, false);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(complete_model_,VIEWSTYLE.model_rgb[0],VIEWSTYLE.model_rgb[1],VIEWSTYLE.model_rgb[2]);
        net_viewer_.addPointCloud<pcl::PointXYZ> (complete_model_,rgb);
        net_viewer_.addCoordinateSystem(0.05);
    }
}

void GlobalCorrespondence::NetCluster() {
    std::vector<int> cor_index_query;
    cor_index_query = correspondences_index_query_;
    td::TicToc net_timer;
//    int i=0;
    for (int i = 0; i < cor_index_query.size(); ++i) {

//        Node node(i,*iter);
        std::vector<pcl::Correspondence> vertexs;
        std::vector<Edge> edges;
        if(cor_index_query[i] == -1)
            continue;
        edge_search(i, vertexs, edges, cor_index_query);
        net_vertex_.push_back(vertexs);
        net_edge_.push_back(edges);
    }
    std::vector<std::pair<int,pcl::Correspondence>> best_cors;
    for (int k = 0; k < net_vertex_.size(); ++k) {
        auto best_cor = std::min_element(net_vertex_[k].begin(), net_vertex_[k].end(),less_than_distance_struct());
        best_cors.emplace_back(std::make_pair(k,*best_cor));
    }
    std::sort(best_cors.begin(),best_cors.end(),less_than_distance_pair());
    for (int l = 0; l < best_cors.size(); l++) {
        if((best_cors[l].second.distance-best_cors[0].second.distance)/best_cors[0].second.distance > 0.2)
            best_cors[l].first = -1;
    }
    std::vector<std::vector<pcl::Correspondence>> net_vertex_backup;
    net_vertex_backup = net_vertex_;
    net_vertex_.clear();
    for (const auto &item : best_cors) {
        if(item.first >= 0)
            net_vertex_.push_back(net_vertex_backup[item.first]);
    }

    if(VISUAL)
        ShowNetClusterResult();
    // filter net_vertex
    std::vector<std::vector<pcl::Correspondence>> big_enough_net;
    std::vector<int> small_net;
    for (int j = 0; j < net_vertex_.size(); ++j) {
        if(net_vertex_[j].size() > 3)
            big_enough_net.push_back(net_vertex_[j]);
//        else
//            small_net.push_back(j);
    }
    // 对于有多个足够大的net,就把小net删除，否则都保留
    if(big_enough_net.size() >=1 ){
        net_vertex_.clear();
        net_vertex_ = big_enough_net;
        if(VISUAL)
            ShowNetClusterResult();
        NetVertexFilter();

    } else
    {
        //每个net找一个distance最小的，然后再用角度进行筛选，再用distance选出最小的三个

        std::vector<pcl::Correspondence> min_vertexs_in_net;
        for (int i = 0; i < net_vertex_.size(); ++i) {
            pcl::Correspondence min_vertex = net_vertex_[i][0];
            for (int j = 1; j < net_vertex_[i].size(); ++j) {
                if(net_vertex_[i][j].distance < min_vertex.distance)
                    min_vertex = net_vertex_[i][j];
            }
            min_vertexs_in_net.push_back(min_vertex);

        }

        FarAwayVertex(min_vertexs_in_net,far_away_vertexs_);
        correspondences_ = far_away_vertexs_;

    }


//    if(big_enough_net.size() == 1)

    std::cout<<"net cluster cost "<<net_timer.toc()<<" ms"<<std::endl;

}
void GlobalCorrespondence::FarAwayVertex(std::vector<pcl::Correspondence> &vertexs, std::vector<pcl::Correspondence> &far_away_vertex)
{

//    Eigen::MatrixXd relative_matrix = Eigen::MatrixXd::Zero(vertexs.size(), vertexs.size());
//
//    for (int i = 0; i < vertexs.size()-1; ++i) {
//        for (int j = i+1; j < vertexs.size(); ++j) {
//            double view_angle = ComputeViewAngle(vertexs[i].index_query, vertexs[j].index_query);
//            if(view_angle > M_PI/4)
//                relative_matrix(i,j) = 1;
//        }
//    }
    std::sort(vertexs.begin(), vertexs.end(), less_than_distance_struct());
    far_away_vertex.push_back(vertexs[0]);
    for (int k = 1; k < vertexs.size(); ++k) {
        double relative_dist = RelativeDist(far_away_vertex[0],vertexs[k]);
        if(relative_dist <0.2)//the distance need approx
        {
            bool is_far_and_similar_dist_vertex = true;
            for (int i = 0; i < far_away_vertex.size(); ++i) {

                if(view_graph_->get_relative_matrix_value(vertexs[k].index_query,far_away_vertex[i].index_query) == 0 ){
                    is_far_and_similar_dist_vertex = false;
                    break;
                }
            }
            if(is_far_and_similar_dist_vertex)
            {
                far_away_vertex.push_back(vertexs[k]);
                if(far_away_vertex.size() >= 3)//将大小限制在３个以内
                    break;
            }

        } else
            break;

    }

}

double GlobalCorrespondence::ComputeViewAngle(int query1, int query2){

    pcl::PointXYZ v1 = view_graph_->get_viewpoint(query1);
    pcl::PointXYZ v2 = view_graph_->get_viewpoint(query2);
    Eigen::Vector3d v1_e(v1.x, v1.y, v1.z);
    Eigen::Vector3d v2_e(v2.x, v2.y, v2.z);
    Eigen::Vector3d v1_v2 = v2_e - v1_e;
    double v1_norm = v1_e.norm();
    double v2_norm = v2_e.norm();
    double v1_v2_norm = v1_v2.norm();

    double angle = acos((v1_norm * v1_norm + v2_norm * v2_norm - v1_v2_norm * v1_v2_norm)/(2*v1_norm*v2_norm));
    return angle;

}
void GlobalCorrespondence::edge_search(int index, std::vector<pcl::Correspondence> &col, std::vector<Edge> &edges, std::vector<int> &cor_index_query) {
    int query = cor_index_query[index];
    col.push_back(correspondences_[index]);

    std::vector<int> neighbor = view_graph_->get_neighbors(query);
    cor_index_query[index] = -1;
//    std::cout<<"*iter = "<<*iter<<std::endl;

    for (int i = 0; i < neighbor.size(); ++i) {
        auto iter_neighbor = std::find(cor_index_query.begin(), cor_index_query.end(), neighbor[i]);
        if(iter_neighbor != cor_index_query.end())
        {
            std::cout<<query<<"'s neighbor = "<<neighbor[i]<<std::endl;
            edges.emplace_back(Edge(query, neighbor[i]));
            edge_search(iter_neighbor - cor_index_query.begin(), col, edges, cor_index_query);
        }
    }
}

void GlobalCorrespondence::NetVertexFilter(){
    //　计算每个net中每个vertex的neighbor group
    std::vector<NetClustering> net_clusterings;
    std::vector<std::vector<double>> net_vertex_neighbor_average_distance;
    for (int n = 0; n < net_vertex_.size(); ++n) {
        auto net_vertex_n = net_vertex_[n];
        NetClustering net_clustering_n;
//        std::vector<double> net_vertex_n_neighbor_average_distance;
        for (int i = 0; i < net_vertex_n.size(); ++i) {
            auto net_vertex_n_i = net_vertex_n[i];
            auto neighbors = view_graph_->get_neighbors(net_vertex_n_i.index_query);
//            int neighbor_count = 0;
//            double neighbor_average_distance = net_vertex_n_i.distance;
            NeighborGroup neighbor_group;
            neighbor_group.push_back(net_vertex_n_i);//每个neighbor_group的第一个数据都是寻找neighbor的这个vertex
            for (int j = 0; j < neighbors.size(); ++j) {
                auto neighbors_in_net_n_iter = std::find_if(net_vertex_n.begin(), net_vertex_n.end(), boost::bind(&GlobalCorrespondence::find_cmp, this, neighbors[j], _1));
                if(neighbors_in_net_n_iter != net_vertex_n.end()){
                    neighbor_group.push_back(*neighbors_in_net_n_iter);
                }
            }
//            neighbor_average_distance /= (neighbor_count + 1);
//            net_vertex_n_neighbor_average_distance.push_back(neighbor_average_distance);
            net_clustering_n.push_back(neighbor_group);
        }
        net_clusterings.push_back(net_clustering_n);
    }

    int min_first_groups_index = 0;
    double min_first_groups_distance = 1000;
    for (int k1 = 0; k1 < net_clusterings.size(); ++k1) {

        net_clusterings[k1].ComputeAverageDistances();
        net_clusterings[k1].FindMostNeighborGroups();
        net_clusterings[k1].ComputeMostNeighborGroupsAverageDistance();
    }
    std::sort(net_clusterings.begin(), net_clusterings.end(), less_than_most_neighbor_groups_average_distance());
    if(VERBOSE){
        for (int k = 0; k < net_clusterings.size(); ++k) {
            for (int i = 0; i <net_clusterings[k].most_neighbor_groups.size() ; ++i) {
                std::cout<< "net_clustering "<<k<<" most_neighbor_group "<<i<<" 's average distance = "<<net_clusterings[k].most_neighbor_groups[i].average_distance<<std::endl;
            }
        }
    }
    if(VISUAL)
    {
        ShowMostNeighborVertex(net_clusterings);

    }

//    temp_clustering.most_neighbor_groups.push_back(net_clusterings[0].most_neighbor_groups[0]);
    NeighborGroup back_net_clusterings_0_most_neighbor_group_0 = net_clusterings[0].most_neighbor_groups[0];
    correspondences_.clear();
    //加入到correspondences_后就删除，免得后面再次加入到temp_groups中
    correspondences_.push_back(net_clusterings[0].most_neighbor_groups[0].get_correspondence(0));
    net_clusterings[0].most_neighbor_groups.erase(0 + net_clusterings[0].most_neighbor_groups.begin());
    //将每个clustering都选出一个来放到最终的correspondences_中
    for (int l1 = 1; l1 < net_clusterings.size(); ++l1) {//net_clusterings[0] has a vertex in correspondences_, so we begin from net_clusterings[1]
        for (int i = 0; i < net_clusterings[l1].most_neighbor_groups.size(); ++i) {
//            std::cout<<"best average_dist = "<<back_net_clusterings_0_most_neighbor_group_0.average_distance<<" now group's average_dist = "<<net_clusterings[l1].most_neighbor_groups[i].average_distance<<std::endl;
            double relative_dist = RelativeAverageDist(back_net_clusterings_0_most_neighbor_group_0, net_clusterings[l1].most_neighbor_groups[i]);
            if(relative_dist < 0.2){
                bool is_far_and_similar_dist_vertex = true;
                for (int j = 0; j < correspondences_.size(); ++j) {
                    if(view_graph_->get_relative_matrix_value(correspondences_[j].index_query, net_clusterings[l1].most_neighbor_groups[0].correspondences[0].index_query) == 0)
                    {
                        is_far_and_similar_dist_vertex = false;
                        break;
                    }
                }
                if(is_far_and_similar_dist_vertex){
                    correspondences_.push_back(net_clusterings[l1].most_neighbor_groups[i].correspondences[0]);
                    net_clusterings[l1].most_neighbor_groups.erase(i + net_clusterings[l1].most_neighbor_groups.begin());
                    break;
                }
            }


        }
    }
    //  big enough cluster >=  3
    if(correspondences_.size() >=3)
    {
        correspondences_.resize(3);
        if(VISUAL)
            ShowCorrespondences();
        return;
    }


    std::vector<NeighborGroup> temp_groups;
    for (const auto &item : net_clusterings) {
        for (const auto &neighbor_groups : item.most_neighbor_groups) {
            temp_groups.push_back(neighbor_groups);
        }
    }
    std::sort(temp_groups.begin(),temp_groups.end(),less_than_average_distance());
    if(VERBOSE){
        for (int l = 0; l < temp_groups.size(); ++l) {
            std::cout<<"temp groups "<<l<<"  's average distance = "<<temp_groups[l].average_distance<<std::endl;
        }
    }

    for (auto &tempGroup : temp_groups) {
        double relative_average_dist = RelativeAverageDist(back_net_clusterings_0_most_neighbor_group_0,tempGroup);
        if(relative_average_dist < 0.2){
            bool is_far_vertex = true;
            for (const auto &corr : correspondences_) {
                if(view_graph_->get_relative_matrix_value(corr.index_query, tempGroup.get_correspondence(0).index_query) == 0){
                    is_far_vertex = false;
                    break;
                }
            }
            if(is_far_vertex)
            {
                correspondences_.push_back(tempGroup.get_correspondence(0));
                if(correspondences_.size() == 3)
                {
                    if(VISUAL)
                        ShowCorrespondences();
                    return;
                }

            }
        }


    }
    if(VISUAL)
        ShowCorrespondences();
}
double GlobalCorrespondence::RelativeDist(pcl::Correspondence &smaller_cor, pcl::Correspondence &larger_cor){
    assert(larger_cor.distance >= smaller_cor.distance);
    return (larger_cor.distance - smaller_cor.distance)/smaller_cor.distance;

}
double GlobalCorrespondence::RelativeAverageDist(NeighborGroup &smaller_neighbor_group, NeighborGroup &larger_neighbor_group){
//    assert(larger_neighbor_group.average_distance >= smaller_neighbor_group.average_distance);
    return (larger_neighbor_group.average_distance - smaller_neighbor_group.average_distance)/smaller_neighbor_group.average_distance;

}

//double GlobalCorrespondence::RelativeDist(pcl::Correspondence &smaller_cor, pcl::Correspondence larger_cor){
//    assert(larger_cor.distance > smaller_cor.distance);
//    return (larger_cor.distance - smaller_cor.distance)/smaller_cor.distance;
//
//}
void GlobalCorrespondence::ShowMostNeighborVertex(std::vector<NetClustering> &net_clusterings){
//    pcl::visualization::PCLVisualizer viewer("most neighbor vertex");
//    viewer.setBackgroundColor(VIEWSTYLE.background_color[0],VIEWSTYLE.background_color[1],VIEWSTYLE.background_color[2]);
    double sphere_raidus = 0.02;
//    view_graph_->add_graph_to_viewer (viewer, sphere_raidus/2, 0, false, false);
    std::vector<std::vector<int>> rgbs;
    std::vector<int> temp;
    temp.push_back(255);
    temp.push_back(0);
    temp.push_back(0);
    rgbs.push_back(temp);
    temp[0] = 0;
    temp[1] = 255;
    rgbs.push_back(temp);
    temp[1] = 0;
    temp[2] = 255;
    rgbs.push_back(temp);
    view_graph_->reset_node_sphere(net_viewer_);

    for (int i = 0; i < net_clusterings.size(); ++i) {

//        int r = lround(td::UniformSampling<float>(0,255));
//        int g = lround(td::UniformSampling<float>(0,255));
//        int b = lround(td::UniformSampling<float>(0,255));
        int r = rgbs[i][0];
        int g = rgbs[i][1];
        int b = rgbs[i][2];

//        std::cout<<"rgb = ["<<r<<", "<<g<<", "<<b<<"]"<<std::endl;
        for (int j = 0; j < net_clusterings[i].most_neighbor_groups.size(); ++j) {
            std::stringstream ss;
            int index = net_clusterings[i].most_neighbor_groups[j][0].index_query;
            ss << "Sphere" << index;
            net_viewer_.updateSphere(view_graph_->get_viewpoint(index), sphere_raidus, r,g,b, ss.str());
//            std::stringstream ss;
//            ss << "net link"<<net_edge_[i][j].index1<<net_edge_[i][j].index2;
//            viewer.addLine (view_graph_->get_viewpoint(net_edge_[i][j].index1), view_graph_->get_viewpoint(net_edge_[i][j].index2), r, g, b, ss.str (), 0);
//            viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, VIEWSTYLE.point_size, ss.str(), 0);


        }
    }
    // Add complete model to viewer
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(complete_model_,VIEWSTYLE.model_rgb[0],VIEWSTYLE.model_rgb[1],VIEWSTYLE.model_rgb[2]);
//
//    viewer.addPointCloud<pcl::PointXYZ> (complete_model_,rgb);
//    viewer.addCoordinateSystem(0.05);
    std::cout << "Press Q to continue..." << std::endl;
    net_viewer_.spin ();


}
void GlobalCorrespondence::ShowCorrespondences(){
//    pcl::visualization::PCLVisualizer viewer("final views");
//    viewer.setBackgroundColor(VIEWSTYLE.background_color[0],VIEWSTYLE.background_color[1],VIEWSTYLE.background_color[2]);
    double sphere_raidus = 0.02;
//    view_graph_->add_graph_to_viewer(viewer,sphere_raidus/2, 0, false, false);
    std::vector<std::vector<int>> rgbs;
    std::vector<int> temp;
    temp.push_back(255);
    temp.push_back(0);
    temp.push_back(0);
    rgbs.push_back(temp);
    temp[0] = 0;
    temp[1] = 255;
    rgbs.push_back(temp);
    temp[1] = 0;
    temp[2] = 255;
    rgbs.push_back(temp);
    temp[0] = 123;
    temp[1] = 25;
    temp[2] = 80;
    rgbs.push_back(temp);
    view_graph_->reset_node_sphere(net_viewer_);

    for (int j = 0; j < correspondences_.size(); ++j) {
        int r = rgbs[j][0];
        int g = rgbs[j][1];
        int b = rgbs[j][2];
        std::stringstream ss;
        ss << "Sphere" << correspondences_[j].index_query;
        net_viewer_.updateSphere(view_graph_->get_viewpoint(correspondences_[j].index_query), sphere_raidus, r,g,b, ss.str());
//            std::stringstream ss;
//            ss << "net link"<<net_edge_[i][j].index1<<net_edge_[i][j].index2;
//            viewer.addLine (view_graph_->get_viewpoint(net_edge_[i][j].index1), view_graph_->get_viewpoint(net_edge_[i][j].index2), r, g, b, ss.str (), 0);
//            viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, VIEWSTYLE.point_size, ss.str(), 0);

    }
    // Add complete model to viewer
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(complete_model_,VIEWSTYLE.model_rgb[0],VIEWSTYLE.model_rgb[1],VIEWSTYLE.model_rgb[2]);
//
//    viewer.addPointCloud<pcl::PointXYZ> (complete_model_,rgb);
//    viewer.addCoordinateSystem(0.05);
    std::cout << "Press Q to continue..." << std::endl;
    net_viewer_.spin ();

}
void GlobalCorrespondence::ShowNetClusterResult() {
//    pcl::visualization::PCLVisualizer viewer("Net cluster");
//    viewer.setBackgroundColor(VIEWSTYLE.background_color[0],VIEWSTYLE.background_color[1],VIEWSTYLE.background_color[2]);
    double sphere_raidus = 0.02;
//    view_graph_->add_graph_to_viewer (viewer, sphere_raidus/2, 0, false, false);
    std::vector<std::vector<int>> rgbs;
    std::vector<int> temp;
    temp.push_back(255);
    temp.push_back(0);
    temp.push_back(0);
    rgbs.push_back(temp);
    temp[0] = 0;
    temp[1] = 255;
    rgbs.push_back(temp);
    temp[1] = 0;
    temp[2] = 255;
    rgbs.push_back(temp);
    temp[0] = 123;
    temp[1] = 25;
    temp[2] = 80;
    rgbs.push_back(temp);
    view_graph_->reset_node_sphere(net_viewer_);
    for (int i = 0; i < net_vertex_.size(); ++i) {

//        int r = lround(td::UniformSampling<float>(0,255));
//        int g = lround(td::UniformSampling<float>(0,255));
//        int b = lround(td::UniformSampling<float>(0,255));
        int r = rgbs[i][0];
        int g = rgbs[i][1];
        int b = rgbs[i][2];

//        std::cout<<"rgb = ["<<r<<", "<<g<<", "<<b<<"]"<<std::endl;
        for (int j = 0; j < net_vertex_[i].size(); ++j) {
            std::stringstream ss;
            ss << "Sphere" << net_vertex_[i][j].index_query;
            net_viewer_.updateSphere(view_graph_->get_viewpoint(net_vertex_[i][j].index_query), sphere_raidus, r,g,b, ss.str());
//            std::stringstream ss;
//            ss << "net link"<<net_edge_[i][j].index1<<net_edge_[i][j].index2;
//            viewer.addLine (view_graph_->get_viewpoint(net_edge_[i][j].index1), view_graph_->get_viewpoint(net_edge_[i][j].index2), r, g, b, ss.str (), 0);
//            viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, VIEWSTYLE.point_size, ss.str(), 0);


        }
    }
    // Add complete model to viewer
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(complete_model_,VIEWSTYLE.model_rgb[0],VIEWSTYLE.model_rgb[1],VIEWSTYLE.model_rgb[2]);

//    viewer.addPointCloud<pcl::PointXYZ> (complete_model_,rgb);
//    viewer.addCoordinateSystem(0.05);
//    std::cout << "Press Q to continue..." << std::endl;
    net_viewer_.spin ();

}
void GlobalCorrespondence::ShowGlobalCorrespondenceInViewGraph(){

    // Add graph to viewer
    pcl::visualization::PCLVisualizer viewer ("Viewer");
    viewer.setBackgroundColor(VIEWSTYLE.background_color[0],VIEWSTYLE.background_color[1],VIEWSTYLE.background_color[2]);
    view_graph_->add_graph_to_viewer (viewer, 0.02, 0, true, true);
    AddCorrespondenceNeighborLine(viewer);
    // Add complete model to viewer
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(complete_model_,VIEWSTYLE.model_rgb[0],VIEWSTYLE.model_rgb[1],VIEWSTYLE.model_rgb[2]);
    viewer.addPointCloud<pcl::PointXYZ> (complete_model_,rgb);
    viewer.addCoordinateSystem(0.05);
    std::cout << "Press Q to continue..." << std::endl;
    viewer.spin ();
}
void GlobalCorrespondence::AddCorrespondenceNeighborLine(pcl::visualization::PCLVisualizer &visu){
    for (int i = 0; i < correspondences_index_query_.size(); ++i) {
//        std::cout<<"candidate view index = "<<model_->correspondences[i].index_query<<std::endl;
        std::vector<int> neighbor = view_graph_->get_neighbors(correspondences_index_query_[i]);
        for (int j = 0; j < neighbor.size(); ++j) {
            bool neighbor_in_cor = std::binary_search(correspondences_index_query_.begin(), correspondences_index_query_.end(), neighbor[j]);
            if(neighbor_in_cor){
                std::stringstream ss;
                ss << "neighbor link"<<correspondences_index_query_[i]<<neighbor[j];
                visu.addLine (view_graph_->get_viewpoint(correspondences_index_query_[i]), view_graph_->get_viewpoint(neighbor[j]), 255, 0, 0, ss.str (), 0);
                visu.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, VIEWSTYLE.point_size, ss.str(), 0);
            }

        }
    }
}
