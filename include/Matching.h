//
// Created by wang on 19-3-22.
//

#ifndef LOCALPIPELINE_MATCHING_H
#define LOCALPIPELINE_MATCHING_H
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/shot.h>
#include <pcl/console/parse.h>
#include <pcl/recognition/cg/hough_3d.h>


#include <pcl/features/board.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>


#include <Eigen/Core>
// Eigen 几何模块
#include <Eigen/Geometry>

#include "config_reader.h"
#include <TdLibrary/PCL/global_feature.h>
#include <TdLibrary/PCL/local_features.h>
#include <TdLibrary/tool/random_tool.hpp>
#include <TdLibrary/PCL/tools.h>
#include <TdLibrary/PCL/cluster_segmentation.hpp>
#include <TdLibrary/PCL/viewer.hpp>
#include "sample_consensus_prerejective.h"
//#include "common_typedef.h"
#include <TdLibrary/td_eigen/eigen_common_typedef.h>

#include "match/LocalPipe.h"
#include "parameters.h"
//#include "include/CsvReader.h"
#include "view_graph/view_graph.h"
#include "access_model_data.h"
#include "ModelDataStructure.h"
#include "SceneDataStructure.h"
//#include "add_model/render_synthetic_views.h"

#include <opencv2/opencv.hpp>
class Matching{

public:
    /**
     * construct function
     */
    Matching();
    /**
     * load the scene image of dataset
     */
    void LoadSceneImage();
    void depthToPCD();
    /**
     * convert the depth image to the point cloud
     * @param depth depth image
     * @param mask the mask region need to be converted to point cloud
     * @param point_cloud the result point cloud
     * @param fx
     * @param fy
     * @param cx
     * @param cy
     * @param scale unit scale
     */
    void depthToPointCloud(cv::Mat& depth, cv::Mat& mask, td::pclib::PointCloudPtr& point_cloud, double fx, double fy, double cx, double cy, double scale);
    /**
     * pre process the scene
     */
    void processScene(td::pclib::PointCloudPtr& scene);
    /**
     * load the scene model, convert the units, downsampling
     *
     */
    void LoadScene();
    /**
    * load the model, convert the units, downsampling
    * @param model_path the scene_path
    */
    void LoadModel();
    /**
     * compute the scene descriptors
     */
    void SceneLocalDescriptorsTrain();

    void PointPick();
    void pointPickingEventOccurred(const pcl::visualization::PointPickingEvent& event, void* viewer_void);
    /**
    * compute the model descriptors
    */
    void ModelDescriptorsTrain();
    /**
     * match the local descriptors
     */
    void MatchLocalDescriptors();
    /**
     * match the global descriptors
     */
    void MatchGlobalDescriptors();
    void ShowGlobalCorrespondenceInViewGraph();
    void GlobalCorrespondenceFilter();

    void SetSpecifiedGlobalResult();
    /**
     * show corrsepondence
     * @param correspondence object for storing the correspondence of every candidate transformation
     * @param model_view offline trainning model view
     * @param scene_view scene view captured by the camera
     * @param all_transformations all the candidate tranformations
     */
    void ShowCorrespondence(const std::vector<std::pair<std::vector<std::vector<int>>,float>>* correspondence, const td::pclib::PointNCloudPtr model_view, const td::pclib::PointNCloudPtr scene_view,
                            const td::VecMat4* all_transformations);
    /**
     * get the poses of snapshots
     * @return vector for storing the pose of snapshot
     */
    td::VecMat4 GetPoses();
    /**
     * get the camera position in the tessellated sphere
     * @return camera position in the tessellated sphere
     */
    td::VecMat3 GetCamPos();
    /**
     * get vector for storing the model with normal
     * @return vector for storing the model with normal
     */
    std::vector<td::pclib::PointNCloudPtr> GetModelWithNormals();
    /**
     * get the best match snapshot id
     * @return the best match snapshot id
     */
    int GetBestMatchSnapshotId();
    /**
     * show th point cloud with normal
     * @param pointNormal point cloud with normal
     */
    void PointNormalVisual(td::pclib::PointNCloudPtr pointNormal);
    /**
    * \brief compute the global descriptor of scene
    */
    void SceneGlobalDescriptorTrain();
    /**
    * compute model global descriptor
    */
//    void ModelGlobalDescriptorTrain();

    /**
    * set global descriptor type
    * @param global_descriptors_type local descriptors type
    */
    void set_global_descripor_type(std::string global_descriptors_type)
    {
        global_descriptors_type_ = global_descriptors_type;
    }

    /**
    Computes the l1-norm of the difference between two histograms
    @param f1 The first histogram
    @param f2 the second histogram
    */
    float l1_norm (td::pclib::EsfDescriptor f1, td::pclib::EsfDescriptor f2);

    /**
	Matches an object to clusters OR clusters to object and returns a vector of vector of poses of the object in each cluster
	@param object The object to estimate pose of
	@param object_features The feature cloud of the object
	@param clusters Vector of clusters
	@param object_to_cluster If true, matches object to cluster, otherwise matches clusters to object
	@param transformations[out] Vector of vector of transformations for each cluster
    */
//    void estimate_poses_for_clusters(PointNCloudPtr object, FpfhCloudPtr object_features, std::vector<PointNCloudPtr> clusters, bool object_to_cluster, std::vector<std::vector<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double> > >* transformations);

/**
	Estimates the pose of an object in a scene
	@param transformation[out] The transformation (pose) of the object
	@param all_transformations[out] Vector with all transformations with inliers above inlier_fraction
	@return Vector of all inliers
 */
    std::vector<int> estimate_pose(Eigen::Matrix4d* transformation,
                                   td::VecMat4* all_transformations, std::vector<double>* inlier_ratio);
    /**
     * process the  ICP
     * @param guess_and_result object for storing the initial guess transformations and the result
     * @param source is the point cloud to be registered to the target.
     * @param target is target point cloud
     */
    void RunIcp(td::VecMat4* guess_and_result, td::pclib::PointCloudPtr source, td::pclib::PointCloudPtr target);
    /**
     * process the trimmed ICP(handle the partially overlapping point sets registration problem)
     * @param guess_and_result object for storing the initial guess transformations and the result
     * @param init_ratio the initial ratio of the overlap
     * @param source is the point cloud to be registered to the target.
     * @param target is target point cloud
     */
    void TrimmedIcp(td::VecMat4* guess_and_result, float init_ratio, td::pclib::PointCloudPtr source, td::pclib::PointCloudPtr target);

    /**
	Estimates the pose of an object in a scene using GA algorithm
	@param transformation[out] The transformation (pose) of the object
	@param all_transformations[out] Vector with all transformations with inliers above inlier_fraction
	@return Vector of all inliers
 */
//    std::vector<int> estimate_pose_ga(Eigen::Matrix4d* transformation,
//                                      td::VecMat4* all_transformations, std::vector<double>* inlier_ratio);
//    void ShowRangeOfGaParameter(const td::VectorPairMatrix4dDouble* first_two_tansformation);
    /**
	Estimates the pose of an object in a scene
	@param transformation[out] The transformation (pose) of the object
	@param all_transformations[out] Vector with all transformations with inliers above inlier_fraction
	@return Vector of all inliers
 */
//    std::vector<int> estimate_pose_openGA(Eigen::Matrix4d* transformation,
//                                          td::VecMat4* all_transformations, std::vector<double>* inlier_ratio);
    void LocalPipeMatch();

    void ShowCandidate();
    void CheckResults(std::string result_file);
    void CheckViewGraph();
    void graph_viewer (View_Graph::Ptr graph, td::pclib::PointCloudPtr complete_model);
    void FusionViews(std::vector<int> &multi_view_indexs);
    void CandidateAddNeighbors(const int &view_candidate_indice, td::pclib::PointNCloudPtr partial_model);
    void LoadCheckResult();
private:

    Config_Reader* cr_;
    td::VecMat4 poses_;
    td::VecMat3 cam_pos_;
    Eigen::Matrix4d m_T_s_;
    int best_match_model_snapshots_;
    std::string local_descriptors_type_;
    std::string global_descriptors_type_;


    ModelData::Ptr model_; //object for storing multiple models


    SceneData::Ptr scene_;
    std::vector<Result::Ptr> GA_results_;
    std::vector<Result::Ptr> GA_ICP_results_;
    std::vector<Result::Ptr> ICP_results_;
//    LocalPipe local_pipe_;
    int fusion_level_;
    Access_Model_Data* ac_;
    std::string base_dir_;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
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



};
#endif //LOCALPIPELINE_MATCHING_H
