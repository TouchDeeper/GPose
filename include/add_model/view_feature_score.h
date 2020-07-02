#ifndef VIEW_FEATURE_SCORE_H
#define VIEW_FEATURE_SCORE_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/filters/extract_indices.h>

class View_Feature_Score {
  private:
    typedef pcl::PointNormal Point_N;
    typedef pcl::PointCloud<Point_N> Point_Cloud_N;

    typedef pcl::PointXYZRGB Point_RGB;
    typedef pcl::PointCloud<Point_RGB> Point_Cloud_RGB;

    typedef pcl::FPFHSignature33 Feature;
    typedef pcl::PointCloud<Feature> Feature_Cloud;

    /**
      Finds points with persistent features in cloud
      @param cloud The cloud to find persistent features in
      @param output_indices The indices of the points with persistent features
    */
    void
    compute_persistent_features(Point_Cloud_N::Ptr cloud, boost::shared_ptr<std::vector<int> > output_indices);

    /**
      Computes a score for a view based on number of points where a persistent point is worth 10 times as much
      @param view The view to compute the score for
      @return The score
    */
    int
    compute_view_score(Point_Cloud_N::Ptr view);

  public:
    /**
      Computes a score for each view in the range 0-1, the best view will always have score 1, and the worst score will always have score 0
      @param views The vector of views
      @return Vector of scores between 0 and 1
    */
    std::vector<double>
    compute_view_score_normalized(std::vector<Point_Cloud_N::Ptr> views);

    /**
      Computes the persistent points of a view and returns persistent and non-peristent point clouds for visualization purposes
      @param view The view
      @param persistent_cloud The persistent point cloud
      @param non_persistent_cloud The non-persistent point cloud
    */
    void
    get_persistent_points(Point_Cloud_N::Ptr view, Point_Cloud_N::Ptr persistent_cloud, Point_Cloud_N::Ptr non_persistent_cloud);
};

#endif
