#include "add_model/view_feature_score.h"

/**
  Finds points with persistent features in cloud
  @param cloud The cloud to find persistent features in
  @param output_indices The indices of the points with persistent features
*/
void
View_Feature_Score::compute_persistent_features(View_Feature_Score::Point_Cloud_N::Ptr cloud, boost::shared_ptr<std::vector<int> > output_indices) {
  pcl::MultiscaleFeaturePersistence<Point_N, Feature> feature_persistence;
  std::vector<float> scale_values;
  scale_values.push_back(0.010);
  feature_persistence.setScalesVector(scale_values);
  feature_persistence.setAlpha(1.30f);

  pcl::FPFHEstimation<Point_N, Point_N, Feature>::Ptr fpfh_estimation (new pcl::FPFHEstimation<Point_N, Point_N, Feature>());
  fpfh_estimation->setInputCloud(cloud);
  fpfh_estimation->setInputNormals(cloud);

  feature_persistence.setFeatureEstimator(fpfh_estimation);
  feature_persistence.setDistanceMetric(pcl::CS);

  Feature_Cloud::Ptr output_features (new Feature_Cloud);
  feature_persistence.determinePersistentFeatures(*output_features, output_indices); // output_indices needs to be boost::shared_ptr
}

/**
  Computes a score for a view based on number of points where a persistent point is worth 10 times as much
  @param view The view to compute the score for
  @return The score
*/
int
View_Feature_Score::compute_view_score(View_Feature_Score::Point_Cloud_N::Ptr view) {
  // Compute persistent indices
  boost::shared_ptr<std::vector<int> > persistent_indices (new std::vector<int>);
  compute_persistent_features(view, persistent_indices);

  int score = view->points.size();
  score += persistent_indices->size() * 9; // 9 because all persistent points have been added once in the previous line

  return score;
}

/**
  Computes a score for each view in the range 0-1, the best view will always have score 1, and the worst score will always have score 0
  @param views The vector of views
  @return Vector of scores between 0 and 1
*/
std::vector<double>
View_Feature_Score::compute_view_score_normalized(std::vector<View_Feature_Score::Point_Cloud_N::Ptr> views) {
  std::vector<double> scores;
  int max_score = 0;
  int min_score = -1; // -1 because it needs to be initialized with the first score
  for(int i = 0; i < views.size(); i++) {
    int score = compute_view_score(views[i]);
    scores.push_back(score);

    if(score > max_score)
      max_score = score;
    if(min_score == -1 || score < min_score)
      min_score = score;
  }

  for(int i = 0; i < scores.size(); i++) {
    scores[i] -= min_score;
    scores[i] /= (max_score - min_score);
  }

  return scores;
}

/**
  Computes the persistent points of a view and returns persistent and non-peristent point clouds
  @param view The view
  @param persistent_cloud The persistent point cloud
  @param non_persistent_cloud The non-persistent point cloud
*/
void
View_Feature_Score::get_persistent_points(View_Feature_Score::Point_Cloud_N::Ptr view, View_Feature_Score::Point_Cloud_N::Ptr persistent_cloud, View_Feature_Score::Point_Cloud_N::Ptr non_persistent_cloud) {
  // Compute persistent indices
  boost::shared_ptr<std::vector<int> > persistent_indices (new std::vector<int>);
  compute_persistent_features(view, persistent_indices);

  // Extract persistent indices from view
  pcl::ExtractIndices<Point_N> extract_filter;
  extract_filter.setInputCloud(view);
  extract_filter.setIndices(persistent_indices);
  extract_filter.filter(*persistent_cloud);
  extract_filter.setNegative(true);
  extract_filter.filter(*non_persistent_cloud);
}
