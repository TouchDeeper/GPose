#ifndef SIMILAR_OBJECT_RECOGNITION_H_
#define SIMILAR_OBJECT_RECOGNITION_H_

#include <pcl/io/pcd_io.h>
#include <pcl/features/esf.h>
#include "access_model_data.h"


class Similar_Object_Recognition
{	
	private:
	
		typedef pcl::ESFSignature640 FeatureT;
		typedef pcl::PointCloud<FeatureT> FeatureCloudT;
		
		float similarity_threshold_;
		
		/*
		A structure for storing data associated to a similar model
		*/
		struct sim_mod
		{
			std::string name;
			std::vector<pcl::Correspondence> similar_views;
			float nbr_of_similar_views;
	
			sim_mod() : nbr_of_similar_views (0) {}
		};

		/*
		A structure for storing data associated to each model
		*/
		struct model_data
		{
			std::string name;
			FeatureCloudT::Ptr feature_cloud;
			std::vector<sim_mod> similar_models;
		};

		/*
		A structure for sorting vectors with pcl::Correspondences
		*/
		struct less_than_key
		{
			inline bool operator() (const pcl::Correspondence corr1, const pcl::Correspondence corr2)
			{
				return (corr1.distance < corr2.distance);
			}
		};
		
		void
		load_target_model_data (std::string source_name, std::vector<model_data> &m_targets);
		
		float
		l1_norm (FeatureT f1, FeatureT f2);
		
		
		void
		search_for_similar_views (model_data &m_source, std::vector<model_data> &m_targets);
		
		void
		save_similar_models_data (model_data m_source, std::vector<model_data> m_targets);
		
		void
		warning_log (model_data model);
		
	public:
	
		/** 
		  Empty constructor. 
	  	*/
		Similar_Object_Recognition (void);
	
		void
		add_model (std::string source_name, FeatureCloudT::Ptr source_features);
	
};

#endif
