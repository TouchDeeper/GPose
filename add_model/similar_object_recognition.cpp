
#include "add_model/similar_object_recognition.h"

/** 
  Constructor. 
*/
Similar_Object_Recognition::Similar_Object_Recognition (void)
{
	similarity_threshold_ = 0.2;
}

void
Similar_Object_Recognition::load_target_model_data (std::string source_name, std::vector<model_data> &m_targets)
{
	// Get all models
	Access_Model_Data* access = Access_Model_Data::GetInstance();
	std::vector<std::string> model_names = access->get_model_names ();
	
	for (int i = 0; i < model_names.size(); i++)
	{	
		if (model_names[i] == source_name)
		{
			continue;
		}
		
		model_data m;
		m.name = model_names[i];
		
		// Load global features
		FeatureCloudT::Ptr features (new FeatureCloudT);
		access->load_global_features (model_names[i], features);
		m.feature_cloud = features;
		
		m_targets.push_back (m);
	}
}

float 
Similar_Object_Recognition::l1_norm (FeatureT f1, FeatureT f2)
{
	float sum = 0.0f;
	for (int i = 0; i < 640; i++)
	{
		sum += std::abs (f1.histogram[i] - f2.histogram[i]);
	}
	return sum;
}

void 
Similar_Object_Recognition::search_for_similar_views (model_data &m_source, std::vector<model_data> &m_targets)
{	
	//
	// Match m_source to m_targets
	//
	
	for (int t_index = 0; t_index < m_targets.size(); t_index++)
	{
		sim_mod sm;
		sm.name = m_targets[t_index].name;
		int similar_view_count = 0;
		for (int i = 0; i < m_source.feature_cloud->points.size(); i++)
		{
			// L1 norm
			std::vector<pcl::Correspondence> model_model_corrs;
			for (int j = 0; j < m_targets[t_index].feature_cloud->points.size(); j++)
			{
				float diff = l1_norm (m_source.feature_cloud->points[i], m_targets[t_index].feature_cloud->points[j]);
				pcl::Correspondence corr (i, j, diff); 
				model_model_corrs.push_back (corr); 
			}
	
			// Sort model_model_corrs in ascending order with respect to the feature distance
			std::sort(model_model_corrs.begin(), model_model_corrs.end(), less_than_key());

			// Add best match to similar views
			sm.similar_views.push_back (model_model_corrs[0]);
		}
		
		// Compute number of similar views
		for (int i = 0; i < sm.similar_views.size(); i++)
		{
			if (sm.similar_views[i].distance < similarity_threshold_)
			{
				similar_view_count++;
			}
		}
		sm.nbr_of_similar_views = similar_view_count / (float)sm.similar_views.size();
	
		m_source.similar_models.push_back (sm);
	}	
	
	//
	// Match m_targets to m_source
	//
	
	for (int t_index = 0; t_index < m_targets.size(); t_index++)
	{
		sim_mod sm;
		sm.name = m_source.name;
		int similar_view_count = 0;
		for (int i = 0; i < m_targets[t_index].feature_cloud->points.size(); i++)
		{
			// L1 norm
			std::vector<pcl::Correspondence> model_model_corrs;
			for (int j = 0; j < m_source.feature_cloud->points.size(); j++)
			{
				float diff = l1_norm (m_targets[t_index].feature_cloud->points[i], m_source.feature_cloud->points[j]);
				pcl::Correspondence corr (i, j, diff); 
				model_model_corrs.push_back (corr); 
			}
	
			// Sort model_model_corrs in ascending order with respect to the feature distance
			std::sort(model_model_corrs.begin(), model_model_corrs.end(), less_than_key());

			// Add best match to similar views
			sm.similar_views.push_back (model_model_corrs[0]);
		}
		
		// Compute number of similar views
		for (int i = 0; i < sm.similar_views.size(); i++)
		{
			if (sm.similar_views[i].distance < similarity_threshold_)
			{
				similar_view_count++;
			}
		}
		sm.nbr_of_similar_views = similar_view_count / (float)sm.similar_views.size();
	
		m_targets[t_index].similar_models.push_back (sm);
	}
}

void 
Similar_Object_Recognition::save_similar_models_data (model_data m_source, std::vector<model_data> m_targets)
{
	Access_Model_Data* access = Access_Model_Data::GetInstance();
	
	//
	// Save similar object data in m_source
	//
	
	for (int i = 0; i < m_source.similar_models.size(); i++)
	{
		std::vector<float> sim_views_vec;
		for (int j = 0; j < m_source.similar_models[i].similar_views.size(); j++)
		{
			sim_views_vec.push_back (m_source.similar_models[i].similar_views[j].distance);
		}
		
		access->save_similar_model_data (m_source.name, m_source.similar_models[i].name, sim_views_vec);
	}
	
	//
	// Save similar object data in m_targets
	//
	
	for (int i = 0; i < m_targets.size(); i++)
	{
		std::vector<float> sim_views_vec;
		for (int j = 0; j < m_targets[i].similar_models[0].similar_views.size(); j++)
		{
			sim_views_vec.push_back (m_targets[i].similar_models[0].similar_views[j].distance);
		}
		
		access->save_similar_model_data (m_targets[i].name, m_source.name, sim_views_vec);
	}	
}

void 
Similar_Object_Recognition::warning_log (model_data model)
{
	bool warn = false;
	for (int i = 0; i < model.similar_models.size(); i++)
	{
		// Add warning if the number of similar views is larger than 50%
		if (model.similar_models[i].nbr_of_similar_views >= 0.5)
		{
			warn = true;
			break;
		}
	}
	
	if (warn)
	{
		std::cout << "\nWARNING! " << model.name << " has many similar views with:\n" << std::endl;
		for (int i = 0; i < model.similar_models.size(); i++)
		{
			if (model.similar_models[i].nbr_of_similar_views >= 0.5)
			{
				std::cout << "\t" << model.similar_models[i].name << ": " << model.similar_models[i].nbr_of_similar_views * 100 << "% similar views!" << std::endl;
			}
		}
	}
	
	std::cout << "\n" << std::endl;
}

void
Similar_Object_Recognition::add_model (std::string source_name, FeatureCloudT::Ptr source_features)
{
	model_data m_source;
	m_source.name = source_name;
	m_source.feature_cloud = source_features;
	
	// Load target models
	std::vector<model_data> m_targets;
	load_target_model_data (source_name, m_targets);
	
	// Check if no target models were found
	if (m_targets.size() == 0)
	{
		std::cout << "\nNo target models found in Model_data\n" << std::endl;
		return;
	} 
	
	// Search for similar views between source and target models
	search_for_similar_views (m_source, m_targets);
	
	// Save data for similar views
	save_similar_models_data (m_source, m_targets);
	
	// Warning log
	warning_log (m_source);
}



















