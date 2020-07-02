#include <parameters.h>
#include "view_graph/view_graph.h"

/**
  Returns the path to Data in /object_identification_localization
*/
boost::filesystem::path 
View_Graph::path_to_model_in_model_data (std::string model)
{
	// Current path
//	boost::filesystem::path p(boost::filesystem::current_path());
	boost::filesystem::path p(cr_->model_data_path);

	// Search parent directories for object_identification_localization
//	while (true)
//	{
//		p = p.parent_path();
//
//		if (p.filename() == "pose_estimation")
//		{
//			break;
//		}
//	}
//
//	// Add path to directory named "data"
//	p /= "data";
//
//	// Check if such path exists
//	if (!boost::filesystem::exists(p))
//	{
//		std::cerr << "\nERROR: No directory found named data!\n" << std::endl;
//		std::exit (EXIT_FAILURE);
//	}
//
//	// Add path to model_data
//	p /= "model_data";
	
	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		std::cerr << "\nERROR: No directory found named model_data!\n" << std::endl;
		std::exit (EXIT_FAILURE);
	}
	
	// Add path to model
	p /= model;
	
	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		// Path does not exist, create directory for model
		if (!boost::filesystem::create_directory(p))
		{
			std::cerr << "\nERROR: Could not create directory for " << model << " in model_data!\n" << std::endl;
			std::exit (EXIT_FAILURE);
		}
	}
	
	return p;
}

/**
  Returns the camera viewpoint for a given view
  @param index The view-index
*/	
pcl::PointXYZ
View_Graph::get_viewpoint (int index)
{
	return graph[index].viewpoint_;
}

/**
  Returns the rotation matrix for a given view 
  @param index The view-index
*/	
Eigen::Matrix3d
View_Graph::get_rotation (int index)
{
	return graph[index].rotation_;
}

/**
  Returns the neighbors for a given view 
  @param index The view-index
*/	
std::vector<int>
View_Graph::get_neighbors (int index)
{
	return graph[index].neighbors_;
}

/**
  Returns the neighbors for a given view 
  @param index The view-index
*/	
int 
View_Graph::get_size (void)
{
	return graph.size();
}

/**
  Returns the name of the model
*/	
std::string
View_Graph::get_model_name (void)
{
	return model_name_;
}

/**
  Sets the name of the model
  @param model_name The name of the model
*/	
void
View_Graph::set_model_name (std::string model_name)
{
	model_name_ = model_name;
}

/**
  Computes and stores all the valid nodes in the graph. Valid nodes are nodes that are above the plane in the scene and doesn't share similar z-axis as current and previous camera positions.
  @param plane The plane in the scene
  @param previous_positions The previous camera positions
  @param angle_thr The angle threshold to determine if a node is invalid due to having similar z-axis as previous camera locations
*/
void
View_Graph::find_valid_nodes (pcl::ModelCoefficients plane, std::vector<Eigen::Matrix<float,4,4,Eigen::DontAlign> > previous_positions, PointT model_center, float angle_thr)
{
	// Find valid nodes above the plane
	std::vector<bool> temp (graph.size(), false);
	for (int i = 0; i < graph.size(); i++)
	{
		PointT p = graph[i].viewpoint_;
		if ((plane.values[0] * p.x + plane.values[1] * p.y + plane.values[2] * p.z + plane.values[3]) > 0.1)
		{
			temp[i] = true;
		}
	}

	// Find invalid nodes from current and previous camera positions
	float pi_const = atan (1.0) * 4; // Pi
	for (int i = 0; i < (previous_positions.size() + 1); i++)
	{
		Eigen::Vector3f cam_axis;
		if (i == 0)
		{
			cam_axis (0) = model_center.x;
			cam_axis (1) = model_center.y;
			cam_axis (2) = model_center.z;
		}
		else
		{
			Eigen::Vector3f prev_cam_center = previous_positions[i-1].block <3,1> (0,3);
			cam_axis (0) = model_center.x - prev_cam_center (0);
			cam_axis (1) = model_center.y - prev_cam_center (1);
			cam_axis (2) = model_center.z - prev_cam_center (2);
		}
		
		cam_axis.normalize ();
		for (int j = 0; j < temp.size(); j++)
		{
			Eigen::Vector3f z_axis_node = graph[j].rotation_.cast <float> ().block <3,1> (0,2);
			z_axis_node.normalize ();
			float angle = acos (cam_axis.dot (z_axis_node)) * (180 / pi_const);
			if (std::abs (angle) < angle_thr)
			{
				temp[j] = false;
			}
		}
	}
	
	valid_nodes_ = temp;
}

/**
  Determines if two views are neighbors
  @param view_index1 The first view 
  @param view_index2 The second view 
  @return True if the views are neighbors and false otherwise
*/
bool
View_Graph::is_neighbor (int view_index1, int view_index2)
{
	for (int i = 0; i < graph[view_index1].neighbors_.size(); i++)
	{
		if (graph[view_index1].neighbors_[i] == view_index2)
		{
			return true;
		}
	}
	return false;
}

/**
  Set manualy if a view-node is valid or not
  @param index The view index
  @param value True if the node is valid and false otherwise
*/
void
View_Graph::set_valid_node (int index, bool value)
{
	valid_nodes_[index] = value;
}

/**
  Returns a vector containing all the valid nodes
  @return The vector containing all the valid nodes
*/
std::vector<bool>
View_Graph::get_valid_nodes (void)
{
	return valid_nodes_;
}

/**
  Returns the visited nodes during the graph search
  @return Vector containing a true value if the node was visited during the graph search.
*/
std::vector<bool>
View_Graph::get_visited_nodes (void)
{
	return visited_nodes_;
}

/**
  Stores all viewpoints as a pointcloud
  @param viewpoints[out] The pointcloud containing all viewpoints in the graph
*/
void 
View_Graph::get_all_viewpoints (PointCloudT::Ptr viewpoints)
{
	for (int i = 0; i < get_size(); i++)
	{
		viewpoints->push_back (graph[i].viewpoint_);
	}
}
	  	
/**
  Plots the view-graph 
  @param visu The visualize-object
  @param radiu The radius of the spheres (nodes)
  @param viewport Id of the viewport
  @param show_rotations Set if the nodes should show rotations
*/	
void 
View_Graph::add_graph_to_viewer (	pcl::visualization::PCLVisualizer &visu, 
									double radius,
									int viewport,
									bool show_rotations, bool show_number )
{
    sphere_radius = radius;
	// Add nodes
	for (int i = 0; i < graph.size(); i++)
	{
		std::stringstream ss;
		ss << "Sphere" << i;
		visu.addSphere (graph[i].viewpoint_, radius, ss.str (), viewport);
		visu.updateSphere(graph[i].viewpoint_,radius,graph[i].rgb[0],graph[i].rgb[1],graph[i].rgb[2],ss.str ());
        if(show_number){
            std::stringstream ssNumber;
            ssNumber << "number "<< i;
            pcl::PointXYZ numberOffset(0.02,0.02,0);
            pcl::PointXYZ numberCenter(numberOffset.x + graph[i].viewpoint_.x,numberOffset.y + graph[i].viewpoint_.y,numberOffset.z + graph[i].viewpoint_.z);
            visu.addText3D(std::to_string(i),numberCenter,radius,VIEWSTYLE.model_rgb[0],VIEWSTYLE.model_rgb[1],VIEWSTYLE.model_rgb[2],ssNumber.str (),viewport);
        }

		if (show_rotations)
		{
			Eigen::Affine3f a;
			Eigen::Matrix4f t;
			t.block <3,3> (0,0) = graph[i].rotation_.cast <float> ();
			t (0,3) = graph[i].viewpoint_.x;
			t (1,3) = graph[i].viewpoint_.y;
			t (2,3) = graph[i].viewpoint_.z;
			a.matrix () = t;
			visu.addCoordinateSystem (0.05, a, ss.str (), viewport);
		}
	}
	
	// Add links
	for (int i = 0; i < graph.size(); i++)
	{
		for (int j = 0; j < graph[i].neighbors_.size(); j++)
		{
			std::stringstream ss;
			ss << "Link" << i << j;
			visu.addLine (graph[i].viewpoint_, graph[graph[i].neighbors_[j]].viewpoint_, VIEWSTYLE.line_rgb[0], VIEWSTYLE.line_rgb[1], VIEWSTYLE.line_rgb[2], ss.str (), viewport);
//            visu.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, point_size/2, ss.str (),viewport);

//			visu.addLine (graph[i].viewpoint_, graph[graph[i].neighbors_[j]].viewpoint_, 0, 0, 0, ss.str (), viewport);
		}
	}
}
void View_Graph::reset_node_sphere(pcl::visualization::PCLVisualizer &visu){
    // Add nodes
    for (int i = 0; i < graph.size(); i++)
    {
        std::stringstream ss;
        ss << "Sphere" << i;
        visu.updateSphere(graph[i].viewpoint_, sphere_radius ,graph[i].rgb[0],graph[i].rgb[1],graph[i].rgb[2],ss.str ());
    }
}

/**
  Plots the view-graph 
  @param visu The visualize-object
  @param radiu The radius of the spheres (nodes)
  @param r_vec Vector containing color information for the spheres (red)
  @param g_vec Vector containing color information for the spheres (green)
  @param b_vec Vector containing color information for the spheres (blue)
  @param viewport Id of the viewport
  @param show_rotations Set if the nodes should show rotations
*/	
void 
View_Graph::add_graph_to_viewer (	pcl::visualization::PCLVisualizer &visu, 
									double radius, 
									std::vector<float> r_vec,
									std::vector<float> g_vec,
									std::vector<float> b_vec,
									int viewport,
									bool show_rotations )
{
	// Add nodes
	for (int i = 0; i < graph.size(); i++)
	{
		std::stringstream ss;
		ss << "Sphere" << i << "viewport" << viewport;
		visu.addSphere (graph[i].viewpoint_, radius, r_vec[i], g_vec[i], b_vec[i], ss.str (), viewport);
		
		if (show_rotations)
		{
			Eigen::Affine3f a;
			Eigen::Matrix4f t;
			t.block <3,3> (0,0) = graph[i].rotation_.cast <float> ();
			t (0,3) = graph[i].viewpoint_.x;
			t (1,3) = graph[i].viewpoint_.y;
			t (2,3) = graph[i].viewpoint_.z;
			a.matrix () = t;
			visu.addCoordinateSystem (0.05, a, ss.str (), viewport);
		}
	}
	
	// Add links
	for (int i = 0; i < graph.size(); i++)
	{
		for (int j = 0; j < graph[i].neighbors_.size(); j++)
		{
			std::stringstream ss;
			ss << "Link" << i << j << "viewport" << viewport;
			visu.addLine (graph[i].viewpoint_, graph[graph[i].neighbors_[j]].viewpoint_, 1.0, 1.0, 1.0, ss.str (), viewport);
		}
	}
}

/**
  Add node to the view-graph 
  @param viewpoint The viewpoint of the virtual camera
  @param rotation The rotation of the virtual camera
  @param neighbors The nearest neighbors of the node
*/	
void 
View_Graph::add_node (PointT viewpoint, Eigen::Matrix3d rotation, std::vector<int> neighbors)
{
	node n;
	n.viewpoint_ = viewpoint;
	n.rotation_ = rotation;
	n.neighbors_ = neighbors;
	n.view_index = graph.size();
	n.rgb[0] = 0;
//	n.rgb[1] = 255;
	n.rgb[1] = 0;
	n.rgb[2] = 0;

	graph.push_back (n);
}

/**
  Saves the generated view-graph to CSV-file.    
*/
void 
View_Graph::save_graph ()
{
	// Check if model_name_ has been set
	if (model_name_.empty ())
	{
		std::cerr << "\nERROR: Model for graph must have a name!\n" << std::endl;
		std::exit (EXIT_FAILURE);
	}
	
	// Add path to model
	boost::filesystem::path p = ac_->path_to_model_in_model_data (model_name_);
	
	std::ofstream graph_file;
	p /= "graph.csv";
	graph_file.open (p.string().c_str());
	if(!graph_file)
    {
        std::cerr<<"can't open "<<p.string()<<std::endl;
        exit(-1);
    }

	std::vector<std::string> string_vec;
	for (int i = 0; i < get_size(); i++)
	{
		PointT viewpoint = get_viewpoint (i);
		Eigen::Matrix3d rotation = get_rotation (i);
		std::vector<int> neighbors = get_neighbors (i);
		
		std::ostringstream ss;
		ss << i << ",";
		ss << viewpoint.x << ",";
		ss << viewpoint.y << ",";
		ss << viewpoint.z << ",";
		ss << rotation(0,0) << ",";
		ss << rotation(0,1) << ",";
		ss << rotation(0,2) << ",";
		ss << rotation(1,0) << ",";
		ss << rotation(1,1) << ",";
		ss << rotation(1,2) << ",";
		ss << rotation(2,0) << ",";
		ss << rotation(2,1) << ",";
		ss << rotation(2,2) << ",";
		for (int j = 0; j < neighbors.size(); j++)
		{
			ss << neighbors[j] << ",";
		}

		string_vec.push_back (ss.str ());
	}
	
	for (int i = 0; i < string_vec.size(); i++)
	{
		graph_file << string_vec[i] << "\n";
	}
	
	graph_file.close();
}
void
View_Graph::generate_relative_matrix(){
    relative_matrix_ = Eigen::MatrixXi::Zero(get_size(), get_size());
    for (int i = 0; i < get_size(); ++i) {
        for (int j = i+1; j < get_size(); ++j) {
            pcl::PointXYZ v1 = get_viewpoint(i);
            pcl::PointXYZ v2 = get_viewpoint(j);
            Eigen::Vector3d v1_e(v1.x, v1.y, v1.z);
            Eigen::Vector3d v2_e(v2.x, v2.y, v2.z);
            Eigen::Vector3d v1_v2 = v2_e - v1_e;
            double v1_norm = v1_e.norm();
            double v2_norm = v2_e.norm();
            double v1_v2_norm = v1_v2.norm();
            double angle = acos((v1_norm * v1_norm + v2_norm * v2_norm - v1_v2_norm * v1_v2_norm)/(2*v1_norm*v2_norm));
            if(angle > M_PI/3)
                relative_matrix_(i,j) = 1;
        }
    }
    // Add path to model
    boost::filesystem::path p = ac_->path_to_model_in_model_data (model_name_);
    std::ofstream relative_matrix_file;
    p /= "relative_matrix.txt";
    relative_matrix_file.open(p.string().c_str());
    if(!relative_matrix_file)
    {
        std::cerr<<"can't open "<<p.string()<<std::endl;
        exit(-1);
    }
    relative_matrix_file << relative_matrix_<<std::endl;

}
/**
  Loads the graph of a model (see structure node for details)  
  @param model The name of the model 
*/
void 
View_Graph::load_graph (std::string model)
{
	set_model_name (model);
	
	// Add path to model in Model_data
	boost::filesystem::path p = ac_->path_to_model_in_model_data (model);
	
	std::stringstream ss;
	ss << p.string () << "/graph.csv";
	std::ifstream graph_file (ss.str().c_str()); 
	
	std::string line;
	while (std::getline (graph_file,line))
	{ 
		std::stringstream buffer(line);
		std::string value;
		std::vector<std::string> data;

		while(std::getline(buffer, value, ','))
		{
		   data.push_back(value);
		}
		
		PointT viewpoint;
		Eigen::Matrix3d rotation;
		std::vector<int> neighbors;
		
		viewpoint.x = std::strtof (data[1].c_str(), NULL);
		viewpoint.y = std::strtof (data[2].c_str(), NULL);
		viewpoint.z = std::strtof (data[3].c_str(), NULL);
		rotation(0,0) = std::strtod (data[4].c_str(), NULL);
		rotation(0,1) = std::strtod (data[5].c_str(), NULL);
		rotation(0,2) = std::strtod (data[6].c_str(), NULL);
		rotation(1,0) = std::strtod (data[7].c_str(), NULL);
		rotation(1,1) = std::strtod (data[8].c_str(), NULL);
		rotation(1,2) = std::strtod (data[9].c_str(), NULL);
		rotation(2,0) = std::strtod (data[10].c_str(), NULL);
		rotation(2,1) = std::strtod (data[11].c_str(), NULL);
		rotation(2,2) = std::strtod (data[12].c_str(), NULL);
		
		for (int j = 13; j < data.size(); j++)
		{
			neighbors.push_back(std::strtol (data[j].c_str(), NULL, 10));
		}	
		
		add_node (viewpoint, rotation, neighbors);
	}

	//load relative_matrix_
    boost::filesystem::path relative_matrix_p;
	relative_matrix_p = p / "/relative_matrix.txt";
//    std::stringstream ss_relative_matrix_path;
//    ss_relative_matrix_path << p.string() << "/relative_matrix.txt";
    td::readMatrix(relative_matrix_p.string(),relative_matrix_);
//    std::cout<<relative_matrix_<<std::endl;


}

/**
  Adds utility to all nodes  
  @param utilities Vector containing the utilities 
*/
void 
View_Graph::add_utilities (std::vector<float> utilities)
{
	if (utilities.size() != graph.size())
	{
		std::cerr << "\nERROR: The size of the utility vector in View_Graph::add_utilities is not equal to the graph size!\n" << std::endl;
		std::exit (EXIT_FAILURE);
	}
	
	utilities_ = utilities;
	
	for (int i = 0; i < graph.size(); i++)
	{
		graph[i].utility_ = utilities[i];
	}
}

/**
  Searches for an optimal view given the valid nodes in the view-graph. Returns the optimal view and an optimal trajectory leading to the optimal view  
  @param initial_view_node The current view in the graph
  @return Vector containing the optimal view and a trajectory of views to visit in order to get to the optimal view
*/
std::vector<int>
View_Graph::search_for_better_view (int initial_view_node)
{
	//
	// Find the valid node with the highest utility
	//
	
	float best_utility = 0.0f;
	int better_view_node;
	for (int i = 0; i < graph.size(); i++)
	{
		if (valid_nodes_[i] && utilities_[i] > best_utility && i != initial_view_node)
		{
			best_utility = utilities_[i];
			better_view_node = i;
		}
	}
	
	//
	// Search through the graph to find the most optimal path from current_view_node to better_view_node using Dijkstra's search algorithm 
	//
	
	std::vector<int> q;
	for (int i = 0; i < graph.size(); i++)
	{
		if (valid_nodes_[i])
		{
			q.push_back (i);
			graph[i].cost = std::numeric_limits<float>::max();
			graph[i].previous = -1;
		}
	}
	
	// Find the largest euclidean distance between the nodes in q
	float largest_dist = 0.0;
	float smallest_dist = std::numeric_limits<float>::max();
	for (int i = 0; i < q.size(); i++)
	{
		for (int j = 0; j < q.size(); j++)
		{
			if (i == j)
			{
				continue;
			}
			
			PointT p1 = graph[q[i]].viewpoint_;
			PointT p2 = graph[q[j]].viewpoint_;
			float dist = sqrt (pow (p1.x - p2.x, 2) + pow (p1.y - p2.y, 2) + pow (p1.z - p2.z, 2));
			largest_dist = std::max (largest_dist, dist);
			smallest_dist = std::min (smallest_dist, dist);
		}
	}
	
	graph[initial_view_node].cost = 0.0;
	std::vector<bool> visited_nodes (graph.size(), false);
	while (!q.empty())
	{
		// Find the node with the smallest cost in q
		float smallest_cost = std::numeric_limits<float>::max();
		int index_in_q;
		int current_node;
		for (int i = 0; i < q.size(); i++)
		{
			if (graph[q[i]].cost < smallest_cost)
			{
				smallest_cost = graph[q[i]].cost;
				index_in_q = i;
				current_node = q[i];
			}
		}
		
		// Remove the current node in q and add it to the list of visited nodes
		visited_nodes[current_node] = true;
		q.erase(q.begin() + index_in_q);
 		
 		// Assign new costs to all valid nodes from the current node
 		for (int i = 0; i < q.size(); i++)
 		{
 			int next_node = q[i];
			PointT p1 = graph[current_node].viewpoint_;
			PointT p2 = graph[next_node].viewpoint_;
			float utility_cost = (1 - graph[next_node].utility_);
			float distance_cost = sqrt (pow (p1.x - p2.x, 2) + pow (p1.y - p2.y, 2) + pow (p1.z - p2.z, 2)) / largest_dist; //L1
			float cost_to_next_node = (0.008*exp (utility_cost * 7)) + (exp (distance_cost * 4) - 1); // Non-linear cost
			if (graph[next_node].cost > graph[current_node].cost + cost_to_next_node)
			{
				graph[next_node].cost = graph[current_node].cost + cost_to_next_node;
				graph[next_node].previous = current_node;
			} 
 		}	
	}
	
	visited_nodes_ = visited_nodes;
	
	// Extract path from initial_view_node to better_view_node
	std::vector<int> trajectory;
	int index = better_view_node;
	while (index != initial_view_node)
	{
		trajectory.push_back (index);
		index = graph[index].previous;
    } 
    trajectory.push_back (initial_view_node);
    
    return trajectory;
}
	
/**
  Adds a transformation to the view-graph
  @param transform The transformation
*/	
void
View_Graph::add_transformation (Eigen::Matrix4f transform)
{
	// Transform all viewpoints
	PointCloudT::Ptr viewpoint_cloud (new PointCloudT);
	get_all_viewpoints (viewpoint_cloud);
	pcl::transformPointCloud (*viewpoint_cloud, *viewpoint_cloud, transform);
	
	// Assign new viewpoints and rotations
	for (int i = 0; i < get_size(); i++)
	{	
		graph[i].viewpoint_ = viewpoint_cloud->points[i];
		graph[i].rotation_ = transform.block <3,3> (0,0).cast <double> () * graph[i].rotation_;
	}
}	

/**
  Finds the view that best aligns with the input camera axis
  @param cam_axis The camera axis
  @return The node with best alignment
*/
int
View_Graph::find_aligned_view (Eigen::Vector3f cam_axis)
{
	int best_alignment;
	float min_angle = 180.0;
	float pi_const = atan (1.0) * 4; // Pi
	for (int i = 0; i < graph.size(); i++)
	{
		Eigen::Vector3f node_axis = graph[i].rotation_.block <3,1> (0,2).cast <float> ();
		cam_axis.normalize ();
		node_axis.normalize ();
		float angle = acos (cam_axis.dot (node_axis)) * (180 / pi_const);
		if (angle < min_angle)
		{
			best_alignment = i;
			min_angle = angle;
		}
	}
	
	return best_alignment;
}

		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
