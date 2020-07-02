#ifndef VIEW_GRAPH_H_
#define VIEW_GRAPH_H_

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <queue>
#include <TdLibrary/td_eigen/eigen_tools.h>
#include "config_reader.h"
#include "access_model_data.h"


class View_Graph
{
public:
    typedef boost::shared_ptr< ::View_Graph> Ptr;
    typedef boost::shared_ptr< ::View_Graph const> ConstPtr;
	private:
	
		typedef pcl::PointXYZ PointT;
		typedef pcl::PointCloud<PointT> PointCloudT;
				
		/**
		  A structure for the node data 
		*/
		struct node
		{
			PointT viewpoint_;
			Eigen::Matrix3d rotation_;
			std::vector<int> neighbors_;
			float utility_;
			int view_index;
			float cost;
			int previous;
			int rgb[3];
			bool operator < (const node &n) const
			{
				return (cost < n.cost);
			}
		};	
		
		std::vector<node> graph;
		std::string model_name_;
		std::vector<float> utilities_;
		std::vector<bool> valid_nodes_;
		std::vector<bool> visited_nodes_; // Array for keeping track of visited nodes
		Eigen::MatrixXi relative_matrix_;
		double sphere_radius; //node sphere radius in viewer
		Config_Reader* cr_;
		Access_Model_Data* ac_;

		/**
		  Returns the path to Data in /masters_thesis
		*/
		boost::filesystem::path 
		path_to_model_in_model_data (std::string model);	

	public:
	
		/** 
		  Empty constructor. 
		*/
	  	View_Graph () : graph () {
	  	    cr_ = Config_Reader::GetInstance();
	  	    ac_ = Access_Model_Data::GetInstance();
	  	}
		  
  		/**
		  Returns the camera viewpoint for a given view
		  @param index The view-index
		*/	
	  	PointT
	  	get_viewpoint (int index);
	  	
  		/**
		  Returns the rotation matrix for a given view 
		  @param index The view-index
		*/	
	  	Eigen::Matrix3d
	  	get_rotation (int index);
	  	
  		/**
		  Returns the neighbors for a given view 
		  @param index The view-index
		*/	
	  	std::vector<int>
	  	get_neighbors (int index);
	  	
  		/**
		  Returns the neighbors for a given view 
		  @param index The view-index
		*/	
	  	int 
	  	get_size (void);
	  	
  		/**
		  Returns the name of the model
		*/	
	  	std::string
	  	get_model_name (void);
	  	
  		/**
		  Sets the name of the model
		  @param model_name The name of the model
		*/	
	  	void
	  	set_model_name (std::string model_name);
	  	
		/**
		  Computes and stores all the valid nodes in the graph. Valid nodes are nodes that are above the plane in the scene and doesn't share similar z-axis as current and previous camera positions.
		  @param plane The plane in the scene
		  @param previous_positions The previous camera positions
		  @param angle_thr The angle threshold to determine if a node is invalid due to having similar z-axis as previous camera locations
		*/
		void
		find_valid_nodes (pcl::ModelCoefficients plane, std::vector<Eigen::Matrix<float,4,4,Eigen::DontAlign> > previous_positions, PointT model_center, float angle_thr);
		
		/**
		  Determines if two views are neighbors
		  @param view_index1 The first view 
		  @param view_index2 The second view 
		  @return True if the views are neighbors and false otherwise
		*/
		bool
		is_neighbor (int view_index1, int view_index2);

		/**
		  Set manualy if a view-node is valid or not
		  @param index The view index
		  @param value True if the node is valid and false otherwise
		*/
		void
		set_valid_node (int index, bool value);
	  	
	  	/**
	  	  Returns a vector containing all the valid nodes
	  	  @return The vector containing all the valid nodes
	  	*/
	  	std::vector<bool>
	  	get_valid_nodes (void);
	  	
  		/**
		  Returns the visited nodes during the graph search
		  @return Vector containing a true value if the node was visited during the graph search.
		*/
	  	std::vector<bool>
	  	get_visited_nodes (void);
	  	
	  	/**
		  Stores all viewpoints as a pointcloud
		  @param viewpoints[out] The pointcloud containing all viewpoints in the graph
		*/
		void 
		get_all_viewpoints (PointCloudT::Ptr viewpoints);
		    
		/**
		  Plots the view-graph 
		  @param visu The visualize-object
		  @param radiu The radius of the spheres (nodes)
		  @param viewport Id of the viewport
		  @param show_rotations Set if the nodes should show rotations
		  @param show_number Set if the nodes should show index
		*/	
		void 
		add_graph_to_viewer (	pcl::visualization::PCLVisualizer &visu, 
								double radius,
								int viewport,
								bool show_rotations, bool show_number );

        void reset_node_sphere(pcl::visualization::PCLVisualizer &visu);

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
		add_graph_to_viewer (	pcl::visualization::PCLVisualizer &visu, 
								double radius, 
								std::vector<float> r_vec,
								std::vector<float> g_vec,
								std::vector<float> b_vec,
								int viewport,
								bool show_rotations );
				
		/**
		  Adds a node to the view-graph 
		  @param viewpoint The viewpoint of the virtual camera
		  @param rotation The rotation of the virtual camera
		  @param neighbors The nearest neighbors of the node
		*/	
		void
		add_node (PointT viewpoint, Eigen::Matrix3d rotation, std::vector<int> neighbors);
		
		/**
		  Saves the view-graph to CSV-file.    
		*/
		void 
		save_graph ();
		/**
		 * generate relative matrix and save it
		 */
        void
        generate_relative_matrix();
		/**
		  Loads the view-graph of a model (see structure node for details), and relative matrix
		  @param model The name of the model 
		*/
		void 
		load_graph (std::string model);
		
		/**
		  Adds utility to all nodes  
		  @param utilities view_graph_.Vector containing the utilities
		*/
		void 
		add_utilities (std::vector<float> utilities);
		
		/**
		  Searches for an optimal view given the valid nodes in the view-graph. Returns the optimal view and an optimal trajectory leading to the optimal view  
		  @param initial_view_node The current view in the graph
		  @return Vector containing the optimal view and a trajectory of views to visit in order to get to the optimal view
		*/
		std::vector<int>
		search_for_better_view (int initial_view_node);
		
		/**
		  Adds a transformation to the view-graph
		  @param transform The transformation
		*/	
		void
		add_transformation (Eigen::Matrix4f transform);
		
		/**
		  Finds the view that best aligns with the input camera axis
		  @param cam_axis The camera axis
		  @return The node with best alignment
		*/
		int
		find_aligned_view (Eigen::Vector3f cam_axis);

		int get_relative_matrix_value(int row, int col){
		    if(row > col)
                return relative_matrix_(col, row);
            else
                return relative_matrix_(row, col);
		}
};

#endif
