//
// Created by wang on 20-6-2.
//
#include "config_reader.h"
#include "parameters.h"
#include "view_graph/view_graph.h"

int main(int argc, char** argv){
    ReadParameters("../../config.ini");
    Config_Reader* cr = Config_Reader::GetInstance();
    for (int i = 1; ; ++i) {
        boost::filesystem::path p(cr->model_data_path);
        std::string model_name;
        if(i<=9)
            model_name = "obj_00000" + std::to_string(i);
        else
            model_name = "obj_0000"  + std::to_string(i);
        p /= model_name;
        if(!boost::filesystem::exists(p))
        {
            std::cout<<"ERROR: No directory found named "<<model_name<<std::endl;
            break;
        }
        std::cout<<"view "<<model_name<<std::endl;
        View_Graph view_grapher;
        pcl::visualization::PCLVisualizer viewer;
        viewer.setBackgroundColor(VIEWSTYLE.background_color[0],VIEWSTYLE.background_color[1],VIEWSTYLE.background_color[2]);
        view_grapher.load_graph(model_name);
        view_grapher.add_graph_to_viewer(viewer, 0.005, 0, false, false);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // Add path to model in Model_data

        std::stringstream ss;
        ss << p.string () << "/complete_model.pcd";
        if ( pcl::io::loadPCDFile <pcl::PointXYZ> (ss.str (), *cloud) < 0 )
        {
            std::stringstream ss;
            ss << "ERROR: Could not load complete_model for " << model_name << " in /model_data!\n\n";
            pcl::console::print_error(ss.str().c_str());
            std::exit (EXIT_FAILURE);
        }
        // Add complete model to viewer
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(cloud,VIEWSTYLE.model_rgb[0],VIEWSTYLE.model_rgb[1],VIEWSTYLE.model_rgb[2]);
        viewer.addPointCloud<pcl::PointXYZ> (cloud,rgb);
        viewer.spin();
    }


}
