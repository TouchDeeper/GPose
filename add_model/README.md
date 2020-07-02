# add_model
Adds a CAD model to the model base. This is needed for the system to be able to recognize and estimate 6DOF poses for the given models.

## Compile and run
Execute the following command in the add_model directory to compile the program
```
mkdir build && cd build && cmake .. && make
```
Execute the following command to run the program
```
./offline_data_generation <input>
```
where \<input\> is an input file of a CAD model. The supported types are OBJ, PLY and STL. 

## Usage
*  Place the CAD models inside the CAD_models folder located in the root directory for this project.
*  If you e.g. wish to add a model called "example.obj", place the example.obj file inside the CAD_models folder and
execute the following command to run the program
    ```
    ./offline_data_generation example.obj
    ```
* The generation data will be saved in `data/model_data`
* SolidWorks part:save the part to stl format. open the stl part in meshlab and export mesh as ply. 

* Edit the config.ini file in the root directory for this project in order to change the configuration parameters when adding a new model.
* The most important configuration parameters are located under the "Add_Model-Default" section. 
* The configuration parameters under Add_Model-Advanced are just a recommendation and does not need to be modified. 
* However, If you are familiar with PCL and point cloud processing then feel free to change the configuration parameters under Add_Model-Advanced 

## Issue
It has been reported that using a NVIDIA GTX 1080 graphics card will sometimes cause problem when rendering the synthetic views. See [this](https://github.com/PointCloudLibrary/pcl/issues/2188) issue for more information. Unfortunately, as of this moment there does not exists a valid solution for this problem. We suggest just re-rendering all views until no "visual error" can be seen in the merged point cloud (enable the view_complete_model to inspect the final merged point cloud. If there are strange looking points then it is adviced to re-render all views). However, some improvment can be seen by enabling the use_delay setting in config.ini. Introducing a small delay before rendering the first view made the problem occur less frequently.

