# Overview
This is a object pose estimation based on PCL.


# Getting Started
## Prerequisites
- Eigen3
- PCL
- OpenCV: modify the [`OpenCV_DIR`](https://github.com/TouchDeeper/TdLib/blob/dev/src/CMakeLists.txt#L8) to your path

- OpenMP
- [TdLib](https://github.com/TouchDeeper/TdLib): modify the [`TdLib_DIR`](https://github.com/TouchDeeper/TdLib/blob/dev/src/CMakeLists.txt#L8) to your path
- [nlohmann_json](https://github.com/nlohmann/json)
## Usage
- add model and generate the offline data, skip to `add_model/README.md` to know how to.
- capture the object point cloud in the scene add save it in the `data` in the pcd format and in meters unit.
- modify the `config.ini`. The main item you need to modify is `model_name`,`scene_name`, `cad_models_path`,`model_data_path`,
- in the root directory
    ```asm
    mkdir build
    cd build
    cmake ..
    make -j4
    ```


## Acknowledgment
`add_model` part is modified from the [object_identification_localization](https://github.com/Laxen/object_identification_localization) project.

