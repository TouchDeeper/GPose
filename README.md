# Overview
Robust pose estimation of objects with feature ambiguity based on point cloud. 

Youtube video, If you cannot link to youtube, try [bilibili](https://www.bilibili.com/video/BV1ya4y1h7iX/).
[![video](https://res.cloudinary.com/marcomontalbano/image/upload/v1593691440/video_to_markdown/images/youtube--H6Rpz66P_sE-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://www.youtube.com/watch?v=H6Rpz66P_sE&feature=youtu.be "video")



# Getting Started
## Prerequisites
- Eigen3
- Sophus
- PCL
- OpenCV: modify the [`OpenCV_DIR`](https://github.com/TouchDeeper/TdLib/blob/dev/src/CMakeLists.txt#L8) to your path

- OpenMP
- Boost
- [TdLib](https://github.com/TouchDeeper/TdLib): modify the [`TdLib_DIR`](https://github.com/TouchDeeper/GPose/blob/36d8dcdd93ca701be0a06f220a7747766273ca32/CMakeLists.txt#L25) to your path
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
- `./globalPipeline`

## Citation
```
@ARTICLE{10261416,
  author={Li, Hai and Zeng, Qingfu and Zhuang, Tingda and Huang, Yanjiang and Zhang, Xianmin},
  journal={IEEE Sensors Journal}, 
  title={Accurate Pose Estimation of the Texture-Less Objects With Known CAD Models via Point Cloud Matching}, 
  year={2023},
  volume={23},
  number={21},
  pages={26259-26268},
  keywords={Point cloud compression;Pose estimation;Solid modeling;Feature extraction;Three-dimensional displays;Genetic algorithms;Data models;Feature ambiguity;iterative closest point (ICP);point cloud matching;pose estimation;texture-less objects},
  doi={10.1109/JSEN.2023.3316457}}
```

## Acknowledgment
`add_model` part is modified from the [object_identification_localization](https://github.com/Laxen/object_identification_localization) project.

