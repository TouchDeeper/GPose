#!/bin/bash

#run_num=120
config_path="/home/wang/imta_project/pose_estimation/config.ini"
trap 'onCtrlC' INT
keep=1
function onCtrlC () {
    echo 'Ctrl+C is captured'
    keep=0
    kill -9 $globalPipeline_PID
    echo "kill globalPipeline_pid"
    sed -i "s:^cad_models_path.*$:cad_models_path=/home/wang/imta_project/pose_estimation/CAD_models:g" ${config_path}
    sed -i "s:^model_data_path.*$:model_data_path=/home/wang/imta_project/pose_estimation/data/model_data:g" ${config_path}
    sed -i "s:^pure_GA_path.*\.txt$:pure_GA_path=\.\./result/pure_GA.txt:g" ${config_path}
    sed -i "s:^verbose=.*[a-zA-Z]$:verbose=true:g" ${config_path}
    sed -i "s:^visual=.*[a-zA-Z]$:visual=true:g" ${config_path}
    sed -i "s:^view_graph=.*[a-zA-Z]$:view_graph=true:g" ${config_path}
    sed -i "s:^view_complete_model=.*[a-zA-Z]$:view_complete_model=true:g" ${config_path}
#    sed -i "s:^F0=.*[0-9]$:F0=0.4:g" ~/imta_project/objectPoseEstimation/src/src/localPipeline/config.ini
    exit
}


cad_model_path="/media/wang/File/dataset/ycbv/models"
sed -i "s:^verbose=.*[a-zA-Z]$:verbose=false:g" ${config_path}
sed -i "s:^visual=.*[a-zA-Z]$:visual=false:g" ${config_path}
sed -i "s:^cad_models_path.*$:cad_models_path=$cad_model_path:g" ${config_path}
sed -i "s:^model_data_path.*$:model_data_path=/media/wang/File/dataset/ycbv/model_data:g" ${config_path}
sed -i "s:^view_graph=.*[a-zA-Z]$:view_graph=false:g" ${config_path}
sed -i "s:^view_complete_model=.*[a-zA-Z]$:view_complete_model=false:g" ${config_path}

#cd /home/wang/imta_project/objectPoseEstimation/src/src/localPipeline/cmake-build-debug/
#rm ../result/banana/dynamic_threading/*.txt
#for loop in true false
#do
##  value=$(awk -v j_awk="$j" 'BEGIN {value_awk=j_awk/10.0; printf "%.1f", value_awk}')
#  sed -i "s:^dynamic_threading=.*[a-zA-Z]$:dynamic_threading=$loop:g" ~/imta_project/objectPoseEstimation/src/src/localPipeline/config.ini
#  sed -i "s:^pure_GA_path.*\.txt$:pure_GA_path=\.\./result/banana/dynamic_threading/$loop.txt:g" ~/imta_project/objectPoseEstimation/src/src/localPipeline/config.ini

#done
#sed -i "s:^pure_GA_path.*\.txt$:pure_GA_path=\.\./result/pure_GA.txt:g" ~/imta_project/objectPoseEstimation/src/src/localPipeline/config.ini
#sed -i "s:^verbose=.*[a-zA-Z]$:verbose=true:g" ~/imta_project/objectPoseEstimation/src/src/localPipeline/config.ini
#sed -i "s:^visual=.*[a-zA-Z]$:visual=true:g" ~/imta_project/objectPoseEstimation/src/src/localPipeline/config.ini


executable_file_path="/home/wang/imta_project/pose_estimation/add_model/cmake-build-debug"
cd  $executable_file_path

a=9
for((i=1;;i++))
do
    cd $cad_model_path
    model_name="obj_0000"
    suffix=".ply"
    if [ $i -le $a ]
    then
        prefix="0"
        prefix=$prefix$i
        model_name=$model_name$prefix$suffix
    else
        model_name=$model_name$i$suffix
    fi

    if [ ! -f "$model_name" ]
    then
        echo "there is no $model_name exist"
        break
    fi
    cd $executable_file_path
    ./offline_data_generation $model_name
done

sed -i "s:^verbose=.*[a-zA-Z]$:verbose=true:g" ${config_path}
sed -i "s:^visual=.*[a-zA-Z]$:visual=true:g" ${config_path}
sed -i "s:^cad_models_path.*$:cad_models_path=/home/wang/imta_project/pose_estimation/CAD_models:g" ${config_path}
sed -i "s:^model_data_path.*$:model_data_path=/home/wang/imta_project/pose_estimation/data/model_data:g" ${config_path}
sed -i "s:^view_graph=.*[a-zA-Z]$:view_graph=true:g" ${config_path}
sed -i "s:^view_complete_model=.*[a-zA-Z]$:view_complete_model=true:g" ${config_path}
