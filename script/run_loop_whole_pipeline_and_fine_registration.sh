#!/bin/bash

#power=1
#power=$(echo "($run_num-1)/2"|bc)
config_path="/home/wang/imta_project/pose_estimation/config.ini"
trap 'onCtrlC' INT
keep=1
function onCtrlC () {
    echo 'Ctrl+C is captured'
    keep=0
    kill -9 $globalPipeline_PID
    echo "kill localPipeline_pid"
    sed -i "s:^pure_GA_path.*\.txt$:pure_GA_path=pure_GA.txt:g" ${config_path}
    sed -i "s:^pure_icp_path.*\.txt$:pure_icp_path=pure_icp.txt:g" ${config_path}
    sed -i "s:^GA_icp_path.*\.txt$:GA_icp_path=GA_icp.txt:g" ${config_path}
    sed -i "s:^time_path.*\.txt$:time_path=time.txt:g" ${config_path}
    sed -i "s:^verbose=.*[a-zA-Z]$:verbose=true:g" ${config_path}
    sed -i "s:^visual=.*[a-zA-Z]$:visual=true:g" ${config_path}
#    sed -i "s:^F0=.*[0-9]$:F0=0.4:g" ~/imta_project/objectPoseEstimation/src/src/localPipeline/config.ini
    exit
}
./run_loop_fine_registration.sh
./run_loop_whole_pipeline.sh
sed -i "s:^pure_GA_path.*\.txt$:pure_GA_path=pure_GA.txt:g" ${config_path}
sed -i "s:^pure_icp_path.*\.txt$:pure_icp_path=pure_icp.txt:g" ${config_path}
sed -i "s:^GA_icp_path.*\.txt$:GA_icp_path=GA_icp.txt:g" ${config_path}
sed -i "s:^time_path.*\.txt$:time_path=time.txt:g" ${config_path}
sed -i "s:^verbose=.*[a-zA-Z]$:verbose=true:g" ${config_path}
sed -i "s:^visual=.*[a-zA-Z]$:visual=true:g" ${config_path}





