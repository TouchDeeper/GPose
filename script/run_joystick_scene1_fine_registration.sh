#!/bin/bash
#acc_n=0.08          # accelerometer measurement noise standard deviation. #0.2   0.04
#gyr_n=0.004         # gyroscope measurement noise standard deviation.     #0.05  0.004
#acc_w=0.00004         # accelerometer bias random work noise standard deviation.  #0.02
#gyr_w=2.0e-6       # gyroscope bias random work noise standard deviation.     #4.0e-5
#scale=1
run_num=120
#power=1
#power=$(echo "($run_num-1)/2"|bc)
config_path="/home/wang/imta_project/pose_estimation/config.ini"
trap 'onCtrlC' INT
keep=1
function onCtrlC () {
    echo 'Ctrl+C is captured'
    keep=0
    kill -9 $localPipeline_PID
    echo "kill localPipeline_pid"
    sed -i "s:^pure_GA_path.*\.txt$:pure_GA_path=pure_GA.txt:g" ${config_path}
    sed -i "s:^pure_icp_path.*\.txt$:pure_icp_path=pure_icp.txt:g" ${config_path}
    sed -i "s:^GA_icp_path.*\.txt$:GA_icp_path=GA_icp.txt:g" ${config_path}
    sed -i "s:^verbose=.*[a-zA-Z]$:verbose=true:g" ${config_path}
    sed -i "s:^visual=.*[a-zA-Z]$:visual=true:g" ${config_path}
#    sed -i "s:^F0=.*[0-9]$:F0=0.4:g" ~/imta_project/objectPoseEstimation/src/src/localPipeline/config.ini
    exit
}
#sed -i "s/^acc_n: [0-9]*\.[0-9]*/acc_n: $acc_n/g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml
#sed -i "s/^acc_w: [0-9]*\.[0-9]*/acc_w: $acc_w/g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml
#sed -i "s/^gyr_n: [0-9]*\.[0-9]*/gyr_n: $gyr_n/g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml
#sed -i "s/^gyr_w: .[0-9]*\.[^[:space:]]*\|^gyr_w: \.[^[:space:]]*/gyr_w: $gyr_w/g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml

#let power=(run_num-1)/2
#eval $(awk -v power_awk="$power" 'BEGIN {scale_awk=0.5^power_awk; printf "scale=%.15f", scale_awk}')
#scale=$(awk -v power_awk="$power" 'BEGIN {scale_awk=0.5^power_awk; printf "%.15f", scale_awk}')

#echo $scale

#scale=$(echo "0.5**$power"|bc)
sed -i "s:^verbose=.*[a-zA-Z]$:verbose=false:g" ${config_path}
sed -i "s:^visual=.*[a-zA-Z]$:visual=false:g" ${config_path}
sed -i "s:^mutation_method=.*[a-zA-Z]$:mutation_method=mutate_by_chromosomes:g" ${config_path}
sed -i "s:^fusion_mode=.*[a-zA-Z]$:fusion_mode=dynamic_fusion:g" ${config_path}

cd /home/wang/imta_project/pose_estimation/cmake-build-debug/

sed -i "s:^model_name=.*[a-zA-Z]$:model_name=joystick:g" ${config_path}
sed -i "s:^scene_name=.*[0-9]$:scene_name=scene1:g" ${config_path}
sed -i "s:^pure_GA_path.*\.txt$:pure_GA_path=fine_registration/scene1/pure_GA.txt:g" ${config_path}
sed -i "s:^pure_icp_path.*\.txt$:pure_icp_path=fine_registration/scene1/pure_icp.txt:g" ${config_path}
sed -i "s:^GA_icp_path.*\.txt$:GA_icp_path=fine_registration/scene1/GA_icp.txt:g" ${config_path}


for((i=0;i<$[run_num];i++))
do
#power_=$(echo "i-$power"|bc)
#let power_=i-power
#scale=$(awk -v power_awk="$power_" 'BEGIN {scale_awk=2^power_awk; printf "%.13f", scale_awk}')
# modify the vio_path
#sed -i "s:^vio_path.*\.txt\"$:vio_path\: \"/home/wang/vins_ws/src/VINS-Mono/output/euroc_result/$loop/vio$i\.txt\":g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml
#if [ "$loop" == "ACC_N" ];then
#  acc_n_scaled=$(echo "$acc_n*$scale"|bc)
  # modify the acc_n's value
#  sed -i "s/^acc_n: [0-9]*\.[0-9]*/acc_n: $acc_n_scaled/g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml
  #echo "scale="$scale "i="$i
 # echo "acc_n="$acc_n_scaled "i="$i
#fi
#if [ "$loop" == "ACC_W" ];then
#  acc_w_scaled=$(echo "$acc_w*$scale"|bc)
#  # modify the acc_w's value
#  sed -i "s/^acc_w: [0-9]*\.[0-9]*/acc_w: $acc_w_scaled/g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml
#  #echo "scale="$scale "i="$i
#  echo "acc_w="$acc_w_scaled "i="$i
#fi

    ../script/is_localPipeline_exist.sh
done

#restore the noise parameters and vio path
#sed -i "s/^acc_n: [0-9]*\.[0-9]*/acc_n: $acc_n/g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml
#sed -i "s/^acc_w: [0-9]*\.[0-9]*/acc_w: $acc_w/g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml
#sed -i "s/^gyr_n: [0-9]*\.[0-9]*/gyr_n: $gyr_n/g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml
#sed -i "s/^gyr_w: .[0-9]*\.[^[:space:]]*\|^gyr_w: \.[^[:space:]]*/gyr_w: $gyr_w/g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml
#sed -i "s:^vio_path.*\.txt\"$:vio_path\: \"/home/wang/vins_ws/src/VINS-Mono/output/euroc_result/vio\.txt\":g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml
sed -i "s:^pure_GA_path.*\.txt$:pure_GA_path=pure_GA.txt:g" ${config_path}
sed -i "s:^pure_icp_path.*\.txt$:pure_icp_path=pure_icp.txt:g" ${config_path}
sed -i "s:^GA_icp_path.*\.txt$:GA_icp_path=GA_icp.txt:g" ${config_path}
sed -i "s:^verbose=.*[a-zA-Z]$:verbose=true:g" ${config_path}
sed -i "s:^visual=.*[a-zA-Z]$:visual=true:g" ${config_path}





