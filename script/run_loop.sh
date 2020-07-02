#!/bin/bash
#acc_n=0.08          # accelerometer measurement noise standard deviation. #0.2   0.04
#gyr_n=0.004         # gyroscope measurement noise standard deviation.     #0.05  0.004
#acc_w=0.00004         # accelerometer bias random work noise standard deviation.  #0.02
#gyr_w=2.0e-6       # gyroscope bias random work noise standard deviation.     #4.0e-5
#scale=1
run_num=120
#power=1
#power=$(echo "($run_num-1)/2"|bc)


#sed -i "s/^acc_n: [0-9]*\.[0-9]*/acc_n: $acc_n/g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml
#sed -i "s/^acc_w: [0-9]*\.[0-9]*/acc_w: $acc_w/g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml
#sed -i "s/^gyr_n: [0-9]*\.[0-9]*/gyr_n: $gyr_n/g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml
#sed -i "s/^gyr_w: .[0-9]*\.[^[:space:]]*\|^gyr_w: \.[^[:space:]]*/gyr_w: $gyr_w/g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml

#let power=(run_num-1)/2
#eval $(awk -v power_awk="$power" 'BEGIN {scale_awk=0.5^power_awk; printf "scale=%.15f", scale_awk}')
#scale=$(awk -v power_awk="$power" 'BEGIN {scale_awk=0.5^power_awk; printf "%.15f", scale_awk}')

#echo $scale

#scale=$(echo "0.5**$power"|bc)
sed -i "s:^verbose=.*[a-zA-Z]$:verbose=false:g" ~/imta_project/objectPoseEstimation/src/src/localPipeline/config.ini
cd /home/wang/imta_project/objectPoseEstimation/src/src/localPipeline/cmake-build-debug/
rm ../result/banana/mutation/non_uniform.txt ../result/banana/mutation/uniform.txt 
for loop in non_uniform uniform
#for loop in GYR_W
do
  sed -i "s:^mutation_method=.*[a-zA-Z]$:mutation_method=$loop:g" ~/imta_project/objectPoseEstimation/src/src/localPipeline/config.ini
  sed -i "s:^output_path.*\.txt$:output_path=\.\./result/banana/mutation/$loop.txt:g" ~/imta_project/objectPoseEstimation/src/src/localPipeline/config.ini
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

    ./localPipeline&
    localPipeline_PID=$!
    echo "localPipeline_PID = "$localPipeline_PID
    sleep 1s
    isLocalPipelineExist=`ps -ef|grep localPipeline|grep -v "grep"|wc -l`
    while [ "$isLocalPipelineExist" -ne "0" ]
    do
      echo "sleep 1s"
      sleep 1s
      isLocalPipelineExist=`ps -ef|grep localPipeline|grep -v "grep"|wc -l`
    done
  done 
  
  #restore the noise parameters and vio path
  #sed -i "s/^acc_n: [0-9]*\.[0-9]*/acc_n: $acc_n/g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml
  #sed -i "s/^acc_w: [0-9]*\.[0-9]*/acc_w: $acc_w/g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml
  #sed -i "s/^gyr_n: [0-9]*\.[0-9]*/gyr_n: $gyr_n/g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml
  #sed -i "s/^gyr_w: .[0-9]*\.[^[:space:]]*\|^gyr_w: \.[^[:space:]]*/gyr_w: $gyr_w/g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml
  #sed -i "s:^vio_path.*\.txt\"$:vio_path\: \"/home/wang/vins_ws/src/VINS-Mono/output/euroc_result/vio\.txt\":g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml
done
sed -i "s:^output_path.*\.txt$:output_path=\.\./result/output.txt:g" ~/imta_project/objectPoseEstimation/src/src/localPipeline/config.ini





