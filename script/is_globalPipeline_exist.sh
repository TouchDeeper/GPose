#!/bin/bash
./globalPipeline&
globalPipeline_PID=$!
echo "globalPipeline_PID = "$globalPipeline_PID
sleep 1s
#isGlobalPipelineExist=`ps -ef|grep globalPipeline|grep -v "grep"|wc -l`

#while [ "$isGlobalPipelineExist" -ne "0" ]
#do
#  echo "sleep 1s"
#  sleep 1s
#  isGlobalPipelineExist=`ps -ef|grep globalPipeline|grep -v "grep"|wc -l`
#done
while [ -d /proc/$globalPipeline_PID ]
do
  echo "sleep 1s"
  sleep 1s
#  isGlobalPipelineExist=`ps -ef|grep globalPipeline|grep -v "grep"|wc -l`
done