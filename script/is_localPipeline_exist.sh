#!/bin/bash
./local_pipeline&
localPipeline_PID=$!
echo "localPipeline_PID = "$localPipeline_PID
sleep 1s
#islocalPipelineExist=`ps -ef|grep localPipeline|grep -v "grep"|wc -l`

#while [ "$islocalPipelineExist" -ne "0" ]
#do
#  echo "sleep 1s"
#  sleep 1s
#  islocalPipelineExist=`ps -ef|grep localPipeline|grep -v "grep"|wc -l`
#done
while [ -d /proc/$localPipeline_PID ]
do
  echo "sleep 1s"
  sleep 1s
#  islocalPipelineExist=`ps -ef|grep localPipeline|grep -v "grep"|wc -l`
done