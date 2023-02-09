#!/bin/bash

source /opt/ros/noetic/setup.bash 

if [ -z "$1" ]
then
  topic=/cmd_trq_0
else
  topic=$1
fi

val=300
rostopic pub $topic std_msgs/Int32 "data: $val" -1
sleep 5

val=0
rostopic pub $topic std_msgs/Int32 "data: $val" -1
sleep 5

val=300
rostopic pub $topic std_msgs/Int32 "data: $val" -1
sleep 5

val=0
rostopic pub $topic std_msgs/Int32 "data: $val" -1
