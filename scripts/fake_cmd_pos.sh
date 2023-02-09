#!/bin/bash

source /opt/ros/noetic/setup.bash 

if [ -z "$1" ]
then
  topic=/cmd_relpos_0
else
  topic=$1
fi

# Increasing speed clockwise
val=0
for i in {1..3}
do
val=360
rostopic pub $topic std_msgs/Int32 "data: $val" -1
sleep 5
done

# Increasing speed counter-clockwise
val=0
for i in {1..3}
do
val=-360
rostopic pub $topic std_msgs/Int32 "data: $val" -1
sleep 5
done
