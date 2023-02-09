#!/bin/bash

comm_interface=$1
bitrate=$2

arg_pass=true
result=1

if [ "$(whoami)" != root ]
then
  echo "Please run this script with sudo. Exiting!"
  exit

else

  if [[ $# -le 1 ]]
  then
    echo "Missing arguments <communication interface> <bitrate>"
    exit 1
  elif [[ $# -ge 3 ]]
  then
    echo "Arguments exceeded <communication interface> <bitrate>"
    exit 1
  fi

  if [ "$comm_interface" -ge 256 ]
  then
    echo " "
    echo "Communication interface out of range"
    echo " > Accepts from 0 - 255"
    arg_pass=false
  fi

  if [ "$bitrate" != 20000 -a "$bitrate" != 50000 -a "$bitrate" != 100000 -a \
      "$bitrate" != 125000 -a "$bitrate" != 250000 -a "$bitrate" != 500000 -a \
      "$bitrate" != 1000000 ]
  then 
    echo " "
    echo "Invalid bitrate"
    echo " == Accepted values =="
    echo " > 20000"
    echo " > 50000"
    echo " > 100000"
    echo " > 125000"
    echo " > 250000"
    echo " > 500000"
    echo " > 1000000"
    arg_pass=false
  fi

  if [ "$arg_pass" = true ]
  then
    # set can interface down to avoid multiple bring up on a single can interface
    pkexec ip link set can$comm_interface down
    pkexec ip link set can$comm_interface up type can bitrate $bitrate
    result=$?
  fi

  if [ "$result" -eq 0 ]
  then
    echo "Initialized! You can now run node or can_interface scripts"
  fi
  
fi