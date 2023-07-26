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
    network_file="99-can.network"
    echo "Creating $network_file ..."
    rm -f $network_file
    touch $network_file

    echo "[Match]" >> $network_file
    echo "Name=can$comm_interface" >> $network_file
    echo "[CAN]" >> $network_file
    echo "BitRate=$bitrate" >> $network_file

    echo "Moving $network_file to /etc/systemd/network ..."
    cp $network_file /etc/systemd/network

    echo "Stop systemd-networkd ..."
    systemctl stop systemd-networkd
    sleep 5

    echo "Enabling systemd-networkd ..."
    systemctl enable systemd-networkd
    sleep 5

    echo "Restarting systemd-networkd ..."
    systemctl restart systemd-networkd
    sleep 5

    result=$?
    rm $network_file
  fi

  if [ "$result" -eq 0 ]
  then
    echo ""
    echo "============================================================"
    echo "Done."
    echo "can$comm_interface will automatically be brought UP on boot"
  fi
  
fi
