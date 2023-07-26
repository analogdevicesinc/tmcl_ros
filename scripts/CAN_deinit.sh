#!/bin/bash

comm_interface=$1
result=1

if [ "$(whoami)" != root ]
then
  echo "Please run this script with sudo. Exiting!"
  exit

else
  if [[ $# -le 0 ]]
  then
    echo "Missing arguments <communication interface>"
    exit 1
  elif [[ $# -ge 2 ]]
  then
    echo "Arguments exceeded <communication interface>"
    exit 1
  fi

  if [ "$comm_interface" -ge 256 ]
  then
    echo " "
    echo "Communication interface out of range"
    echo " > Accepts from 0 - 255"
  else
    pkexec ip link set can$comm_interface down
    result=$?

    if [ "$result" -eq 0 ]
    then
      echo "Stop systemd-networkd ..."
      systemctl stop systemd-networkd
      sleep 5

      network_file="99-can.network"
      echo "Removing CAN network file ..."
      rm /etc/systemd/network/${network_file}

      echo "Restarting systemd-networkd ..."
      systemctl restart systemd-networkd
      sleep 5
      
      result=$?
    fi
  fi

  if [ "$result" -eq 0 ]
  then
    echo ""
    echo "============================================================"
    echo "Done."
    echo "Automatic CAN bring up is now disabled"
  fi
  
fi
