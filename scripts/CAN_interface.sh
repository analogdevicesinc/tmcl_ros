#!/bin/bash

change_tx_id=false
change_rx_id=false
change_bitrate=false

tx_id=0
rx_id=0
bitrate=0

# Instruction Types for Global Parameter
cmd_type_bitrate=69
cmd_type_rxid=70
cmd_type_txid=71

# Files
def_launch='tmcm_1636.launch'
def_yaml='tmcm_1636.yaml'

# Text colors
CYAN='\033[0;36m'
GREEN='\033[0;32m'
RED='\033[0;31m'
BLUE='\033[1;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

if [ "$(whoami)" != root ]
then

  echo -e "${GREEN}STARTING...${NC}"
  echo "Run this if you intend to change the CAN configurations"
  echo "IMPORTANT! Do not STOP via CTRL-C in the middle of this script once you have already seen \"PROCESSING! DO NOT STOP NOW!\" message!"
  echo "Repercussions when you stop this script's execution midway is you were able to change CAN configurations and you don't know the "
  echo "last set CAN configurations in the controller so you won't be able to communicate with it anymore."
  echo " "
  echo -e "${YELLOW}WARNING! This script will kill all actively running ROS nodes.${NC}"

  while true
  do
    read -p "Continue? (Y/N)" yn
    case $yn in
      [yY] ) 
        break ;;
      [nN] ) 
        echo "Exiting..."
        exit 0 ;;
      * )
        echo -e "${RED}Invalid input.${NC}";;
    esac
  done 

  echo " "
  while true
  do
    read -p "Set TxID? (Y/N)" yn
    case $yn in
      [yY] ) 
        read -p "Enter new tx_id value: " tx_id
        while [ "$tx_id" -ge 256 -o "$tx_id" -le -1 ]
        do 
          echo  -e "${RED}Value out of range.${NC}"
          echo " > Accepts from 0 - 255"
          read -p "Enter new tx_id value: " tx_id
        done
        change_tx_id=true
        break ;;
      [nN] ) 
        break ;;
      * )
        echo -e "${RED}Invalid input.${NC}";;
    esac
  done 

  echo " "
  while true
  do
    read -p "Set RxID? (Y/N)" yn
    case $yn in
      [yY] ) 
        read -p "Enter new rx_id value: " rx_id
        while [ "$rx_id" -ge 256 -o "$rx_id" -le -1 ]
        do 
          echo -e "${RED}Value out of range.${NC}"
          echo " > Accepts from 0 - 255"
          read -p "Enter new rx_id value: " rx_id
        done
        change_rx_id=true
        break ;;
      [nN] ) 
        break ;;
      * )
        echo -e "${RED}Invalid input.${NC}";;
    esac
  done 

  echo " "
  while true
  do
    read -p "Set Bitrate? (Y/N)" yn
    case $yn in
      [yY] ) 
        read -p "Enter new bitrate value: " bitrate
        while [ "$bitrate" -ge 9 -o "$bitrate" -le 1 ]
        do 
          echo -e "${RED}Value out of range.${NC}"
          echo " == Accepted values =="
          echo " > 2 = 20KBPS"
          echo " > 3 = 50KBPS"
          echo " > 4 = 100KBPS"
          echo " > 5 = 125KBPS"
          echo " > 6 = 250KBPS"
          echo " > 7 = 500KBPS"
          echo " > 8 = 1000KBPS"
          read -p "Enter new bitrate value: " bitrate
        done
        change_bitrate=true
        break ;;
      [nN] ) 
        break ;;
      * )
        echo -e "${RED}Invalid input.${NC}";;
    esac
  done 

  if [ "$change_tx_id" = false -a "$change_rx_id" = false -a "$change_bitrate" = false ]
  then
    echo -e "${YELLOW}No parameter is changed.${NC} Exiting"
    exit
  fi

  cd src/
  echo " "
  
  for entry in `ls "tmcl_ros/launch"`
  do
    echo -e ${CYAN}$entry${NC}
  done

  while true
  do
    read -p "Enter launch file to use: " def_launch
    launch_location=tmcl_ros/launch/$def_launch
    if [ -f "$launch_location" ]
    then
      echo "Launch file exists."
      break
    else
      echo -e "${RED}Launch file does not exist.${NC} Please select another file"
    fi   
  done 

### Script Proper ###

  echo " "
  echo "========================================================"
  echo -e "${GREEN}PROCESSING! DO NOT STOP NOW!${NC}"
  echo "========================================================"

  # Creates temporary launch file
  rm -f tmcl_ros/launch/tmp.launch
  cp tmcl_ros/launch/$def_launch tmcl_ros/launch/tmp.launch

  # Extract yaml file name from the launch file
  LOC=$(grep -w "file=" tmcl_ros/launch/tmp.launch)
  def_yaml=${LOC%\"*}
  def_yaml=${def_yaml##*"config/"} 
  echo -e "YAML file detected from the launch file: ${CYAN}$def_yaml${NC}"

  yaml_location=tmcl_ros/config/$def_yaml
  if [ -f "$yaml_location" ]
  then
    echo "YAML file exists"

    if [ -d "tmcl_ros/tmp/" -a "$(ls -A tmcl_ros/tmp/)" ]
    then
      while true
      do
        echo " "
        echo -e "${YELLOW}There is tmp folder already and it is not empty. ${NC}Do you want to continue? (Y/N)"
        echo "Note: If (Y), the tmp folder and its contents will be removed. Check tmcl_ros/tmp first!"
        read -p "Note: If (N), the script will exit." yn
        case $yn in
          [yY] ) 
            break ;;
          [nN] )
            rm -f tmcl_ros/launch/tmp.launch
            echo "Exiting"
            exit 0 ;;
          * )  
            echo -e "${RED}Invalid input.${NC}";;
        esac
      done
    fi

    # Creates temporary folder which will contain copied (temporary) yaml file loaded to the launch file
    rm -rf tmcl_ros/tmp/
    mkdir tmcl_ros/tmp/
    cp tmcl_ros/config/$def_yaml tmcl_ros/tmp/
    LOC=$(grep -n "Ext" tmcl_ros/launch/tmp.launch | cut -d':' -f 1)          ## Get LOC of Extension yaml in launch file
    sed -i "$LOC s/config/tmp/" tmcl_ros/launch/tmp.launch                    ## Replace config location to tmp
  else
    echo -e "${RED}YAML file does not exist. Please check your YAML file at tmcl_ros/config folder. Exiting!${NC}"
    exit 1
  fi

  # Extract servicename from tmcl_ros_service.cpp file
  SN=$(grep -w "s_node_name +"  tmcl_ros/src/tmcl_ros_service.cpp)
  servicename=${SN%\"*} 
  servicename=${servicename##*\"} 
  echo " "
  echo -e "Found ROS Service: ${CYAN}$servicename${NC}"
  echo " "

  if [ "$change_tx_id" = true ]
  then
    echo "Now changing tx_id..."
    roslaunch tmcl_ros tmp.launch mode:="service" &
    sleep 2

    for entry in `rosservice list`
    do
      if [[ $entry == *"$servicename"* ]]
      then
        break
      fi
    done

rosservice call $entry "instruction: 'SGP'
instruction_type: $cmd_type_txid
motor_num: 0
value: $tx_id" &
    sleep 2
    rosnode kill -a
    sleep 1

    LOC=$(grep -n "comm_tx_id" tmcl_ros/tmp/$def_yaml | cut -d':' -f 1)         ## Get LOC where comm_tx_id is placed
    sed -i "$LOC s/.*/comm_tx_id: $tx_id/g" tmcl_ros/tmp/$def_yaml              ## Change whole LOC with comm_tx_id: value
    echo "tx_id changed. New value: $tx_id"
    echo " "
    echo -e "${YELLOW}Next process will take 10 seconds.Please wait...${NC}"
    sleep 10
    echo " "
  fi

  if [ "$change_rx_id" = true ]
  then
    echo "Now changing rx_id..."
    roslaunch tmcl_ros tmp.launch mode:="service" &
    sleep 2

    for entry in `rosservice list`
    do
      if [[ $entry == *"$servicename"* ]]
      then
        break
      fi
    done

rosservice call $entry "instruction: 'SGP'
instruction_type: $cmd_type_rxid
motor_num: 0
value: $rx_id" &
    sleep 2
    rosnode kill -a
    sleep 1
    
    LOC=$(grep -n "comm_rx_id" tmcl_ros/tmp/$def_yaml | cut -d':' -f 1)
    sed -i "$LOC s/.*/comm_rx_id: $rx_id/g" tmcl_ros/tmp/$def_yaml
    echo "rx_id changed. New value: $rx_id"
    echo " "
    echo -e "${YELLOW}Next process will take 10 seconds.Please wait...${NC}"
    sleep 10
    echo " "
  fi

  if [ "$change_bitrate" = true ]
  then
    echo " "
    echo "Now changing bitrate..."
    roslaunch tmcl_ros tmp.launch mode:="service" &
    sleep 2

    for entry in `rosservice list`
    do
      if [[ $entry == *"$servicename"* ]]
      then
        break
      fi
    done

rosservice call $entry "instruction: 'SGP'
instruction_type: $cmd_type_bitrate
motor_num: 0
value: $bitrate" &
    sleep 2
    rosnode kill -a
    sleep 1

    LOC=$(grep -n "comm_bit_rate" tmcl_ros/tmp/$def_yaml | cut -d':' -f 1)
    case $bitrate in
      [2] )
        bitrate=20000
        ;;
      [3] )
        bitrate=50000
        ;;
      [4] )
        bitrate=100000
        ;;
      [5] )
        bitrate=125000
        ;;
      [6] )
        bitrate=250000
        ;;
      [7] )
        bitrate=500000
        ;;
      * )
        bitrate=1000000
        ;;
      esac
    sed -i "$LOC s/.*/comm_bit_rate: $bitrate/g" tmcl_ros/tmp/$def_yaml
    echo "bitrate changed. New value: $bitrate"
    echo " "
    echo -e "${YELLOW}Next process will take 5 seconds. Please wait...${NC}"
    sleep 5
    echo " "
  fi

  rm -f tmcl_ros/launch/tmp.launch
  echo " "
  echo "========================================================"
  echo -e "${GREEN}DONE PROCESSING!${NC}"
  echo "Updated yaml file with the new CAN configurations at tmcl_ros/tmp/$def_yaml"
  while true
    do
      read -p "Do you want to replace your tmcl_ros/config/$def_yaml with this one? (Y/N)" yn
      case $yn in
        [yY] ) 
          rm -f tmcl_ros/config/$def_yaml
          cp tmcl_ros/tmp/$def_yaml tmcl_ros/config
          rm -rf tmcl_ros/tmp/
          echo "Replaced! You may now readily use this updated tmcl_ros/config/$def_yaml"
          break ;;
        [nN] )
          echo -e "${YELLOW}Not replaced! Take note that your original tmcl_ros/config/$def_yaml might not work anymore since the following changes are needed${NC}"
          echo -e "${BLUE}--------------------------------------------------------${NC}"
          diff -Naur tmcl_ros/config/$def_yaml tmcl_ros/tmp/$def_yaml
          echo -e "${BLUE}--------------------------------------------------------${NC}"
          echo "tmp folder will not be removed."
          echo "If you change your mind later, replace tmcl_ros/config/$def_yaml with the corrected yaml at tmcl_ros/tmp/$def_yaml"
          break ;;
        * )  
          echo -e "${RED}Invalid input.${NC}";;
      esac
    done
  if [ "$change_bitrate" = true ]
  then
    echo " "
    echo -e "${YELLOW}New bitrate detected!${NC}"
    echo "Please restart the board and run CAN_init.sh with the new bitrate as input."
    echo "Otherwise, error will occur for the next run of TMCL rosnode"
  fi
  echo " "
  echo "EXITING..."
  echo "========================================================"

else
  echo -e "${RED}Do not run this script on sudo. Exiting${NC}"
  exit
fi
