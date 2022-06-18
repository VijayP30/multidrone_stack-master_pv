#!/bin/bash

export ROS_IP=$(hostname -I | cut -d' ' -f1)
echo "Set ROS_IP to $ROS_IP"
