#!/bin/bash
# Script to run Docker container with GUI support

# Allow X11 connections from localhost
xhost +local:root

# Run container with X11 forwarding
docker run -it --rm \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/root/.Xauthority:rw \
    --net=host \
    rpi-robotics:latest

# Revoke X11 access
xhost -local:root
