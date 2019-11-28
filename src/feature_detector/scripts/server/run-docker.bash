#!/usr/bin/env bash

cd $1/scripts/server

set -e
# https://stackoverflow.com/a/13322549
export IP_ADDRESS=127.0.0.1
docker build --tag ros3 --file ros3.Dockerfile .
docker build --tag shape_detect .
docker run \
    -it \
    -e DISPLAY \
    -e ROS_IP=$IP_ADDRESS \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $XAUTHORITY:/root/.Xauthority  \
    --device=/dev/video0:/dev/video0 \
    --net=host \
    shape_detect

cd -