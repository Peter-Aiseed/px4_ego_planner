#!/bin/bash

xhost +local:root

docker rm noetic

#docker run -it --rm \
#  --platform linux/arm64 \
#  --network host \
#  --privileged \
#  -e DISPLAY=$DISPLAY \
#  -v /tmp/.X11-unix:/tmp/.X11-unix \
#  -v $HOME/.Xauthority:/root/.Xauthority:ro \
#  openvins-noetic-pi5

# docker run -it -d \
#   --platform linux/arm64 \
#   --network host \
#   --privileged \
#   --device /dev/bus/usb \
#   --device /dev/dri \
#   -e DISPLAY=$DISPLAY \
#   -e LIBGL_ALWAYS_SOFTWARE=1 \
#   -e MESA_GL_VERSION_OVERRIDE=3.3 \
#   -e QT_X11_NO_MITSHM=1 \
#   -v /dev:/dev \
#   -v /tmp/.X11-unix:/tmp/.X11-unix \
#   -v $HOME/.Xauthority:/root/.Xauthority:ro \
#   -v $HOME/Projects/realsense:/ws_realsense \
#   --name noetic noetic:latest


docker run -it -d \
  --platform linux/arm64 \
  --network host \
  --privileged \
  -w /root/.. \
  --device /dev/bus/usb \
  --device /dev/dri \
  -e DISPLAY=$DISPLAY \
  -e LIBGL_ALWAYS_SOFTWARE=0 \
  -v /dev:/dev \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/.Xauthority:/root/.Xauthority:ro \
  -v $HOME/Projects/ego_ws:/ws_ego \
  --name noetic noetic:latest \
  bash -c "
    if ! grep -q '/ws_ego/devel/setup.bash' ~/.bashrc; then
      echo 'source /ws_ego/devel/setup.bash' >> ~/.bashrc
    fi
    if ! grep -q 'ROS_IP' ~/.bashrc; then
      echo 'export ROS_IP=\$(hostname -I | cut -d\" \" -f1)' >> ~/.bashrc
      echo 'export ROS_MASTER_URI=http://\$ROS_IP:11311' >> ~/.bashrc
      echo 'unset ROS_HOSTNAME' >> ~/.bashrc
    fi
    bash
  "
