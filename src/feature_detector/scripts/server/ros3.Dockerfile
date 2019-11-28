# ROS Kinetic with Python 3.5.2
FROM ubuntu:xenial
RUN apt-get update
RUN apt-get install -y wget
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN wget http://packages.ros.org/ros.key -O - | apt-key add -
RUN apt-get update

RUN apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential \
 python-catkin-tools python3-dev python3-catkin-pkg-modules \
 ros-kinetic-cv-bridge
RUN rosdep init
RUN rosdep update
RUN mkdir /ros_catkin_ws
WORKDIR /ros_catkin_ws
RUN rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall

RUN roslocate --distro=kinetic info sensor_msgs >> kinetic-ros_comm-wet.rosinstall
RUN roslocate --distro=kinetic info vision_opencv >> kinetic-ros_comm-wet.rosinstall

RUN wstool init -j8 src kinetic-ros_comm-wet.rosinstall
RUN rosdep install --from-paths src --ignore-src --rosdistro kinetic -y

RUN apt-get install -y python3-pip
RUN python3 -m pip install --upgrade pip
RUN pip3 install catkin_pkg pyyaml empy

RUN ln -T /usr/lib/x86_64-linux-gnu/libboost_python-py35.so  /usr/lib/x86_64-linux-gnu/libboost_python3.so
RUN pip3 install opencv-python
ENV OpenCV_DIR=/opt/ros/kinetic/share/OpenCV-3.3.1-dev
RUN catkin build --cmake-args -DPYTHON_VERSION=3.5.2
