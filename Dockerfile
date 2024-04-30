# import the ros2 humble 
FROM ros:humble

SHELL ["/bin/bash", "-c"]
# Install dependencies needed for FaSTTUBe
RUN sudo apt update && sudo apt -y upgrade
RUN apt-get -y install git curl python3 python3-pip python3-colcon-common-extensions
RUN yes | pip install --upgrade pip
    ## install packaged dependencies
RUN yes | pip install setuptools==58.2 catkin-pkg

# Create and go to ros2 src directory
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws/src

# Add packages to ros2
COPY /git_fetched /root/ros2_ws/src

# This package
RUN mkdir -p cone_markers/resource cone_markers/src cone_markers/test
ADD ./package.xml cone_markers/
ADD ./setup.cfg cone_markers/
ADD ./setup.py cone_markers/
ADD ./resource/* cone_markers/resource/
ADD ./src/* cone_markers/src/
ADD ./test/* cone_markers/test/

## Source ROS2 & Local build
WORKDIR /root/ros2_ws
RUN source /opt/ros/humble/setup.bash \
    && colcon build \
    && source ./install/setup.bash

# ENTRYPOINT
RUN printf "#!/usr/bin/env bash \n . /opt/ros/humble/setup.sh \n . /root/ros2_ws/install/setup.sh \n ros2 run cone_markers mark_cones" >> /root/entry.sh
RUN chmod 755 /root/entry.sh

ENTRYPOINT /root/entry.sh