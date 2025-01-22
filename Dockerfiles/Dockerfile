FROM osrf/ros:humble-desktop-full

# set the environment variable with the command ENV <key>=<value>, it can be replaced online
ENV DEBIAN_FRONTEND=noninteractive
# RUN is used to execute and add new layer on top of the base immage
RUN apt update --fix-missing
RUN apt install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ur
RUN apt install -y ros-humble-plotjuggler
RUN apt install -y \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-tf-transformations \
    ros-humble-xacro \
    ros-humble-rosbag2-storage-mcap \
    chrony 
RUN apt install -y \
    tmux python3-pip\
    libeigen3-dev \
    nano python3-vcstool\
    ros-humble-rviz2 \
    iputils-ping \
    iproute2  \
    python3-rosdep \
    && apt clean
RUN apt install -y \
    ros-humble-rmw-cyclonedds-cpp \
    && apt clean


RUN apt update
RUN apt upgrade -y

 

ENV DEBIAN_FRONTEND=dialog

# Create a new user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
RUN if id -u ${USER_UID} ; then userdel `id -un ${USER_UID}` ; fi
RUN groupadd --gid ${USER_GID} ${USERNAME} 
RUN useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && apt update \
    && apt install -y sudo git \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

#Change HOME environment variable
ENV HOME=/home/${USERNAME}
# Choose to run as user
ENV USER=${USERNAME}

USER ${USERNAME}

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# Install the python packages cosi non vengono installati da root
RUN pip3 install numpy 
# install gazebo ignition
# RUN sudo apt install -y  curl lsb-release gnupg
# RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpgecho "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
# RUN echo " CURL DONE"



# Set up auto-source of workspace for ros user
ARG WORKSPACE=THE_manipulation_ws
WORKDIR /home/${USERNAME}/${WORKSPACE}
RUN mkdir src
RUN mkdir src/Universal_Robots_ROS2_Driver

RUN git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver
# RUN vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver.humble.repos
RUN sudo apt-get update
RUN rosdep update 
RUN rosdep install --from-paths src -y --ignore-src


# install the UR library 

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

RUN echo "if [ -f ~/${WORKSPACE}/install/setup.bash ]; then source ~/${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
RUN echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib/" >> ~/.bashrc
RUN sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen


# rosdep realsense library 
RUN sudo apt install python3-rosdep -y
RUN sudo rm /etc/ros/rosdep/sources.list.d/20-default.list



# RUN sudo rosdep init
# RUN rosdep update 
# RUN echo "rosdep install -i --from-path /home/ros/${WORKSPACE}/src --rosdistro humble --skip-keys=librealsense2 -y" >> /home/ros/.bashrc
ENTRYPOINT ["/ros_entrypoint.sh"]
# RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
# RUN echo "source /home/ros/ros2_ws/install/setup.bash 2>/dev/null || true" >> ~/.bashrc