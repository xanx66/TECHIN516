FROM osrf/ros:humble-desktop-full

# Add vscode user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install Git
RUN sudo apt install -y git vim tmux

# Rosdep update
RUN rosdep update

# Source the ROS setup files
RUN echo "" >> ~/.bashrc \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "source /colcon_ws/install.setup.bash" >> ~/.bashrc \
    && echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc \ 
    && echo ". /usr/share/gazebo/setup.sh" >> ~/.bashrc

# helpful tools
RUN sudo apt update && sudo apt install -y \
    ros-humble-tf2-tools \
    ros-humble-rqt-*

# gazebo
RUN sudo apt update && sudo apt install -y \
    ros-humble-gazebo-*

# turtlbot dependencies
RUN sudo apt update && sudo apt install -y \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup

# turtlebot packages
RUN sudo apt update && sudo apt install -y \
    ros-humble-dynamixel-sdk \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3-simulations \
    ros-humble-turtlebot3

CMD source ~/.bashrc