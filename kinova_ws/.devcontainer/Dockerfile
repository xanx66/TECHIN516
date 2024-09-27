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

# Install Git and other basic tools
RUN sudo apt install -y git vim tmux

# Rosdep update
RUN rosdep update

# Source the ROS setup files
RUN echo "" >> ~/.bashrc \
    && echo "# ros2 stuff" >> ~/.bashrc \
    && echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "source /kinova_ws/install/setup.bash" >> ~/.bashrc \
    && echo ". /usr/share/gazebo/setup.sh" >> ~/.bashrc

# helpful tools
RUN sudo apt update && sudo apt install -y \
    ros-humble-tf2-tools \
    ros-humble-rqt-*

# gazebo
RUN sudo apt update && sudo apt install -y \
    ros-humble-gazebo-*

CMD source ~/.bashrc
