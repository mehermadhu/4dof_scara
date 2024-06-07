FROM ros:humble

LABEL title="ROS2 Humble Docker Image"
LABEL authors="Meher M D <meherfreelance24@gmail.com>"
LABEL description="A Docker Image for developing applications based on the ROS2"
LABEL version="v0.1"

# Set non-interactive frontend (avoid some prompts)
ENV DEBIAN_FRONTEND noninteractive

RUN apt update \
    && apt upgrade -y \
    && apt install -y \
    sudo \
    software-properties-common \
    locales \ 
    curl \
    python3 \
    python3-pip \
    bash-completion

# Set locale. By default, the docker has a locale set to POSIX, and it is 
# necessary to have UTF-8 support.
RUN locale-gen en_US en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8

ENV LANG en_US.UTF-8 

ENV LC_ALL en_US.UTF-8

# Add the ROS2 apt
RUN add-apt-repository universe

# Install additional ROS2 packages and utilities
RUN apt update && apt install -y \
    ros-humble-rviz2 \
    ros-humble-rqt* \
    python3-rosdep \
    python3-colcon-common-extensions \
    can-utils \
    nano \
    && rm -rf /var/lib/apt/lists/* 

# Add a non-root user for ROS2 operations
RUN addgroup --gid 1000 ros2_user \
    && adduser --uid 1000 --gid 1000 --disabled-password --gecos "" ros2_user \
    && echo 'ros2_user ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers \
    && chmod -R 775 /home/ros2_user \
    && chown -R ros2_user:ros2_user /home/ros2_user

USER ros2_user
RUN rosdep update
# Set up the ROS2 environment by default
RUN echo "source /opt/ros/humble/setup.bash" >> /home/ros2_user/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/ros2_user/.bashrc

# Set directory to avoid runtime warning in GUI tools like RViz2
ENV XDG_RUNTIME_DIR=/tmp/runtime-ros2_user

# Prepare workspace
RUN mkdir -p /home/ros2_user/ws_ros/src
COPY ./src /home/ros2_user/ws_ros/src
# Set workspace directory
WORKDIR /home/ros2_user/ws_ros