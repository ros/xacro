# Set base image
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

# Arguments
ARG GID=1000
ARG UID=1000

# Install some dependencies packages
RUN apt update -q \
 && apt upgrade -q -y \
 && apt install -y gdb python3-pip software-properties-common sudo --no-install-recommends \
 && apt clean \
 && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Add the 'dev' user
RUN groupadd -g ${GID} dev \
 && useradd -lm -u ${UID} -g dev -s /bin/bash dev \
 && echo "dev ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER dev

# Create workspace so that user 'dev' own this directory
RUN mkdir -p /home/dev/ros_ws/src \
 && echo 'source /opt/ros/'${ROS_DISTRO}'/setup.bash' >> /home/dev/.bashrc \
 && echo 'source /home/dev/ros_ws/install/setup.bash' >> /home/dev/.bashrc

WORKDIR /home/dev/ros_ws
CMD ["bash"]
