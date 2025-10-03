# syntax=docker/dockerfile:1.4
ARG ROS_DISTRO=humble
ARG VARIANT=ros-core
FROM ros:${ROS_DISTRO}-${VARIANT}

ARG ROS_DISTRO
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Install base tooling and ROS development dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    locales \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    gnupg2 \
    lsb-release \
    rsync \
    bash-completion \
    python3-pip \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-diagnostic-aggregator \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    libpcl-dev \
    libqhull-dev \
    libqhull8.0 \
    && rm -rf /var/lib/apt/lists/*

# Configure locale
RUN locale-gen en_US.UTF-8 && update-locale LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Create non-root user
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# Install Python tooling and utilities
RUN pip3 install --no-cache-dir \
    numpy \
    scipy \
    matplotlib \
    opencv-python \
    pillow \
    psutil \
    pandas \
    black \
    isort \
    flake8 \
    pylint \
    pytest

# Prepare rosdep
RUN rosdep init || true

# Set up workspace directory
RUN mkdir -p /workspaces/go2_ros2_sdk

# Bash defaults for the user
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "export ROS_DOMAIN_ID=42" >> /home/${USERNAME}/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/${USERNAME}/.bashrc && \
    echo "alias cb='colcon build'" >> /home/${USERNAME}/.bashrc && \
    echo "alias cbt='colcon test'" >> /home/${USERNAME}/.bashrc && \
    echo "alias source-ws='[ -f install/setup.bash ] && source install/setup.bash'" >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /workspaces/go2_ros2_sdk

# Default shell
SHELL ["/bin/bash", "-c"]

CMD ["sleep", "infinity"]
