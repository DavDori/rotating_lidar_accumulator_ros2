FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    gdb \
    ccache \
    sudo \
    libeigen3-dev \
    libpcl-dev \
    libboost-all-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libceres-dev \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-tf2-eigen \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Non-root user with passwordless sudo
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash" >> /home/$USERNAME/.bashrc

COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER $USERNAME
WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]