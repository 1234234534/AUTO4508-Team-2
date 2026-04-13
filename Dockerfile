FROM ros:jazzy

# Auto-source ROS2 for every shell
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc

# Install dependencies
RUN apt-get update && apt-get install -y \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-joy \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-sick-scan-xd \
    python3-pip \
    git \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install depthai
RUN pip3 install depthai --break-system-packages

WORKDIR /opt
RUN git clone https://github.com/reedhedges/Aria.git && cd Aria && make
RUN git clone https://github.com/reedhedges/ArNetworking.git && cd ArNetworking && make

# Set working directory
WORKDIR /workspace

CMD ["/bin/bash"]
