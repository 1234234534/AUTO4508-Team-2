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
    build-essential \
    make \
    g++ \
    cmake \
    doxygen \
    tmux \
    ros-jazzy-nmea-navsat-driver \
    ros-jazzy-depthai-ros \
    ros-jazzy-rqt-image-view \
    ros-jazzy-phidgets-spatial \
    ros-jazzy-robot-localization \
    ros-jazzy-pcl-conversions \
    ros-jazzy-pcl-ros \
    iproute2 \
    nmap \
    ros-jazzy-rviz2 \
    zathura \
    python3-opencv \
    ros-jazzy-cv-bridge \
    feh \
    ros-jazzy-foxglove-bridge \
    python3-numpy \
    python3-sklearn \
    python3-skimage \
    python3-joblib \
    ros-jazzy-rosbridge-suite \
    ros-jazzy-rosbag2 \
    && rm -rf /var/lib/apt/lists/*
    

# Install depthai
RUN pip3 install depthai --break-system-packages

WORKDIR /opt
RUN git clone https://github.com/reedhedges/AriaCoda.git /AriaCoda && cd /AriaCoda && make && make install

#WORKDIR /workspace/src
#RUN git clone https://github.com/RichbeamTechnology/Lakibeam_ROS2_Driver

# Set working directory
WORKDIR /workspace

CMD ["/bin/bash"]