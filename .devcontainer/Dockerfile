FROM sunside/ros-gazebo-gpu:kinetic-nvidia

USER root

RUN apt-get update && \
    apt-get install -y locales && \
    echo "en_US.UTF-8 UTF-8" >> /etc/locale.gen && \
    locale-gen

## This stuff is needed for running the Rotors simulator
# RUN apt-get update && \
#     apt-get install -y python-pip && \
#     pip install future pygame==2.0.3

# RUN git clone https://github.com/devbharat/python-uinput.git && \
#     cd python-uinput && \
#     python setup.py build && \
#     sudo python setup.py install && \
#     cd udev-rules && \
#     mkdir -p /etc/udev/rules.d/ && \
#     sudo cp 40-uinput.rules /etc/udev/rules.d/ && \
#     cd ../.. && \
#     rm -rf python-uinput


RUN sudo addgroup uinput && \
    sudo adduser ros uinput && \
    sudo usermod -a -G dialout ros

RUN apt-get install -y --allow-unauthenticated  \ 
    ros-kinetic-desktop-full  \
    ros-kinetic-joy  \
    ros-kinetic-octomap-ros  \
    ros-kinetic-mavlink  \
    python-wstool  \
    python-catkin-tools  \
    protobuf-compiler  \
    libgoogle-glog-dev  \
    ros-kinetic-control-toolbox  \
    ros-kinetic-mavros  \
    autoconf \
    libignition-math2-dev \
    # Needed for DJI onboard sdk
    ros-kinetic-nmea-comms \
    ffmpeg \
    libsdl2-dev \ 
    # Gives add-apt-repository, which is needed for gcc-8/g++-8
    software-properties-common \
    # Needed for AirSim
    ros-kinetic-tf2-sensor-msgs \
    ros-kinetic-tf2-geometry-msgs \
    ros-kinetic-mavros*

# RUN git clone https://github.com/dji-sdk/Onboard-SDK.git && \
#     cd Onboard-SDK && \
#     # git checkout 3.8.0 && \
#     mkdir build && cd build && \
#     cmake .. && \
#     make -j4 install && \
#     cd ../.. && \
#     rm -rf Onboard-SDK 

# g++-8 is required for AirSim
RUN add-apt-repository -y ppa:ubuntu-toolchain-r/test && \
    apt-get update && \
    apt-get install -y gcc-8 g++-8 && \
    # Set the default to gcc-8/g++-8
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-8 8 && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 8 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-5 5 && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 5


# CMake >= 3.10.0 is required for AirSim
RUN wget https://github.com/Kitware/CMake/releases/download/v3.23.0-rc2/cmake-3.23.0-rc2-linux-x86_64.sh && \ 
    sudo sh ./cmake-3.23.0-rc2-linux-x86_64.sh --prefix=/usr --skip-license

RUN sudo add-apt-repository -y ppa:borglab/gtsam-release-4.0 && \
    sudo apt-get update && \
    sudo apt-get install -y libgtsam-dev libgtsam-unstable-dev

# # Install Eigen 3.3.7, overwriting header files in the apt package libeigen3-dev, which is version 3.2.92
# RUN wget https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz && \
#     tar xvf eigen-3.3.7.tar.gz && \
#     cd eigen-3.3.7 && \
#     mkdir build && \
#     cd build && \
#     cmake -DCMAKE_INSTALL_PREFIX=/usr .. && \
#     make install && \
#     cd ../.. && \
#     rm -rf eigen-3.3.7 && \
#     rm eigen-3.3.7.tar.gz


# # Instal GTSAM, which is a dependency of aprilslam
# RUN mkdir deps && \
#     cd deps && \
#     wget https://github.com/borglab/gtsam/archive/refs/tags/4.1.0.tar.gz -O gtsam.tgz && \
#     tar -xvf gtsam.tgz && \
#     cd gtsam* && \
#     mkdir build && \
#     cd build && \
#     # Use the system Eigen. Since ROS also uses system Eigen, everything will use same Eigen version
#     # (using different eigen versions can cause problems)
#     cmake -DGTSAM_USE_SYSTEM_EIGEN=ON .. && \
#     make install


RUN mkdir -p /etc/udev/rules.d/ && \
    echo 'SUBSYSTEM =="usb", ATTRS {idVendor} =="2ca3", MODE ="0666"' >> /etc/udev/rules.d/DJIDevice.rules

ENV EDITOR code
USER ros

COPY UserConfig.txt /home/ros/.ros/UserConfig.txt
# RUN rosdep install --from-paths ../src --ignore-src -r -y
