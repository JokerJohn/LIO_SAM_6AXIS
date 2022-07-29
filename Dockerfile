# check more detail on: https://hub.docker.com/r/nvidia/cuda
FROM nvidia/cuda:10.2-devel-ubuntu18.04
LABEL maintainer="Kin Zhang <kin_eng@163.com>"

# Just in case we need it
ENV DEBIAN_FRONTEND noninteractive

# basic elements
RUN apt update && apt install -y --no-install-recommends git curl vim rsync ssh wget zsh tmux g++

# ==========> INSTALL zsh <=============
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.2/zsh-in-docker.sh)" -- \
    -t robbyrussell \
    -p git \
    -p ssh-agent \
    -p https://github.com/agkozak/zsh-z \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-completions \
    -p https://github.com/zsh-users/zsh-syntax-highlighting

# ==========> INSTALL ROS melodic <=============
RUN apt update && apt install -y curl lsb-release
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt update && apt install -y ros-melodic-desktop-full
RUN apt-get install -y libgtest-dev ros-melodic-catkin python-pip python3-pip

RUN echo "source /opt/ros/melodic/setup.zsh" >> ~/.zshrc
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

# needs to be done before we can apply the patches
RUN git config --global user.email "xxx@163.com"
RUN git config --global user.name "kin-docker"

# ===========> Install Packages <==============
# Install C++ Dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    libblosc-dev \
    libboost-iostreams-dev \
    libboost-numpy-dev \
    libboost-python-dev \
    libboost-system-dev \
    libeigen3-dev \
    libtbb-dev \
    libgflags-dev \
    libgl1-mesa-glx \
    libgoogle-glog-dev \
    protobuf-compiler \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# ======================> DEPENDENCIES <=========================
# gtsam 4.0.2 --> https://github.com/borglab/gtsam/issues/145
RUN git clone https://github.com/borglab/gtsam.git \
    && cd gtsam && git checkout b10963802c13893611d5a88894879bed47adf9e0 \
    && mkdir build && cd build && cmake .. && make -j$(nproc) && make install

# resolved the conflict ========> see issue: https://github.com/ethz-asl/lidar_align/issues/16#issuecomment-504348488
RUN mv /usr/include/flann/ext/lz4.h /usr/include/flann/ext/lz4.h.bak  && \
    mv /usr/include/flann/ext/lz4hc.h /usr/include/flann/ext/lz4.h.bak && \
    ln -s /usr/include/lz4.h /usr/include/flann/ext/lz4.h && \
    ln -s /usr/include/lz4hc.h /usr/include/flann/ext/lz4hc.h

RUN mkdir -p /root/workspace/src && mkdir -p /home/xchu/data/ramlab_dataset
WORKDIR /root/workspace
RUN cd src && git clone https://github.com/JokerJohn/LIO_SAM_6AXIS.git

