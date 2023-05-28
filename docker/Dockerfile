FROM nvidia/cuda:11.3.0-cudnn8-devel-ubuntu20.04

# Just in case we need it
ENV DEBIAN_FRONTEND noninteractive

RUN apt update && apt install -y --no-install-recommends git curl wget \
      git tmux vim g++ net-tools ipuils-ping openssh-server

# ==========> INSTALL ROS noetic <=============
RUN apt update && apt install -y curl lsb-release
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -

RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" | tee /etc/apt/sources.list.d/ros-focal.list

RUN apt update && apt install -y ros-noetic-desktop ros-noetic-ros-numpy \
	ros-noetic-vision-msgs ros-noetic-catkin python3-catkin-tools \
	python3-rosdep python3-rosinstall python3-rosinstall-generator \
	python3-wstool build-essential -y

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# =========> INSTALL packages <=============
RUN apt-get update \
 && apt-get -V install -y \
      build-essential clang clang-tidy libbenchmark-dev \
      libblas-dev libboost-serialization-dev libboost-filesystem-dev \
      libboost-iostreams-dev libboost-system-dev libflann-dev libglew-dev \
      libgtest-dev libopenni-dev libopenni2-dev libpcap-dev libproj-dev \
      libqhull-dev libqt5opengl5-dev libusb-1.0-0-dev lsb-release wget \
      python3-pip python3-setuptools libpcl-dev ros-noetic-octomap ros-noetic-octomap-ros \
      ros-noetic-eigen-conversions ros-noetic-tf-conversions ros-noetic-tf2-bullet\
      unzip git ros-noetic-grid-map-core ros-noetic-grid-map-ros  \
      ros-noetic-grid-map-rviz-plugin ros-noetic-rviz ros-noetic-pcl-ros

# =========> INSTALL GTSAM <=============
RUN git clone https://github.com/borglab/gtsam.git && cd gtsam \
      && git checkout d304358deeaa4625cf24a8e0d94145bb3435d5bc && mkdir build && cd build \
      && cmake .. && make install && cp /usr/local/lib/libmetis-gtsam.so /usr/lib

# =========> INSTALL kindr <=============
RUN wget -O kindr.tar.gz https://github.com/ANYbotics/kindr/archive/refs/tags/1.2.0.tar.gz \
    && tar -xvzf kindr.tar.gz && cd kindr-1.2.0 && mkdir build && cd build \
    && cmake .. && make install -j8 && cd / && rm kindr.tar.gz && rm -r kindr-1.2.0/

# =========> INSTALL fftw <=============
RUN wget -O fftw.tar.gz http://fftw.org/fftw-3.3.10.tar.gz && tar -xvzf fftw.tar.gz \
    && cd fftw-3.3.10 && mkdir build && cd build && cmake .. \
    && make install -j8 && cd / && rm fftw.tar.gz && rm -r fftw-3.3.10/

RUN pip3 config set global.index-url http://pypi.douban.com/simple
RUN pip3 config set install.trusted-host pypi.douban.com

RUN pip3 install torch==1.11.0+cu113 torchvision==0.12.0+cu113 torchaudio==0.11.0 --extra-index-url https://download.pytorch.org/whl/cu113
RUN pip3 install scipy open3d scikit-image alpha-transform

COPY Mapping /home/Mapping
COPY LoopDetection /home/LoopDetection

SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/noetic/setup.bash && cp /usr/local/lib/libmetis-gtsam.so /usr/lib && cd /home/Mapping && catkin_make -DBUILD_PYTHON_BINDINGS=ON
RUN source /opt/ros/noetic/setup.bash && cd /home/LoopDetection && catkin_make -DBUILD_PYTHON_BINDINGS=ON

