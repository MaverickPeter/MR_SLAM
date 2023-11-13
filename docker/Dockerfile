FROM nvidia/cuda:11.1.1-cudnn8-devel-ubuntu20.04

# Just in case we need it
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends git curl wget \
      git tmux vim g++ net-tools iputils-ping openssh-server

# ==========> INSTALL ROS noetic <=============
RUN apt-get update && apt-get install -y curl lsb-release
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -

RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" | tee /etc/apt/sources.list.d/ros-focal.list

RUN apt-get update && apt-get install -y ros-noetic-desktop ros-noetic-ros-numpy \
	ros-noetic-vision-msgs ros-noetic-catkin python3-catkin-tools \
	python3-rosdep python3-rosinstall python3-rosinstall-generator \
	python3-wstool build-essential -y

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# =========> INSTALL packages <=============
RUN apt-get update \
 && apt-get -V install -y \
      build-essential clang clang-tidy libbenchmark-dev libgoogle-glog-dev \
      libblas-dev libboost-serialization-dev libboost-filesystem-dev \
      libboost-iostreams-dev libboost-system-dev libflann-dev libglew-dev \
      libgtest-dev libopenni-dev libopenni2-dev libpcap-dev libproj-dev \
      libqhull-dev libqt5opengl5-dev libusb-1.0-0-dev lsb-release wget \
      python3-pip python3-setuptools libpcl-dev ros-noetic-octomap ros-noetic-octomap-ros \
      ros-noetic-eigen-conversions ros-noetic-tf-conversions ros-noetic-tf2-bullet\
      unzip git ros-noetic-grid-map-core ros-noetic-grid-map-ros  \
      ros-noetic-grid-map-rviz-plugin ros-noetic-rviz ros-noetic-pcl-ros \
      ros-noetic-costmap-2d ros-noetic-navfn ros-noetic-base-local-planner \
      ros-noetic-clear-costmap-recovery ros-noetic-move-base-msgs \
      ros-noetic-rotate-recovery 

# =========> INSTALL GTSAM <============= 
RUN git clone https://github.com/borglab/gtsam.git -b 4.0.0-alpha2 && cd gtsam \
      && mkdir build && cd build && cmake .. && make install \
      && cd / && rm -r gtsam

# =========> INSTALL kindr <=============
RUN wget -O kindr.tar.gz https://github.com/ANYbotics/kindr/archive/refs/tags/1.2.0.tar.gz \
    && tar -xvzf kindr.tar.gz && cd kindr-1.2.0 && mkdir build && cd build \
    && cmake .. && make install -j8 && cd / && rm kindr.tar.gz && rm -r kindr-1.2.0/

# =========> INSTALL fftw <=============
RUN wget -O fftw.tar.gz http://fftw.org/fftw-3.3.10.tar.gz && tar -xvzf fftw.tar.gz \
    && cd fftw-3.3.10 && mkdir build && cd build && cmake .. \
    && make install -j8 && cd / && rm fftw.tar.gz && rm -r fftw-3.3.10/

# =========> INSTALL ceres <=============
RUN wget -O ceres.tar.gz https://github.com/ceres-solver/ceres-solver/archive/refs/tags/1.14.0.tar.gz && tar -xvzf ceres.tar.gz \
    && cd ceres-solver-1.14.0 && mkdir build && cd build && cmake .. \
    && make install -j8 && cd / && rm ceres.tar.gz && rm -r ceres-solver-1.14.0/

RUN pip3 config set global.index-url http://pypi.douban.com/simple
RUN pip3 config set install.trusted-host pypi.douban.com

RUN pip3 install torch==1.10.1+cu111 torchvision==0.11.2+cu111 torchaudio==0.10.1 -f https://download.pytorch.org/whl/cu111/torch_stable.html
RUN pip3 install scipy open3d scikit-image alpha-transform

COPY Mapping /home/Mapping
COPY LoopDetection /home/LoopDetection
COPY Localization /home/Localization
COPY Visualization /home/Visualization
COPY Costmap /home/Costmap
COPY Tools /home/Tools

SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/noetic/setup.bash && cd /home/Localization && catkin_make \
      && source /opt/ros/noetic/setup.bash && cd /home/Localization && catkin_make \
      && cd /home/Mapping && catkin_make -DBUILD_PYTHON_BINDINGS=ON && cd src/fast_gicp && python3 setup.py install \
      && source /opt/ros/noetic/setup.bash && cd /home/LoopDetection && catkin_make -DBUILD_PYTHON_BINDINGS=ON \
      && cd generate_bev_cython_binary && python3 setup.py install && cd .. && cd torch-radon && python3 setup.py install \
      && cd .. && cd generate_bev_pointfeat_cython && python3 setup.py install \
      && cd /home/LoopDetection/src/disco_ros/tools/multi-layer-polar-gpu/cython && python3 setup.py install \
      && cd /home/Costmap && catkin_make


