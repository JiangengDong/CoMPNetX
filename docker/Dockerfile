FROM nvidia/cudagl:10.2-devel-ubuntu16.04

SHELL ["/bin/bash", "-c"]
WORKDIR /usr/local/src

# basic utils
RUN apt-get update && \
    apt-get install -y --no-install-recommends apt-utils sudo dialog wget lsb-release vim build-essential git unzip cmake software-properties-common && \
    apt-get autoremove -y && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/*

# install cudnn
ENV CUDNN_VERSION 7.6.5.32
LABEL com.nvidia.cudnn.version="${CUDNN_VERSION}"
RUN apt-get update && \
    apt-get install -y --no-install-recommends libcudnn7=$CUDNN_VERSION-1+cuda10.2 libcudnn7-dev=$CUDNN_VERSION-1+cuda10.2 && \
    apt-mark hold libcudnn7 && \
    apt-get autoremove -y && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/*

# install dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends software-properties-common && \
    apt-get autoremove -y && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/*
# Thanks to https://github.com/crigroup/openrave-installation
RUN apt-get update && \
    git clone --depth=1 https://github.com/crigroup/openrave-installation.git && \
    cd /usr/local/src/openrave-installation && \
    ./install-dependencies.sh && \
    cd /usr/local/src/openrave-installation && \
    ./install-osg.sh && \
    cd /usr/local/src/openrave-installation && \
    ./install-fcl.sh && \
    cd /usr/local/src/openrave-installation && \
    ./install-openrave.sh && \
    rm -rf /root/git && \
    apt-get autoremove -y && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/*

# install ompl
RUN apt-get update && \
    apt-get -y install g++ cmake pkg-config libboost-serialization-dev libboost-filesystem-dev libboost-system-dev \
    libboost-program-options-dev libboost-test-dev libeigen3-dev libode-dev wget libyaml-cpp-dev && \
    git clone --recursive --branch 1.4.2 --depth 1 https://github.com/ompl/ompl.git && \
    mkdir -p ompl/build/Release && \
    cd ompl/build/Release && \
    cmake ../.. -DPYTHON_EXEC=/usr/bin/python && \
    make install && \
    cd /usr/local/src && \
    rm -rf ompl && \
    apt-get autoremove -y && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/* && \
    ln -sf /usr/include/eigen3/Eigen /usr/include/Eigen

# install ROS
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends ros-kinetic-ros-base && \
    apt-get autoremove -y && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/* && \
    rosdep init
RUN apt-get update && \
    apt-get install -y --no-install-recommends ros-kinetic-urdfdom-py ros-kinetic-srdfdom liburdfdom-headers-dev ros-kinetic-urdf && \
    apt-get autoremove -y && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/*

# set locales
RUN apt-get update && \
    apt-get install -y --no-install-recommends python3-pip locales locales-all && \
    apt-get autoremove -y && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/*
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8 LANGUAGE=en_US:en LC_ALL=en_US.UTF-8
# install conda
RUN wget -O miniconda.sh https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh  && \
    chmod +x /usr/local/src/miniconda.sh && \
    /usr/local/src/miniconda.sh -b -p /opt/conda && \
    rm /usr/local/src/miniconda.sh && \
    /opt/conda/bin/conda install -y python=3 conda-build && \
    /opt/conda/bin/conda clean -ya && \
    chmod -R ugo+rw /opt/conda/
ENV PATH /opt/conda/bin:$PATH
# install libtorch
RUN wget -O libtorch.zip https://download.pytorch.org/libtorch/cu102/libtorch-cxx11-abi-shared-with-deps-1.8.1%2Bcu102.zip && \
    unzip libtorch.zip && \
    rm libtorch.zip && \
    cp -r libtorch/bin libtorch/include libtorch/lib libtorch/share /usr/local/ && \
    rm -rf libtorch

# add a user "compnetx" with password "compnetx"
RUN useradd -m compnetx && \
    echo "compnetx:compnetx" | chpasswd && \
    adduser compnetx sudo
# change user and workdir
USER compnetx
WORKDIR /home/compnetx

# install ROS packages
RUN mkdir -p /home/compnetx/catkin_ws/src
COPY --chown=compnetx catkin_ws/src/baxter_common /home/compnetx/catkin_ws/src/baxter_common
RUN rosdep update && \
    echo "source /opt/ros/kinetic/setup.bash" >> /home/compnetx/.bashrc && \
    echo "source /home/compnetx/catkin_ws/devel/setup.bash" >> /home/compnetx/.bashrc && \
    source /opt/ros/kinetic/setup.bash && \
    cd /home/compnetx/catkin_ws/src && \
    git clone --depth 1 --branch kinetic-devel https://github.com/ros-planning/srdfdom.git && \
    git clone --depth 1 --branch kinetic_fixes https://github.com/personalrobotics/or_urdf.git && \
    git clone --depth 1 https://github.com/personalrobotics/openrave_catkin.git && \
    git clone --depth 1 https://github.com/ros/urdfdom.git && \
    cd /home/compnetx/catkin_ws && \
    catkin_make
# install pytorch
RUN conda init 
RUN pip3 install --no-cache-dir torch torchvision torchaudio