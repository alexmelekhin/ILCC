FROM nvidia/opengl:1.2-glvnd-runtime-ubuntu16.04

ENV DEBIAN_FRONTEND noninteractive

ARG NUM_THREADS=1

WORKDIR /usr/local/src

# Install dependencies
RUN apt-get update && apt-get install -y \
    clang \
    cmake \
    g++ \
    git \
    vim \
    sudo \
    python-pip \
    python-tk \
    libopencv-dev \
    libpcl-dev \
    lsb-release && \
    rm -rf /var/lib/apt/lists/*

# Install ROS1 Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && apt-get install --no-install-recommends -y \
        ros-kinetic-desktop \
        ros-kinetic-cv-bridge \
        ros-kinetic-image-transport-plugins \
        python-rosdep \
        python-rosinstall \
        python-rosinstall-generator \
        python-wstool && \
    rosdep init && \
    rosdep update

# Upgrade pip
RUN pip install --upgrade pip==20.3.4

# Install python dependencies
RUN pip install \
    tqdm==4.64.1 \
    numpy==1.16.6 \
    pandas==0.24.2 \
    matplotlib==2.2.5 \
    opencv-python==3.4.9.31 \
    pathlib==1.0.1 \
    pathlib2==2.3.7.post1 \
    transforms3d==0.3.1 \
    pyopen==0.0.5 \
    scipy==1.2.3 \
    scikit-learn==0.20.4 \
    python-pcl==0.3.0a1
#    PyYAML==5.4.1

# Build and install OpenGV
ARG OPENGV_COMMIT=91f4b19c73450833a40e463ad3648aae80b3a7f3
RUN set -x && \
    git clone --recurse-submodules https://github.com/laurentkneip/opengv.git && \
    cd opengv && \
    git config --global --add safe.directory /usr/local/src/opengv && \
    git checkout ${OPENGV_COMMIT} && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON=ON -DPYBIND11_PYTHON_VERSION=2.7 && \
    make -j${NUM_THREADS} && \
    make install && \
    rm -rf /usr/local/src/opengv

# User setup
ARG USER=docker_ilcc
ARG UID=1000
ARG GID=1000
ARG PW=user

# Add user and his password
RUN useradd -m -G sudo ${USER} --uid=${UID} && echo "${USER}:${PW}" | chpasswd

WORKDIR /home/${USER}
USER ${UID}:${GID}
