# Image Variables
ARG L4T_MAJOR="36"
ARG L4T_MINOR="3"
ARG L4T_PATCH="0"
ARG L4T_BASE="l4t-jetpack"

# Base Image
FROM nvcr.io/nvidia/${L4T_BASE}:r${L4T_MAJOR}.${L4T_MINOR}.${L4T_PATCH}

# Install Variables
ARG L4T_MAJOR="36"
ARG L4T_MINOR="3"
ARG L4T_PATCH="0"
ARG L4T_BASE="l4t-jetpack"
 
# Set Non-Interactive Mode
ARG DEBIAN_FRONTEND=noninteractive

# Set Jetson Streaming Evironment Variables
ENV LOGNAME root

# Set L4T Version
RUN echo "# R${L4T_MAJOR} (release), REVISION: ${L4T_MINOR}.${L4T_PATCH}" > /etc/nv_tegra_release

# Clean APT Cache
RUN rm /var/lib/dpkg/info/libc-bin.*
# Add APT Repo for PCIe drivers and Bazel.
RUN apt update && apt install -y wget && \
    echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | tee /etc/apt/sources.list.d/coral-edgetpu.list && \
    wget -q -O - https://packages.cloud.google.com/apt/doc/apt-key.gpg | apt-key add -

# Install Required Ubuntu Packages
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential gfortran cmake git gdb file tar libatlas-base-dev apt-transport-https iputils-ping \
    libswresample-dev libcanberra-gtk3-module zstd less libx264-dev libdrm-dev python-is-python3 \
    libeigen3-dev libglew-dev libgstreamer-plugins-base1.0-dev udev net-tools libssl-dev \
    libgstreamer-plugins-good1.0-dev libgstreamer1.0-dev libgtk-3-dev libjpeg-dev sudo usbutils \
    libjpeg8-dev libjpeg-turbo8-dev liblapack-dev liblapacke-dev libopenblas-dev libpng-dev tzdata \
    libpostproc-dev libtbb-dev libtbb2 libtesseract-dev libtiff-dev libv4l-dev \
    libxine2-dev libxvidcore-dev libx264-dev libgtkglext1 libgtkglext1-dev pkg-config qv4l2 \
    v4l-utils zlib1g-dev python3-dev libboost-all-dev valgrind doxygen graphviz nano \
    vim-common libedgetpu1-std gasket-dkms ca-certificates nlohmann-json3-dev curl \
    python3-dev python3-pip python3-numpy libaom-dev libass-dev libfdk-aac-dev libdav1d-dev libmp3lame-dev \
    libopus-dev libvorbis-dev libvpx-dev libx264-dev libx265-dev

# Nice to have
RUN apt-get update && apt-get install --no-install-recommends -y bat \
    bash-completion fish git-lfs

# Install Required Python Packages.
RUN python -m pip install numpy opencv-python pyopengl matplotlib

# This symbolic link is needed to use the streaming features on Jetson inside a container
RUN ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 /usr/lib/aarch64-linux-gnu/libv4l2.so

# Set Timezone
ARG TZ="America/Chicago"
RUN echo "${TZ}" > /etc/localtime && \
    echo "${TZ}" > /etc/timezone

# Install gcc/g++
RUN apt-get install  --no-install-recommends -y \
    gcc-10 g++-10 && \
    rm -rf /var/lib/apt/lists/* && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 100 --slave /usr/bin/g++ g++ /usr/bin/g++-10 --slave /usr/bin/gcov gcov /usr/bin/gcov-10

# Install CMake
ARG CMAKE_VERSION="3.30.2"
COPY install-cmake.sh /tmp/
RUN if [ "${CMAKE_VERSION}" != "none" ]; then \
    chmod +x /tmp/install-cmake.sh && /tmp/install-cmake.sh ${CMAKE_VERSION}; \
    fi && \
    rm -f /tmp/install-cmake.sh

# Set Working Directory
WORKDIR /opt

# Install ZED SDK
ARG ZED_MAJOR="4"
ARG ZED_MINOR="2"
RUN wget -q --no-check-certificate -O ZED_SDK_Linux.run \
    https://download.stereolabs.com/zedsdk/${ZED_MAJOR}.${ZED_MINOR}/l4t${L4T_MAJOR}.${L4T_MINOR}/jetsons && \
    chmod +x ZED_SDK_Linux.run ; ./ZED_SDK_Linux.run silent && \
    rm -rf /usr/local/zed/resources/* \
    rm -rf ZED_SDK_Linux.run && \
    rm -rf /var/lib/apt/lists/* && \
    mkdir -p /root/Documents/ZED/ && \
    sed -i '/#pragma message*/d' /usr/local/zed/include/sl/Fusion.hpp && \
    sed -i '/#pragma message*/d' /usr/local/zed/include/sl/Camera.hpp && sed -i '/#warning*/d' /usr/local/zed/include/sl/Camera.hpp

# Install OpenCV
ARG OPENCV_VERSION="4.11.0"
RUN wget -q https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/opencv/arm64/opencv_${OPENCV_VERSION}_arm64.deb && \
    dpkg -i opencv_${OPENCV_VERSION}_arm64.deb && \
    rm opencv_${OPENCV_VERSION}_arm64.deb

# Install PyTorch.
ARG TORCH_VERSION="2.2.2"
RUN wget -q https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/pytorch/arm64/pytorch_${TORCH_VERSION}_arm64.deb && \
    dpkg -i pytorch_${TORCH_VERSION}_arm64.deb && \
    rm pytorch_${TORCH_VERSION}_arm64.deb

# Install Abseil.
ARG ABSEIL_VERSION="20230802.1"
RUN wget -q https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/abseil/arm64/abseil_${ABSEIL_VERSION}_arm64.deb && \
    dpkg -i abseil_${ABSEIL_VERSION}_arm64.deb && \
    rm abseil_${ABSEIL_VERSION}_arm64.deb

# Install GeographicLib
ARG GEOLIB_VERSION="2.3"
RUN wget -q https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/geolib/arm64/geolib_${GEOLIB_VERSION}_arm64.deb && \
    dpkg -i geolib_${GEOLIB_VERSION}_arm64.deb && \
    rm geolib_${GEOLIB_VERSION}_arm64.deb

# Install FFMPEG
ARG FFMPEG_VERSION="7.1"
RUN wget -q https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/ffmpeg/arm64/ffmpeg_${FFMPEG_VERSION}_arm64.deb && \
    dpkg -i ffmpeg_${FFMPEG_VERSION}_arm64.deb && \
    rm ffmpeg_${FFMPEG_VERSION}_arm64.deb

# Install Libdatachannel
ARG LIBDATACHANNEL_VERSION="0.22"
RUN wget -q https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/libdatachannel/arm64/libdatachannel_${LIBDATACHANNEL_VERSION}_arm64.deb && \
    dpkg -i libdatachannel_${LIBDATACHANNEL_VERSION}_arm64.deb && \
    rm libdatachannel_${LIBDATACHANNEL_VERSION}_arm64.deb

# Install Tensorflow.
ARG TENSORFLOW_VERSION="2.15.0"
RUN wget -q https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/tensorflow/arm64/tensorflow_${TENSORFLOW_VERSION}_arm64.deb && \
    dpkg -i tensorflow_${TENSORFLOW_VERSION}_arm64.deb && \
    rm tensorflow_${TENSORFLOW_VERSION}_arm64.deb

# Install Quill
ARG QUILL_VERSION="8.1.0"
RUN wget -q https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/quill/arm64/quill_${QUILL_VERSION}_arm64.deb && \
    dpkg -i quill_${QUILL_VERSION}_arm64.deb && \
    rm quill_${QUILL_VERSION}_arm64.deb

# Install Google Test
ARG GTEST_VERSION="1.16.0"
RUN wget -q https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/gtest/arm64/gtest_${GTEST_VERSION}_arm64.deb && \
    dpkg -i gtest_${GTEST_VERSION}_arm64.deb && \
    rm gtest_${GTEST_VERSION}_arm64.deb

# Enable Make Threads
RUN echo 'export MAKEFLAGS=-j$(($(grep -c "^processor" /proc/cpuinfo) - 1))' >> .bashrc

# Set Fish as Default Shell
RUN chsh -s /usr/bin/fish && mkdir -p ~/.config/fish/ && echo 'set fish_greeting' >> ~/.config/fish/config.fish

# Clone Autonomy Software Repository
RUN git clone --recurse-submodules -j8 https://github.com/MissouriMRDT/Autonomy_Software.git /opt/Autonomy_Software

# Disable the VSCode server requirements check, this fixes the cross architecture issues and potentially fixes mismatching VSCode server versions. Can be unstable.
RUN touch /tmp/vscode-skip-server-requirements-check

# Set Working Directory
WORKDIR /opt/Autonomy_Software/

# Set Labels
LABEL authors="Missouri S&T Mars Rover Design Team"
LABEL maintainer="Mars Rover Design Team <marsrover@mst.edu>"
LABEL org.opencontainers.image.source=https://github.com/missourimrdt/autonomy_software
LABEL org.opencontainers.image.licenses=GPL-3.0-only
LABEL org.opencontainers.image.version="v24.5.0"
LABEL org.opencontainers.image.description="Docker Image for ${L4T_BASE} ${L4T_MAJOR}.${L4T_MINOR}.${L4T_PATCH} with CUDA ${CUDA_MAJOR}.${CUDA_MINOR}, ZED SDK ${ZED_MAJOR}.${ZED_MINOR}, OpenCV ${OPENCV_VERSION}, Quill ${QUILL_VERSION} and Google Test ${GTEST_VERSION}."
