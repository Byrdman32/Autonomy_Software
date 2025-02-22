# Image Variables
ARG UBUNTU_MAJOR="22"
ARG CUDA_MAJOR="12"
ARG CUDA_MINOR="2"
ARG CUDA_PATCH="2"
ARG CUDNN_VERSION="8"

# Base Image
FROM nvcr.io/nvidia/cuda:${CUDA_MAJOR}.${CUDA_MINOR}.${CUDA_PATCH}-cudnn${CUDNN_VERSION}-devel-ubuntu${UBUNTU_MAJOR}.04

# Install Variables
ARG UBUNTU_MAJOR="22"
ARG CUDA_MAJOR="12"
ARG CUDA_MINOR="2"
ARG CUDA_PATCH="2"
ARG CUDNN_VERSION="8"

# Set Non-Interactive Mode
ARG DEBIAN_FRONTEND=noninteractive

# Set NVIDIA Driver Enviroment Variables
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}compute,video,utility,graphics

# Set Timezone
ARG TZ="America/Chicago"
RUN echo "${TZ}" > /etc/localtime && \
    echo "${TZ}" > /etc/timezone

# Set CUDA Version
RUN echo "CUDA Version ${CUDA_MAJOR}.${CUDA_MINOR}.${CUDA_PATCH}" > /usr/local/cuda/version.txt

# Add APT Repo for PCIe drivers.
RUN apt update && apt install -y wget gnupg && \
    echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | tee /etc/apt/sources.list.d/coral-edgetpu.list && \
    wget -qO - https://packages.cloud.google.com/apt/doc/apt-key.gpg | apt-key add - && \
    wget -qO - https://bazel.build/bazel-release.pub.gpg | gpg --dearmor > bazel-archive-keyring.gpg && \
    mv bazel-archive-keyring.gpg /usr/share/keyrings && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/bazel-archive-keyring.gpg] https://storage.googleapis.com/bazel-apt stable jdk1.8" | tee /etc/apt/sources.list.d/bazel.list

# Install Required Ubuntu Packages
RUN apt-get update && apt-get install --no-install-recommends -y iputils-ping \
    build-essential gdb less udev zstd sudo libgomp1 python-is-python3 \
    cmake git libgtk2.0-dev pkg-config libx264-dev libdrm-dev ssh \
    libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev tzdata net-tools \
    yasm libatlas-base-dev gfortran libpq-dev libpostproc-dev \
    libxine2-dev libglew-dev libtiff5-dev zlib1g-dev cowsay lolcat locales usbutils \
    libeigen3-dev python3-dev python3-pip python3-numpy libx11-dev xauth libssl-dev \
    valgrind doxygen graphviz htop nano fortune fortunes \
    vim-common gasket-dkms nlohmann-json3-dev gcovr lcov curl \
    libaom-dev libass-dev libfdk-aac-dev libdav1d-dev libmp3lame-dev \
    libopus-dev libvorbis-dev libvpx-dev libx264-dev libx265-dev

# Nice to have
RUN apt-get update && apt-get install --no-install-recommends -y bat \
    bash-completion fish git-lfs

# Install Required Python Packages and link python3 executable to python.
RUN python -m pip install numpy opencv-python pyopengl matplotlib

# Set Timezone
RUN echo "${TZ}" > /etc/localtime && \
    echo "${TZ}" > /etc/timezone

# Install gcc/g++
RUN apt-get install  --no-install-recommends -y \
    gcc-10 g++-10 && \
    rm -rf /var/lib/apt/lists/* && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 100 --slave /usr/bin/g++ \
    g++ /usr/bin/g++-10 --slave /usr/bin/gcov gcov /usr/bin/gcov-10

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
RUN wget -q -O ZED_SDK_Linux_Ubuntu${UBUNTU_MAJOR}.run \
    https://download.stereolabs.com/zedsdk/${ZED_MAJOR}.${ZED_MINOR}/cu${CUDA_MAJOR}/ubuntu${UBUNTU_MAJOR} && \
    chmod +x ZED_SDK_Linux_Ubuntu${UBUNTU_MAJOR}.run ; ./ZED_SDK_Linux_Ubuntu${UBUNTU_MAJOR}.run silent && \
    ln -sf /lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/lib/x86_64-linux-gnu/libusb-1.0.so && \
    rm ZED_SDK_Linux_Ubuntu${UBUNTU_MAJOR}.run && \
    rm -rf /var/lib/apt/lists/* && \
    mkdir -p /root/Documents/ZED/ && \
    sed -i '/#pragma message*/d' /usr/local/zed/include/sl/Fusion.hpp && \
    sed -i '/#pragma message*/d' /usr/local/zed/include/sl/Camera.hpp && sed -i '/#warning*/d' /usr/local/zed/include/sl/Camera.hpp

# Install OpenCV
ARG OPENCV_VERSION="4.11.0"
RUN wget -q https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/opencv/amd64/opencv_${OPENCV_VERSION}_amd64.deb && \
    dpkg -i opencv_${OPENCV_VERSION}_amd64.deb && \
    rm opencv_${OPENCV_VERSION}_amd64.deb

# Install PyTorch.
ARG TORCH_VERSION="2.2.2"
RUN wget -q https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/pytorch/amd64/pytorch_${TORCH_VERSION}_amd64.deb && \
    dpkg -i pytorch_${TORCH_VERSION}_amd64.deb && \
    rm pytorch_${TORCH_VERSION}_amd64.deb

# Install Abseil.
ARG ABSEIL_VERSION="20230802.1"
RUN wget -q https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/abseil/amd64/abseil_${ABSEIL_VERSION}_amd64.deb && \
    dpkg -i abseil_${ABSEIL_VERSION}_amd64.deb && \
    rm abseil_${ABSEIL_VERSION}_amd64.deb

# Install GeographicLib
ARG GEOLIB_VERSION="2.3"
RUN wget -q https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/geolib/amd64/geolib_${GEOLIB_VERSION}_amd64.deb && \
    dpkg -i geolib_${GEOLIB_VERSION}_amd64.deb && \
    rm geolib_${GEOLIB_VERSION}_amd64.deb

# Install FFMPEG
ARG FFMPEG_VERSION="7.1"
RUN wget -q https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/ffmpeg/amd64/ffmpeg_${FFMPEG_VERSION}_amd64.deb && \
    dpkg -i ffmpeg_${FFMPEG_VERSION}_amd64.deb && \
    rm ffmpeg_${FFMPEG_VERSION}_amd64.deb

# Install Libdatachannel
ARG LIBDATACHANNEL_VERSION="0.22"
RUN wget -q https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/libdatachannel/amd64/libdatachannel_${LIBDATACHANNEL_VERSION}_amd64.deb && \
    dpkg -i libdatachannel_${LIBDATACHANNEL_VERSION}_amd64.deb && \
    rm libdatachannel_${LIBDATACHANNEL_VERSION}_amd64.deb

# Install Tensorflow.
ARG TENSORFLOW_VERSION="2.15.0"
RUN wget -q https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/tensorflow/amd64/tensorflow_${TENSORFLOW_VERSION}_amd64.deb && \
    dpkg -i tensorflow_${TENSORFLOW_VERSION}_amd64.deb && \
    rm tensorflow_${TENSORFLOW_VERSION}_amd64.deb

# Install Quill
ARG QUILL_VERSION="8.1.0"
RUN wget -q https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/quill/amd64/quill_${QUILL_VERSION}_amd64.deb && \
    dpkg -i quill_${QUILL_VERSION}_amd64.deb && \
    rm quill_${QUILL_VERSION}_amd64.deb

# Install Google Test
ARG GTEST_VERSION="1.16.0"
RUN wget -q https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/gtest/amd64/gtest_${GTEST_VERSION}_amd64.deb && \
    dpkg -i gtest_${GTEST_VERSION}_amd64.deb && \
    rm gtest_${GTEST_VERSION}_amd64.deb

# Enable Make Threads
RUN echo 'export MAKEFLAGS=-j$(($(grep -c "^processor" /proc/cpuinfo) - 1))' >> /root/.bashrc

# Fix Perl Locale Error
RUN sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen && locale-gen
ENV LANG en_US.UTF-8  
ENV LANGUAGE en_US:en  
ENV LC_ALL en_US.UTF-8  

# Enable Cowsay, Fortune, Lolcat, and other fun commands.
RUN mkdir -p ~/.config/fish/ && echo 'set fish_greeting; function random_message_or_donut; set rand (random 0 9999); if test $rand -eq 0; /workspaces/Autonomy_Software/data/Spinning_Donut/donut; else; set cowfile (ls /workspaces/Autonomy_Software/data/Cowsay_Cows/*.cow | shuf -n 1); /usr/games/fortune | /usr/games/cowsay -f $cowfile | /usr/games/lolcat -f; end; end; if status is-interactive; random_message_or_donut; end' >> ~/.config/fish/config.fish

# Set Fish as Default Shell.
RUN chsh -s /usr/bin/fish

# Clone Autonomy Software Repository
RUN git clone --recurse-submodules -j8 https://github.com/MissouriMRDT/Autonomy_Software.git

# Set Working Directory
WORKDIR /opt/Autonomy_Software/

# Set Labels
LABEL authors="Missouri S&T Mars Rover Design Team"
LABEL maintainer="Mars Rover Design Team <marsrover@mst.edu>"
LABEL org.opencontainers.image.source=https://github.com/missourimrdt/autonomy_software
LABEL org.opencontainers.image.licenses=GPL-3.0-only
LABEL org.opencontainers.image.version="v24.5.0"
LABEL org.opencontainers.image.description="Docker Image for Ubuntu ${UBUNTU_MAJOR}.${UBUNTU_MINOR} with CUDA ${CUDA_MAJOR}.${CUDA_MINOR}, ZED SDK ${ZED_MAJOR}.${ZED_MINOR}, OpenCV ${OPENCV_VERSION}, Quill ${QUILL_VERSION} and Google Test ${GTEST_VERSION}."
