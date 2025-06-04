ARG TARGETARCH

# Base images for different architectures
FROM robotis/ros:jazzy-base-torch2.7.0-cuda12.8.0 AS base-arm64
FROM robotis/ros:jazzy-base-torch2.7.0-cuda12.8.0 AS base-amd64

# Select the appropriate base image based on TARGETARCH
FROM base-${TARGETARCH} AS base

#################################
#   Librealsense Builder Stage  #
#################################
FROM base AS librealsense-builder

ARG TARGETARCH

# ARG LIBRS_VERSION=2.56.3
# # Make sure that we have a version number of librealsense as argument
# RUN test -n "$LIBRS_VERSION"

# To avoid waiting for input during package installation
ENV DEBIAN_FRONTEND=noninteractive

RUN rm /etc/apt/sources.list.d/ros2.list || true && \
    rm /usr/share/keyrings/ros-archive-keyring.gpg || true && \
    rm /etc/apt/sources.list.d/ros2-latest.list || true && \
    rm /usr/share/keyrings/ros2-latest-archive-keyring.gpg || true

RUN apt-get update \
  && apt-get install -y ca-certificates curl

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') ;\
    curl -L -s -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt-get update \
    && apt-get install /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb

# Builder dependencies installation
RUN apt-get update || true \
    && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    curl \
    python3 \
    python3-dev \
    python3-pip \
    libpython3-dev \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Download sources
WORKDIR /usr/src
# RUN curl https://codeload.github.com/IntelRealSense/librealsense/tar.gz/refs/tags/v$LIBRS_VERSION -o librealsense.tar.gz
# RUN tar -zxf librealsense.tar.gz \
#     && rm librealsense.tar.gz
# RUN ln -s /usr/src/librealsense-$LIBRS_VERSION /usr/src/librealsense

RUN git clone https://github.com/IntelRealSense/librealsense.git && \
    cd librealsense && \
    git checkout 1e34346894be975e8e594fab22f8b31a89a710bb

# Build and install
RUN cd /usr/src/librealsense \
    && mkdir build && cd build \
    && cmake \
    -DCMAKE_C_FLAGS_RELEASE="${CMAKE_C_FLAGS_RELEASE} -s" \
    -DCMAKE_CXX_FLAGS_RELEASE="${CMAKE_CXX_FLAGS_RELEASE} -s" \
    -DCMAKE_INSTALL_PREFIX=/opt/librealsense \
    -DBUILD_GRAPHICAL_EXAMPLES=true \
    -DBUILD_PYTHON_BINDINGS:bool=true \
    -DCMAKE_BUILD_TYPE=Release ../ \
    -DFORCE_RSUSB_BACKEND=true \
    -DBUILD_WITH_CUDA=true \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    && make -j$(($(nproc)-1)) all \
    && make install

# Use a conditional RUN step to handle pyrealsense2 copy
# We'll copy to a temp location first to unify the path
RUN if [ "$TARGETARCH" = "arm64" ]; then \
    cp -r /opt/venv/lib/python3.12/site-packages/pyrealsense2 /usr/local/lib/python3.12/dist-packages/; \
    fi

###################################
#   AI Worker Base Image Stage    #
###################################
FROM base

ARG TARGETARCH

RUN if [ "$TARGETARCH" = "arm64" ]; then \
    echo 'export PATH=/opt/venv/bin:$PATH' >> /root/.bashrc && \
    echo 'export PYTHONPATH=/opt/venv/lib/python3.12/site-packages:$PYTHONPATH' >> /root/.bashrc && \
    export PATH=/opt/venv/bin:$PATH && \
    export PYTHONPATH=/opt/venv/lib/python3.12/site-packages:$PYTHONPATH; \
    fi

# Copy binaries from builder stage
COPY --from=librealsense-builder /opt/librealsense /usr/local/
COPY --from=librealsense-builder /usr/local/lib/python3.12/dist-packages/pyrealsense2 /usr/local/lib/python3.12/dist-packages/pyrealsense2
COPY --from=librealsense-builder /usr/src/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
COPY --from=librealsense-builder /usr/src/librealsense/config/99-realsense-d4xx-mipi-dfu.rules /etc/udev/rules.d/
ENV PYTHONPATH=$PYTHONPATH:/usr/local/lib

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

RUN rm /etc/apt/sources.list.d/ros2.list || true && \
    rm /usr/share/keyrings/ros-archive-keyring.gpg || true && \
    rm /etc/apt/sources.list.d/ros2-latest.list || true && \
    rm /usr/share/keyrings/ros2-latest-archive-keyring.gpg || true

RUN apt-get update \
  && apt-get install -y ca-certificates curl

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') ;\
    curl -L -s -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt-get update \
    && apt-get install /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb

# Install ZED SDK
ARG L4T_MAJOR_VERSION=36
ARG L4T_MINOR_VERSION=4
ARG L4T_PATCH_VERSION=3
ARG ZED_SDK_MAJOR=5
ARG ZED_SDK_MINOR=0

RUN if [ "$TARGETARCH" = "arm64" ]; then \
    apt-get update -y || true && \
    apt-get install -y --no-install-recommends \
    lsb-release wget less zstd sudo apt-transport-https \
    udev=255.4-1ubuntu8 libudev1=255.4-1ubuntu8 --allow-downgrades && \
    echo "# R${L4T_MAJOR_VERSION} (release), REVISION: ${L4T_MINOR_VERSION}.${L4T_PATCH_VERSION}" > /etc/nv_tegra_release && \
    wget -q --no-check-certificate -O ZED_SDK_Linux.run https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/l4t${L4T_MAJOR_VERSION}.${L4T_MINOR_VERSION}/jetsons && \
    chmod +x ZED_SDK_Linux.run && \
    ./ZED_SDK_Linux.run silent skip_drivers && \
    rm -rf /usr/local/zed/resources/* && \
    rm -rf ZED_SDK_Linux.run && \
    rm -rf /var/lib/apt/lists/* && \
    ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 /usr/lib/aarch64-linux-gnu/libv4l2.so; \
    fi

# Install realsense-viewer dependencies
RUN apt-get update && apt-get install -y \
    libusb-1.0-0-dev \
    libglfw3-dev \
    && rm -rf /var/lib/apt/lists/*

# Define build argument for Gazebo support
ARG WITH_GAZEBO=true

# Install common system dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-pal-statistics \
    vim \
    nano \
    && apt-get install -y ros-${ROS_DISTRO}-moveit --no-install-recommends \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo dependencies conditionally
RUN if [ "$WITH_GAZEBO" = "true" ]; then \
    apt-get update && apt-get install -y ros-${ROS_DISTRO}-gz* \
    && rm -rf /var/lib/apt/lists/*; \
    fi

ENV COLCON_WS=/root/ros2_ws
WORKDIR ${COLCON_WS}

RUN mkdir -p ${COLCON_WS}/src && \
    cd ${COLCON_WS}/src && \
    git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git && \
    git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git && \
    git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git && \
    git clone -b jazzy https://github.com/ROBOTIS-GIT/physical_ai_tools.git --recursive && \
    git clone -b jazzy https://github.com/ROBOTIS-GIT/ai_worker.git && \
    git clone -b r/4.56.4 https://github.com/IntelRealSense/realsense-ros.git && \
    cd realsense-ros && git checkout cbd8b8487d4c0126bb590822db9469414496e62a

RUN if [ "$TARGETARCH" = "arm64" ]; then \
    cd ${COLCON_WS}/src && \
    git clone https://github.com/stereolabs/zed-ros2-wrapper.git && \
    cd zed-ros2-wrapper && git checkout 4c8ee060862c9a85d711d68b6a4f1571bfa3dcc9; \
    fi

# Clone gz_ros2_control only if WITH_GAZEBO is true
RUN if [ "$WITH_GAZEBO" = "true" ]; then \
    cd ${COLCON_WS}/src && \
    git clone -b jazzy https://github.com/ros-controls/gz_ros2_control; \
    fi

RUN cd ${COLCON_WS} && \
    apt-get update && \
    rosdep update && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys="librealsense2 dynamixel_hardware_interface dynamixel_interfaces dynamixel_sdk physical_ai_tools ai_worker gz_ros2_control" -y && \
    rm -rf /var/lib/apt/lists/*

RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${COLCON_WS} && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

RUN apt-get update && \
    apt-get install -y ffmpeg && \
    rm -rf /var/lib/apt/lists/*

RUN cd ${COLCON_WS}/src/physical_ai_tools/lerobot && \
    pip install --ignore-installed -e .

RUN pip install setuptools==68.1.2
RUN pip install 'numpy<2'

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc  && \
    echo "source ${COLCON_WS}/install/setup.bash" >> ~/.bashrc  && \
    echo "alias cb='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'" >> ~/.bashrc && \
    echo "alias ffw_lg2_leader_ai='ros2 launch ffw_bringup ffw_lg2_leader_ai.launch.py'" >> ~/.bashrc && \
    echo "alias ffw_bg2_follower_ai='ros2 launch ffw_bringup ffw_bg2_follower_ai.launch.py'" >> ~/.bashrc && \
    echo "alias ffw_bg2_ai='ros2 launch ffw_bringup ffw_bg2_ai.launch.py'" >> ~/.bashrc
