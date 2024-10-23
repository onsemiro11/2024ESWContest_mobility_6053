# Base image
FROM ros:jazzy-ros-core

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-jazzy-cv-bridge \
    ros-jazzy-sensor-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-cyclonedds \
    iputils-ping \
    nano \
    libopencv-dev \
    python3-rosdep \
    ros-jazzy-ament-cmake \
    python3-opencv \
    python3-colcon-common-extensions \
    build-essential

# Initialize rosdep
RUN rosdep init && rosdep update

# Set up workspace
WORKDIR /ros2_ws

# Copy your ROS2 workspace and build it
COPY ./src /ros2_ws/src
RUN apt-get update && \
    /bin/bash -c "source /opt/ros/jazzy/setup.bash && rosdep install --from-paths src --ignore-src -r -y && colcon build"

# Source the workspace
RUN echo "export ROS_DOMAIN_ID=10" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
RUN echo "export AMENT_PREFIX_PATH="/ros2_ws/install/my_ros2_package:$AMENT_PREFIX_PATH"" >> ~/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
SHELL ["/bin/bash", "-c"]
RUN apt-get install ros-jazzy-rmw-cyclonedds-cpp

RUN apt-get update && \
    apt-get install -y gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly python3-gst-1.0 && \
    apt-get install -y python3-gi


# Run the node
# CMD ["ros2", "run", "my_ros2_package", "camera_image_publisher"]
