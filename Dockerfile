FROM ros:jazzy-ros-base

# Install necessary dependencies
RUN apt-get update
RUN apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-venv \
    python3-opencv \
    libopencv-dev \
    python3-rosdep \
    qtbase5-dev \
    ros-jazzy-rsl \
    ros-jazzy-tcb-span \
    ros-jazzy-tl-expected \
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins \
    python3-jinja2 \
    python3-typeguard
RUN rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

ENV OpenCV_DIR=//usr/share/opencv4
ENV QT_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins
ENV QT_QPA_PLATFORM=xcb
ENV XDG_RUNTIME_DIR=/tmp/runtime-root
# ENV LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

# Creat src directory and clone the generate parameter library
RUN mkdir -p src && cd src && \
    git clone https://github.com/PickNikRobotics/generate_parameter_library.git

COPY sine_wave ./src/sine_wave
COPY grayscale_image ./src/grayscale_image

# Install ROS-dependencies using rosdep
RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update && \
    /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y"

# Create a virtual environment and install Python dependencies
RUN python3 -m venv ros2_ws_venv
RUN /bin/bash -c "source ./ros2_ws_venv/bin/activate" && \
    ./ros2_ws_venv/bin/pip install --upgrade pip && \
    ./ros2_ws_venv/bin/pip install -r src/sine_wave/requirements.txt

# Build the workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# Source the setup file
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
ENTRYPOINT ["/bin/bash", "-c", "source /ros2_ws/install/setup.bash && bash"]
