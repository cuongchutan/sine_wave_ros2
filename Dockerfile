# Use the official ROS 2 Jazzy base image
FROM osrf/ros:jazzy-desktop-full

# Set the maintainer label
LABEL maintainer="yourname@example.com"

# Install necessary dependencies
RUN apt-get update && \
    apt-get install -y \
    python3-rosdep \
    python3-pip \
    python3-venv \
    python3-opencv \
    python3-jinja2 \
    python3-typeguard \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN if [ -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then rm /etc/ros/rosdep/sources.list.d/20-default.list; fi && \
    rosdep init && \
    rosdep update

# Set the working directory
WORKDIR /ros2_ws

# Clone the repositories
RUN mkdir -p src && cd src && \
    git clone https://github.com/cuongchutan/sine_wave_ros2.git && \
    git clone https://github.com/PickNikRobotics/generate_parameter_library.git

# Install dependencies using rosdep
RUN rosdep install --from-paths src --ignore-src -r -y

# Install Python dependencies
# Create a virtual environment and install Python dependencies
RUN python3 -m venv /ros2_ws/venv && \
    /ros2_ws/venv/bin/pip install --upgrade pip && \
    /ros2_ws/venv/bin/pip install -r src/sine_wave_ros2/sine_wave/requirements.txt

# Build the workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# Source the setup script
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Set the entrypoint
ENTRYPOINT ["/bin/bash", "-c", "source /ros2_ws/install/setup.bash && bash"]
