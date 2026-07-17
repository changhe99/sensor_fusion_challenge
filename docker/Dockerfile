# ROS 2 Humble (Ubuntu 22.04, Python 3.10) dev image for the sensor fusion
# challenge. Host is Ubuntu 24.04; this container provides the Humble toolchain
# and talks to the BMI088 Application Board over USB (libusb via coinespy).
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-colcon-common-extensions \
        build-essential \
        libusb-1.0-0 \
        udev \
        ros-humble-sensor-msgs \
        ros-humble-imu-filter-madgwick \
    && rm -rf /var/lib/apt/lists/*

# Bosch COINES Python wrapper — how the driver reaches the Application Board.
# Ubuntu 22.04 pip is not PEP 668 externally-managed, so a plain install works.
# coinespy pulls in pytest 9, which breaks ROS Humble's launch_testing pytest
# plugins; pin pytest <8 so both the ROS plugins and our unit tests run.
RUN pip3 install --no-cache-dir coinespy "pytest<8"

# The repo is mounted here at runtime (see docker-compose.yml).
WORKDIR /ws

# Auto-source ROS and the workspace overlay in interactive shells.
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
 && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc

CMD ["bash"]
