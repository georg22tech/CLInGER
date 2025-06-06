FROM ros:humble-ros-base

# Install required packages
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-rviz2 \
    ros-humble-joy \
    ros-humble-urdf-launch \
    jstest-gtk \
    xvfb \
    sudo \
    git \
    x11-apps \
    curl \
    gnupg \
    lsb-release \
    && apt-get clean

# Create user 'ros'
RUN useradd -m -s /bin/bash ros && \
    echo "ros ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER ros
WORKDIR /raspberry_pi_ws  

# Copy entrypoint
COPY --chown=ros:ros entrypoint.sh /home/ros/entrypoint.sh
RUN chmod +x /home/ros/entrypoint.sh

# Auto-source ROS and your workspace on every shell start
RUN echo 'source /opt/ros/humble/setup.bash' >> /home/ros/.bashrc && \
    echo 'source /raspberry_pi_ws/install/setup.bash' >> /home/ros/.bashrc


ENTRYPOINT ["/home/ros/entrypoint.sh"]
CMD ["/bin/bash"]
