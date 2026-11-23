FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=burger
# Let gzserver find Gazebo's bundled models (sun, ground_plane) for the stock
# turtlebot3_world, and never block on the online model database.
ENV GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models
ENV GAZEBO_MODEL_DATABASE_URI=""

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-pip \
    python3-pytest \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY src/ /ros2_ws/src/

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch my_robot_navigation robot_navigation.launch.py"]
