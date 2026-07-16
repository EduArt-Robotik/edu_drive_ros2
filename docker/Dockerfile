FROM eduartrobotik/eduart-ros-base:jazzy-1.2.3

ENV USER=user
ENV DEBIAN_FRONTEND=noninteractive

# install packages
USER root
RUN apt-get update \
    && apt-get install -y software-properties-common build-essential git

USER $USER

# set up ros workspace
RUN mkdir -p /home/$USER/ros/src/
COPY edu_drive_ros2 /home/$USER/ros/src/edu_drive_ros2
WORKDIR /home/$USER/ros

# build the sensorring ros package
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/.bashrc \
    && colcon build --packages-select edu_drive_ros2 --symlink-install --executor sequential --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && source install/setup.bash

# add entry point
COPY --chown=$USER edu_drive_ros2/docker/entrypoint /home/$USER/
RUN chmod a+x /home/$USER/entrypoint
ENTRYPOINT [ "/home/user/entrypoint" ]