FROM ros:humble

RUN  apt update

WORKDIR /tf_to_poses_ws/src

RUN git clone https://github.com/yakir-cogniteam/tf_to_poses_ros2_humble.git and cd ..
WORKDIR /tf_to_poses_ws/


# Set up an entry point script
COPY entrypoint.sh /tf_to_poses_ws/entrypoint.sh
RUN chmod +x /tf_to_poses_ws/entrypoint.sh

# Entrypoint script
ENTRYPOINT ["/tf_to_poses_ws/entrypoint.sh"]


RUN . /opt/ros/humble/setup.sh . && . /tf_to_poses_ws/install/setup.sh && colcon build --symlink-install 
