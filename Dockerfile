FROM ros:noetic

RUN  apt update && apt-get install git -y

WORKDIR /tf_to_poses_ws/src

RUN git clone https://github.com/yakir-cogniteam/tf_to_poses.git -b noetic
WORKDIR /tf_to_poses_ws/


# Set up an entry point script
COPY entrypoint.sh /tf_to_poses_ws/entrypoint.sh
RUN chmod +x /tf_to_poses_ws/entrypoint.sh

# Entrypoint script
ENTRYPOINT ["/tf_to_poses_ws/entrypoint.sh"]


RUN . /opt/ros/noetic/setup.sh && catkin_make

#ros2 launch tf_to_poses bringup_launch.py
