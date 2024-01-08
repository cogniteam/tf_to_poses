# tf to poses
## Introduction
This project is used for users who wants to receive the position of the robot, the lidar and the camera on the in geometry_msg.Pose on wanted global frame.
The messageד is calculated by the lookuptransform mechanism of the TF.
### docker image
https://hub.docker.com/repository/docker/cognimbus/tf_to_poses/general
  

### parameters
#### base_frame
the wanted base_frame (robot's location on the map).
#### laser_frame
the wanted laser_frame (lidar's location on the map).
#### camera_frame
the wanted camera_frame (camera's location on the map).
#### rate
The rate in HZ for publishing the messages

### running the launch
#### ros2 launch tf_to_poses bringup_launch.py


