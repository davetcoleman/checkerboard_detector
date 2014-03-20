checkerboard_detector
=====================

Detects a checkerboard and publishes the camera's location with respect to the known checkerboard frame transform

## Run

```
roslaunch openni_launch openni.launch depth_registration:=true
roslaunch baxter_moveit_config planning_context.launch load_robot_description:=true
roslaunch baxter_experimental flex_perception_rviz.launch
roslaunch checkerboard_detector objectdetection_tf_publisher.launch 
roslaunch checkerboard_detector checkerboard_detector.launch 
rosrun tf static_transform_publisher 0.25 0 0 1.57 0 1.57 /base /checkerboard 100

Fake the calibration
```
rosrun tf static_transform_publisher 3 0 0 0 3.14 3.14 /base /camera_link 100 
```