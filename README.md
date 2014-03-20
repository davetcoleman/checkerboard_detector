checkerboard_detector
=====================

Detects a checkerboard and publishes the camera's location with respect to the known checkerboard frame transform

## Run

```
roslaunch openni_launch openni.launch depth_registration:=true
roslaunch checkerboard_detector objectdetection_tf_publisher.launch 
roslaunch checkerboard_detector checkerboard_detector.launch 
```

Start Rviz and set your fixed frame to 'base'.