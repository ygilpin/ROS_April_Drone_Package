# ROS_April_Drone_Package
april_drone a is a combination of many other ROS Indigo Packages. It links [usb_cam](http://wiki.ros.org/usb_cam), and [april_tags](http://wiki.ros.org/apriltags_ros) so allow for realtime image AprilTag recoginition and relative localization. 
# Additional pseudodependencies
1) It is important to calibrate your camera so that the april tag library can determine its relative position from the image alone. 
2) The ROS [camera_calibration](http://wiki.ros.org/camera_calibration) package works well. After downloading the package, follow the [instructions](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration).
3) 
4) The calibration data is by default stored in `/tmp`
5) Extract using `tar xfv ...` 
6) Put the .yaml file where the april_tag package expects it `mv ost.yaml /home/ubuntu/.ros/camera_info/`
7) Change the camera_name parameter from narrow_stereo to head_camera and rename the file:
  `cat ost.yaml | sed -E 's/camera_name:.*$/camera_name: head_camera > head_camera.yaml`
