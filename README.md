# ROS_April_Drone_Package
april_drone a is a combination of many other ROS Indigo Packages. It links [usb_cam](http://wiki.ros.org/usb_cam), and [april_tags](http://wiki.ros.org/apriltags_ros) so allow for realtime image AprilTag recoginition and relative localization. 
## Additional pseudodependencies
1) It is important to calibrate your camera so that the april tag library can determine its relative position from the image alone. 
2) The ROS [camera_calibration](http://wiki.ros.org/camera_calibration) package works well. After downloading the package, follow the [instructions](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration). Depending on setup an example process flow might be: 
3) `roslaunch usb_cam usb_cam-test.launch &`
4) `rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0254 image:=/usb_cam?image_raw camera:=/usb_cam`
5) The calibration data is by default stored in `/tmp`
6) Extract using `tar xfv ...` 
7) Put the .yaml file where the april_tag package expects it `mv ost.yaml /home/ubuntu/.ros/camera_info/`
8) Change the camera_name parameter from narrow_stereo to head_camera and rename the file:
  `cat ost.yaml | sed -E 's/camera_name:.*$/camera_name: head_camera > head_camera.yaml`
## Launch Files
1) `demo.launch` runs most the core ros packages (without automatically a live stream). If you wish to see these things run the approprtiate image view or demo.launch.bak
2) `demo.launch.bak` runs all of the core ros packages and will automatically open up the camera stream using the image view. Running another image view on the /tag_detections_image topic will show the live stream with the detected tags circled.
3) `highspeed.launch` as the name implies does not open a live view nor does it publish the tag_detections_image topic to save computational power. 
## Other branches
1) The openCV branch has similar work, but with an resized image stream using openCV or img_proc package. The image processing one has a slightly better performance, but the openCV has much more image processing options
