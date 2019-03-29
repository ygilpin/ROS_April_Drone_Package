/* imageResize.cpp 
 * This ROS node subscribes to a rostopic image stream then resizes
 * the image and publishes it to a new image stream. It also publishes
 * a camera info topic.
 * By Yann Gilpin
 * With help from https://linuxconfig.org/resize-an-image-with-opencv-cvresize-function*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <camera_info_manager/camera_info_manager.h>
#include <sstream>

#define factor 0.5
#define calibration_url "file:///home/ubuntu/.ros/camera_info/head_camera.yaml"
#define camera_url "/dev/video0"
#define camera_name "head_camera"

static const std::string OPENCV_WINDOW = "Resize window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::CameraPublisher image_pub_;


public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertiseCamera("/imageResize/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Resize the image
    cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(), factor, factor, cv::INTER_AREA);

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);


    // Make a camera info topic object associated with the node
    ros::NodeHandle camera_nh;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
    cinfo_.reset(new camera_info_manager::CameraInfoManager(nh_, camera_name, calibration_url ));

    // Get calibration data from the file
    //cinfo_->loadCameraInfo();

    /*if (!cinfo_->isCalibrated()){
        cinfo_->setCameraName(video_device_name);
        sensor_msgs::CameraInfo camera_info;
        camera_info.header.frame_id = cv_ptr->toImageMsg().header.frame_id;
        camera_info.width = 25;
        camera_info.height = 25;
        cinfo_->setCameraInfo(camera_info);
    }*/
    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
    ci-> header.frame_id = "resize_stream";//cv_ptr->header.frame_id;
    ci-> header.stamp = cv_ptr->header.stamp;
    ci-> height = cv_ptr->image.rows;
    ci-> width = cv_ptr->image.cols;

    //Fix size in cv_ptr;
    sensor_msgs::ImagePtr img_ptr = cv_ptr->toImageMsg();
    (*img_ptr).height = cv_ptr->image.rows;
    (*img_ptr).width = cv_ptr->image.cols;
    (*img_ptr).header.frame_id = "resize_stream";
    // Output modified video stream
    image_pub_.publish(*(img_ptr), *ci, cv_ptr->header.stamp);
  }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imageResize");
  ImageConverter ic;
  ros::spin();
  return 0;
}
