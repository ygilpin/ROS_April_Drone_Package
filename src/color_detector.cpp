/* color_detection.cpp 
 * This ROS node detects a light blue brick and then computes its 
 * XY position in the image. It publishes the processed image on
 * ROS topic.It receives input from a /usb_cam/image_raw 
 * By Yann Gilpin
 * With help from */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define Hue_low   90
#define Hue_high  110
#define Sat_low   100
#define Sat_high  200
#define Val_low   100
#define Val_high  255
#define object_detect 60000

//Open CV 0-180 0-255 0-255
// light blue is 190-195 degrees various staturation 90 to 100 values
int posX = -1;
int posY = -1;
static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

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

    cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2HSV); //Convert the image to HSV space

    cv::Mat imgThresholded;
    cv::inRange(cv_ptr->image, cv::Scalar(Hue_low,Sat_low,Val_low), cv::Scalar(Hue_high,Sat_high,Val_high), imgThresholded);

    //Calulate the moments of the thresholded image
    cv::Moments oMoments = cv::moments(imgThresholded);

    double dArea = oMoments.m00;

    if (dArea > object_detect){
        posX = oMoments.m10 / dArea;
        posY = oMoments.m01 / dArea;
        cv_ptr->image = imgThresholded;
    }
    else {
        posX = -1;
        posY = -1;
    }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}