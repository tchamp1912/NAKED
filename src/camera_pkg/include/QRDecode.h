#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include <ros/console.h>

using namespace cv;
using namespace std;

std::string detectAndDecode(Mat inputImage) {
  cv::QRCodeDetector qrDecoder;
  cv::Mat rectifiedImage;

  Mat bbox;

  std::string data = qrDecoder.detectAndDecode(inputImage, bbox, rectifiedImage);
  if(data.length()>0)
  {
    ROS_INFO("QR detected and decoded by OpenCV");
    return data;
  }
  else
  {
    data = "None";
    return data;
  }
}
