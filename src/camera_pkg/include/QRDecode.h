#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include <ros/console.h>

using namespace cv;
using namespace std;

std::string detectAndDecode(Mat inputImage, Mat rectifiedImage) {

  static QRCodeDetector qrDecoder;

  Mat bbox;

  std::string data = qrDecoder.detectAndDecode(inputImage, bbox, rectifiedImage);
  if(data.length()>0)
  {
    return data;
  }
  else 
  {
    return NULL;
  }
}
