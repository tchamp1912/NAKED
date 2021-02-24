#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include <ros/console.h>

using namespace cv;
using namespace std;

#define QR_FOUND 0
#define NO_QR_FOUND -1

void display(Mat &im, Mat &bbox)
{
  int n = bbox.rows;
  for(int i = 0 ; i < n ; i++)
  {
    line(im, Point2i(bbox.at<float>(i,0),bbox.at<float>(i,1)), Point2i(bbox.at<float>((i+1) % n,0), bbox.at<float>((i+1) % n,1)), Scalar(255,0,0), 3);
  }
  imshow("Result", im);
}

int detectAndDecode(Mat inputImage, Mat rectifiedImage) {

  static QRCodeDetector qrDecoder;

  Mat bbox;

  imwrite("/home/husarion/NAKED/Image.jpg", inputImage);

  std::string data = qrDecoder.detectAndDecode(inputImage, bbox, rectifiedImage);
  if(data.length()>0)
  {
    cout << "Decoded Data : " << data << endl;

    display(inputImage, bbox);
    rectifiedImage.convertTo(rectifiedImage, CV_8UC3);
    imshow("Rectified QRCode", rectifiedImage);

    return QR_FOUND;
  }
  else 
  {
    cout << "QR Code not detected" << endl;
    return NO_QR_FOUND;
  }
}
