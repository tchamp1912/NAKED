#include <functional>

#include <sensor_msgs/Image.h>

#include <opencv2/core>

class Camera {

public:
	Camera(int height_, int width) : 
		res(width_, height_)
		{}


	bool RegisterCallback(std::function<cv::Mat> callback_);

	void imageCallback(const sensor_msgs::ImageConstPtr &image);

private:

	cv::Size res;
	std::function<cv::Mat> callback;
	
};
