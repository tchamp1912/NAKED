#include <functional>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>		

#include <opencv2/core/mat.hpp>

class Camera {

public:
	Camera(int height_, int width_) : 
		res(width_, height_)
		{}

	bool registerCallback(std::function<std::string(cv::Mat&)> callback_) { callback = callback_; };

	void imageCallback(const sensor_msgs::ImageConstPtr &image) {
		
		try {
			rawImage = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

        invokeCallback();
	};

	std::string getData() { return data; }

private:

	void invokeCallback() { data = callback(rawImage->image, rectifiedImage); };

	cv::Size res;
	std::string data;
	cv_bridge::CvImagePtr rawImage;
	std::function<std::string(cv::Mat&, cv::Mat&)> callback;
		
};
