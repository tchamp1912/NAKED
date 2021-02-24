#include <functional>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>		

#include <opencv2/core/mat.hpp>

class Camera {

public:
	Camera(int height_, int width_) : 
		res(width_, height_)
		{}

	bool init() {
		rectifiedImage = cv::Mat(res, CV_8UC3);
	};

	bool registerCallback(std::function<void(cv::Mat&, cv::Mat&)> callback_) { callback = callback_; };

	void imageCallback(const sensor_msgs::ImageConstPtr &image) {
		
		try {
			rawImage = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

        	invokeCallback();
	};

	void invokeCallback() { callback(rawImage->image, rectifiedImage); };

	cv::Mat getRectifiedImage() { return rectifiedImage; }

private:

	cv::Size res;
	cv_bridge::CvImagePtr rawImage;
	cv::Mat rectifiedImage;
	std::function<void(cv::Mat&, cv::Mat&)> callback;
		
};
