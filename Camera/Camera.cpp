#include "Camera.h"

void Camera::RegisterCallback(std::function<cv::Mat> callback_) {
	callback = callback_;
}

void Camera::imageCallback(const sensor_msgs::ImageConstPtr &image) {
	cv::Mat imageMat = cv::Mat(res, CV_8U);

	memcpy(imageMat.data, image.data, res.width * res.height);

	callback(imageMat);
}
