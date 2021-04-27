#include <iostream>
#include <opencv2/opencv.hpp>


void video() { }

int main() {
	cv::VideoCapture* capdev;

	// open the video device
	capdev = new cv::VideoCapture(0);
	if (!capdev->isOpened()) {
		printf("Unable to open video device\n");
		return (-1);
	}

	// get some properties of the image
	cv::Size refS((int)capdev->get(cv::CAP_PROP_FRAME_WIDTH),
	              (int)capdev->get(cv::CAP_PROP_FRAME_HEIGHT));

	cv::namedWindow("Video", 1); // identifies a window
	cv::Mat frame;
	cv::Mat gray;

	while (true) {
		*capdev >> frame; // get a new frame from the camera, treat as a stream
		if (frame.empty()) {
			printf("frame is empty\n");
			break;
		}

		cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

		std::vector<cv::Point2f> corners;
		cv::goodFeaturesToTrack(gray, corners, 100, 0.1, 10);

		for (int idx = 0; idx < corners.size(); idx++) {
			cv::circle(frame, corners.at(idx), 2, cv::Scalar(0, 0, 255), 2);
		}

		std::cout << corners.size() << std::endl;


		char key = cv::waitKey(10);

		if (key == 'q') { break; }

		cv::imshow("video", frame);
	}
	return 0;
}
