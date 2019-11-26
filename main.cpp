#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <fstream>

#include "WebcamClass.cpp"


using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
	Mat intrinsic = Mat::eye(3, 3, CV_32FC1);
	Mat distCoeffs = Mat::zeros(8, 1, CV_32FC1);

	vector<Mat> captures;

	// dummy camera parameters
	WebcamClass webcam(intrinsic, distCoeffs, 4, 4, 11, captures);
	webcam.set_calibration(true);

	// save some images
	webcam.capture_and_show(false);



	Mat H = webcam.pairwise_homography();

	std::cout << H << std::endl;

	//webcam.calibrate();
	//webcam.load_params();
	webcam.print();
	

	waitKey();
}