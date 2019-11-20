#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <fstream>

#include "WebcamClass.cpp"


using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
	Mat intrinsic;
	Mat distCoeffs; 

	WebcamClass webcam(intrinsic, distCoeffs, 4, 4, 11);

	//webcam.capture_and_show();

	//webcam.calibrate();
	webcam.load_params();
	webcam.print();
	

	waitKey();
}