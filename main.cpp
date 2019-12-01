#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <fstream>

#include "WebcamClass.cpp"


using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
	vector<Mat> captures;

	//// dummy camera parameters
	WebcamClass webcam(captures);

	//// save some images
	webcam.capture_and_show(true);
	webcam.clear();

	webcam.load_params();


	//webcam.calibrate_from_video(Pattern::ASYMMETRIC_CIRCLES_GRID, 4, 11, 10);

	cout << webcam.cameraMatrix << endl;

	webcam.show_undistorted();




	//Mat H = webcam.pairwise_homography();

	//std::cout << H << std::endl;

	//Mat P;
	//P = webcam.projected_points(H, webcam.cam0pnts, webcam.cam1pnts);

	//std::cout << P << std::endl;

	////webcam.calibrate();
	////webcam.load_params();
	////webcam.print();
	
	

	waitKey();
	return 0;
}