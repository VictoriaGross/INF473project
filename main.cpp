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
	WebcamClass webcam(captures);

	//// save some images
	//webcam.capture_and_show(true);
	//webcam.clear();

	webcam.load_params();
	webcam.print();
	//webcam.calibrate_from_video(Pattern::ASYMMETRIC_CIRCLES_GRID, 4, 11, 10);
	
	
	
	webcam.capture_and_show(true);
	
	cout << "Stored frames:" << captures.size() << endl;


	Mat frame1 = captures[0];
	Mat frame2 = captures[1];
	
	Mat points = webcam.points3d(frame1, frame2);

	webcam.writeCSV("../points.csv", points);

	//webcam.show_undistorted();




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