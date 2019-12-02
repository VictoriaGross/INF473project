#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <fstream>

#include "WebcamClass.cpp"


using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
	Pattern pattern;
	int w, h, n_calib;

	vector<Mat> captures;
	WebcamClass webcam(captures);

	cv::CommandLineParser parser(argc, argv, "{w|4|}{h|11|}{pt|acircles|}{n_calib|10|}{calibrate||}{load||}");

	if (parser.has("calibrate"))
	{
		if (!(parser.has("pt") && parser.has("n_calib") && parser.has("w") && parser.has("h")))
		{
			cerr << "Not enough parameters, please specify -pt, -w, -h, -n_calib\n";
			abort();
		}
		string val = parser.get<string>("pt");
		if (val == "circles")
			pattern = CIRCLES_GRID;
		else if (val == "acircles")
			pattern = ASYMMETRIC_CIRCLES_GRID;
		else if (val == "chessboard")
			pattern = CHESSBOARD;
		else
			return fprintf(stderr, "Invalid pattern type: must be chessboard or circles\n"), -1;

		n_calib = parser.get<int>("n_calib");
		w = parser.get<int>("w");
		h = parser.get<int>("h");
		cout << "Performing camera calibration with " << val << "pattern, " << w << "x" << h << "size and " << n_calib << "samples\n";
		webcam.calibrate_from_video(pattern, w, h, n_calib);
	}
	else if (parser.has("load"))
	{
		webcam.load_params();
	}

	cout << "Current camera parameters:\n";
	webcam.print();
		
	

	//// save some images
	//webcam.capture_and_show(true);
	//webcam.clear();

	//webcam.load_params();
	//webcam.print();
	////webcam.calibrate_from_video(Pattern::ASYMMETRIC_CIRCLES_GRID, 4, 11, 10);
	//
	//
	//
	//webcam.capture_and_show(true);
	//
	//cout << "Stored frames:" << captures.size() << endl;


	//Mat frame1 = captures[0];
	//Mat frame2 = captures[1];
	//
	//Mat points = webcam.points3d(frame1, frame2);

	//webcam.writeCSV("../points.csv", points);

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