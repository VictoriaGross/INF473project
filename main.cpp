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
	bool debug;

	vector<Mat> captures;
	WebcamClass webcam(captures);

	// Argument parsing & camera setup

	CommandLineParser parser(argc, argv, "{w|4|}{h|11|}{pt|acircles|}{n_calib|10|}{calibrate||}{load||}{debug||}");

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

	if (parser.has("debug"))
		debug = true;

	cout << "Current camera parameters:\n";
	webcam.print();

	// Actual 3D point generation
	
	// save some images
	webcam.capture_and_show(true);

	
	cout << "Stored frames:" << captures.size() << endl;

	Mat points = webcam.points3d(captures[0], captures[1], debug);
	webcam.writeCSV("../points.csv", points);

	//webcam.show_undistorted();

	waitKey();
	return 0;
}