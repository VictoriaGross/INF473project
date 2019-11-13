#include "opencv2/opencv.hpp"
#include <iostream>

using namespace std;
using namespace cv;

const int capture_number = 4;

void capture_and_show(int n)
{
	VideoCapture cap(0);

	// Check if camera opened successfully
	if (!cap.isOpened()) {
		cout << "Error opening video stream" << endl;
		return;
	}

	vector<Mat> captures;
	int capture_count = 0;

	for (;;)
	{
		// show webcam image and capture keys
		Mat frame;
		cap >> frame;
		if (frame.empty()) break;
		imshow("Webcam", frame);

		int key = waitKey(1);

		// on SPACE key, capture and show current frame
		if (key == 32)
		{
			captures.push_back(frame);
			capture_count += 1;
			//imshow("Capture", frame);
		}
		// on ESC key or sufficient frames, close application
		else if (key == 27 || capture_count == capture_number) break;

	}

	// after capture_number images taken, we stop image capturing
	cap.release();
	destroyWindow("Webcam");
	destroyWindow("Capture");

	// show captured images

	for (int i = 0; i < captures.size(); i++)
	{
		Mat img = captures[i];
		string img_name = to_string(i);
		imshow(img_name, img);
	}
}

void calibrate(Size s, Mat& cameraMatrix, Mat &distCoeffs)
{

	VideoCapture cap(0);
	vector<Mat> captures;
	int capture_count = 0;

	// Check if camera opened successfully
	if (!cap.isOpened()) {
		cout << "Error opening video stream" << endl;
		return;
	}

	// Capture until circles are recognised
	for (;;)
	{
		Mat frame, gray;
		cap >> frame;
		if (frame.empty())
		{
			cout << "Error reading webcam frame" << endl;
			break;
		}

		int key = waitKey(1);

		imshow("Webcam", frame);

		if (key == 27) break;

		cvtColor(frame, gray, COLOR_BGR2GRAY);
		vector<Point2f> centers; //this will be filled by the detected centers

		bool patternfound = findCirclesGrid(gray, s, centers, CALIB_CB_ASYMMETRIC_GRID);

		if (patternfound)
		{
			drawChessboardCorners(frame, s, Mat(centers), patternfound);
			imshow("found circles", frame);
			break;
		}
	}

	cap.release();

	cameraMatrix = Mat::eye(3, 3, CV_64F);

	distCoeffs = Mat::zeros(8, 1, CV_64F);





	

	


}



int main(){
	// input video from webcam
	//capture_and_show(capture_number);

	Size patternsize(4, 11); //number of centers
	Mat cameraMatrix, distCoeffs;

	calibrate(patternsize, cameraMatrix, distCoeffs);
	waitKey();

	/*------ CALIBRATION -------*/
	return 0;
}