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

static void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners)
{
	corners.resize(0);

	for (int i = 0; i < boardSize.height; i++)
		for (int j = 0; j < boardSize.width; j++)
			corners.push_back(Point3f(float((2 * j + i % 2) * squareSize),
				float(i * squareSize), 0));

}
bool calibrate(Size s, Size imageSize, Mat& cameraMatrix, Mat &distCoeffs)
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

	/*-                                      */

	cameraMatrix = Mat::eye(3, 3, CV_64F);
	distCoeffs = Mat::zeros(8, 1, CV_64F);
	vector<vector<Point3f> > objectPoints(1);
	vector<Point3f> newObjPoints;
	vector<vector<Point2f> > imagePoints;

	vector<Mat> rvecs, tvecs;

	float grid_width = 1.0f * (s.width - 1);

	calcChessboardCorners(s, 1.0f, objectPoints[0]);
	objectPoints[0][s.width - 1].x = objectPoints[0][0].x + grid_width;
	newObjPoints = objectPoints[0];

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	double rms;
	int iFixedPoint = -1;
	
	rms = calibrateCameraRO(objectPoints, imagePoints, imageSize, iFixedPoint, cameraMatrix, distCoeffs, rvecs, tvecs, newObjPoints, CALIB_FIX_K3 | CALIB_USE_LU);

	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

	objectPoints.clear();
	objectPoints.resize(imagePoints.size(), newObjPoints);
	totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
		rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

	return ok;



	

	


}



int main(){
	// input video from webcam
	//capture_and_show(capture_number);

	Size patternsize(4, 11); //number of centers
	Mat cameraMatrix, distCoeffs;

	Size imageSize;

	calibrate(patternsize, imageSize, cameraMatrix, distCoeffs);
	waitKey();

	/*------ CALIBRATION -------*/
	return 0;
}