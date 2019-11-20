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

void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners)
{
	corners.resize(0);

	for (int i = 0; i < boardSize.height; i++)
		for (int j = 0; j < boardSize.width; j++)
			corners.push_back(Point3f(float((2 * j + i % 2) * squareSize),
				float(i * squareSize), 0));

}


void calibrate2()
{
	int numBoards = 4;
	int numCornersHor = 4;
	int numCornersVer = 11;

	int numSquares = numCornersHor * numCornersVer;
	Size board_sz = Size(numCornersHor, numCornersVer);

	VideoCapture capture = VideoCapture(0);

	vector<vector<Point3f>> object_points;
	vector<vector<Point2f>> image_points;

	vector<Point2f> corners;
	int successes = 0;

	Mat image;
	Mat gray_image;
	capture >> image;

	vector<Point3f> obj;

	float grid_size = 0.03;// 3cm, or whatever

	for (int i = 0; i < board_sz.height; i++)
		for (int j = 0; j < board_sz.width; j++)
			obj.push_back(Point3f(i * grid_size, (2 * j + i % 2) * grid_size, 0.0f));




	//for (int j = 0; j < numSquares; j++)
	//	obj.push_back(Point3f(j / numCornersHor, j % numCornersHor, 0.0f));

	while (successes < numBoards)
	{
		cvtColor(image, gray_image, COLOR_BGR2GRAY);
		bool found = findCirclesGrid(image, board_sz, corners, CALIB_CB_ASYMMETRIC_GRID);

		if (found)
		{
			cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.1));
			drawChessboardCorners(gray_image, board_sz, corners, found);
		}

		imshow("win1", image);
		imshow("win2", gray_image);

		capture >> image;
		int key = waitKey(1);

		if (key == 27)

			return;

		if (key == ' ' && found != 0)
		{
			image_points.push_back(corners);
			object_points.push_back(obj);

			std::cout << "Snap stored!" << std::endl;

			successes++;

			if (successes >= numBoards)
				break;
		}
	}

	float grid_width = 1.0f * (board_sz.width - 1);

	calcChessboardCorners(board_sz, 1.0f, object_points[0]);
	object_points[0][board_sz.width - 1].x = object_points[0][0].x + grid_width;
	//newObjPoints = object_points[0];

	object_points.resize(image_points.size(), object_points[0]);

	destroyWindow("win1");
	destroyWindow("win2");


	Mat intrinsic = Mat(3, 3, CV_32FC1);
	Mat distCoeffs;
	vector<Mat> rvecs;
	vector<Mat> tvecs;

	intrinsic.ptr<float>(0)[0] = 1;
	intrinsic.ptr<float>(1)[1] = 1;

	calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);

	std::cout << intrinsic << std::endl << std::endl << distCoeffs << std::endl;

	Mat imageUndistorted;
	while (1)
	{
		capture >> image;
		undistort(image, imageUndistorted, intrinsic, distCoeffs);

		imshow("win1", image);
		imshow("win2", imageUndistorted);
		waitKey(1);
	}

	capture.release();

	return;
}





//bool calibrate(Size s, Size imageSize, Mat& cameraMatrix, Mat &distCoeffs)
//{
//
//	VideoCapture cap(0);
//	vector<Mat> captures;
//	int capture_count = 0;
//
//	// Check if camera opened successfully
//	if (!cap.isOpened()) {
//		cout << "Error opening video stream" << endl;
//		return;
//	}
//
//	// Capture until circles are recognised
//	for (;;)
//	{
//		Mat frame, gray;
//		cap >> frame;
//		if (frame.empty())
//		{
//			cout << "Error reading webcam frame" << endl;
//			break;
//		}
//
//		int key = waitKey(1);
//
//		imshow("Webcam", frame);
//
//		if (key == 27) break;
//
//		cvtColor(frame, gray, COLOR_BGR2GRAY);
//		vector<Point2f> centers; //this will be filled by the detected centers
//
//		bool patternfound = findCirclesGrid(gray, s, centers, CALIB_CB_ASYMMETRIC_GRID);
//
//		if (patternfound)
//		{
//			drawChessboardCorners(frame, s, Mat(centers), patternfound);
//			imshow("found circles", frame);
//			break;
//		}
//	}
//
//	cap.release();
//
//	/*-                                      */
//
//	cameraMatrix = Mat::eye(3, 3, CV_64F);
//	distCoeffs = Mat::zeros(8, 1, CV_64F);
//	vector<vector<Point3f> > objectPoints(1);
//	vector<Point3f> newObjPoints;
//	vector<vector<Point2f> > imagePoints;
//
//	vector<Mat> rvecs, tvecs;
//
//	float grid_width = 1.0f * (s.width - 1);
//
//	calcChessboardCorners(s, 1.0f, objectPoints[0]);
//	objectPoints[0][s.width - 1].x = objectPoints[0][0].x + grid_width;
//	newObjPoints = objectPoints[0];
//
//	objectPoints.resize(imagePoints.size(), objectPoints[0]);
//
//	double rms;
//	int iFixedPoint = -1;
//	
//	rms = calibrateCameraRO(objectPoints, imagePoints, imageSize, iFixedPoint, cameraMatrix, distCoeffs, rvecs, tvecs, newObjPoints, CALIB_FIX_K3 | CALIB_USE_LU);
//
//	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);
//
//	objectPoints.clear();
//	objectPoints.resize(imagePoints.size(), newObjPoints);
//	//totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
//	//	rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);
//
//	return ok;
//
//
//
//	
//
//	
//
//
//}


//
//int main(){
//	// input video from webcam
//	//capture_and_show(capture_number);
//
//	//Size patternsize(4, 11); //number of centers
//	//Mat cameraMatrix, distCoeffs;
//
//	//Size imageSize;
//
//	//calibrate(patternsize, imageSize, cameraMatrix, distCoeffs);
//
//	calibrate2();
//	waitKey();
//
//	/*------ CALIBRATION -------*/
//	return 0;
//}