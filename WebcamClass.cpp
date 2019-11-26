#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

class WebcamClass
{
public:

	// memory
	vector<Mat> captures;

	WebcamClass(Mat &intrinsic_, Mat& distCoeffs_, int calib_samples_, int calib_x, int calib_y, vector<Mat> &captures_)
	{
		intrinsic = intrinsic_;
		distCoeffs = distCoeffs_;
		calib_samples = calib_samples_;
		board_sz = Size(calib_x, calib_y);
		calibrated = false;
		captures = captures_;

		R = Mat::eye(3, 3, CV_32FC1);
		t = Mat::zeros(3, 1, CV_32FC1);

	}

	void calibrate()
	{
		int numBoards = calib_samples;
		int numCornersHor = board_sz.width;
		int numCornersVer = board_sz.height;

		int numSquares = numCornersHor * numCornersVer;
		Size board_sz = Size(numCornersHor, numCornersVer);

		VideoCapture capture(0);

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


		//vector<Point3f> newObjPoints;
		//calcChessboardCorners(board_sz, 1.0f, object_points[0]);
		//object_points[0][board_sz.width - 1].x = object_points[0][0].x + grid_width;
		//newObjPoints = object_points[0];

		//object_points.resize(image_points.size(), object_points[0]);

		destroyWindow("win1");
		destroyWindow("win2");


		intrinsic = Mat(3, 3, CV_32FC1);
		vector<Mat> rvecs;
		vector<Mat> tvecs;

		intrinsic.ptr<float>(0)[0] = 1;
		intrinsic.ptr<float>(1)[1] = 1;

		calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);


		//int iFixedPoint = -1;
		//calibrateCameraRO(object_points, image_points, board_sz, iFixedPoint, intrinsic, distCoeffs, rvecs, tvecs, newObjPoints, CALIB_FIX_K3 | CALIB_USE_LU);


		std::cout << intrinsic << std::endl << std::endl << distCoeffs << std::endl;

		Mat imageUndistorted;
		for (;;) 
		{
			capture >> image;
			undistort(image, imageUndistorted, intrinsic, distCoeffs);

			imshow("win1", image);
			imshow("win2", imageUndistorted);
			int key = waitKey(1);
			if (key == 27)
				break;
		}

		capture.release();

		calibrated = true;
	}

	void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners)
	{
		corners.resize(0);

		for (int i = 0; i < boardSize.height; i++)
			for (int j = 0; j < boardSize.width; j++)
				corners.push_back(Point3f(float((2 * j + i % 2) * squareSize),
					float(i * squareSize), 0));

	}

	void capture_and_show(bool show)
	{
		VideoCapture cap(0);

		// Check if camera opened successfully
		if (!cap.isOpened()) {
			cout << "Error opening video stream" << endl;
			return;
		}


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
				captures.push_back(frame);

			// on ESC key, close application
			else 
				if (key == 27) 
					break;

		}

		// after capture_number images taken, we stop image capturing
		cap.release();
		destroyWindow("Webcam");

		// show captured images

		if (show)
		{
			for (int i = 0; i < captures.size(); i++)
			{
				Mat img = captures[i];
				string img_name = to_string(i);
				imshow(img_name, img);
			}
		}
	}

	void save_params()
	{
		FileStorage fs("../camera_params.xml", FileStorage::WRITE);
		
		fs << "intrinsic" << intrinsic;
		fs << "distCoeffs" << distCoeffs;
		
		fs.release();
	}

	void load_params()
	{
		FileStorage fs("../camera_params.xml", FileStorage::READ);

		fs["intrinsic"] >> intrinsic;
		fs["distCoeffs"] >> distCoeffs;

		fs.release();

		calibrated = true;
	}

	void print()
	{
		std::cout << intrinsic << std::endl 
			<< distCoeffs << std::endl
			<< "Calibrated camera: " << calibrated << std::endl;
	}

	void set_calibration(bool c)
	{
		calibrated = c;
	}

	Mat pairwise_homography()//(Mat &I1, Mat &I2)
	{

		Mat &I1 = captures[0];
		Mat &I2 = captures[1];

		// keypoints detector
		Ptr<ORB> D = ORB::create();
		//Ptr<AKAZE> D = AKAZE::create();

		vector<KeyPoint> m1, m2;
		Mat desc1, desc2;

		D->detectAndCompute(I1, Mat(), m1, desc1);
		D->detectAndCompute(I2, Mat(), m2, desc2);

		// simple 2-NN matcher
		BFMatcher M(NORM_L2/*,true*/);

		vector<DMatch> matches;
		M.match(desc1, desc2, matches);

		vector<Point2f> matches1, matches2;

		Mat H;

		for (int i = 0; i < matches.size(); i++) {
			matches1.push_back(m1[matches[i].queryIdx].pt);
			matches2.push_back(m2[matches[i].trainIdx].pt);
		}

		Mat mask; // Inliers?

		H = findHomography(matches1, matches2, RANSAC, 3, mask);
		//decomposeHomographyMat(H, intrinsic, R, t, noArray()); // BUG

		return H;

		//Mat proj1(3, 4, CV_32FC1);
		//Mat proj2(3, 4, CV_32FC1);

		//for (int i = 0; i < proj1.rows; i++)
		//{
		//	for (int j = 0; j < proj1.cols; j++)
		//	{
		//		if (i == j)
		//		{
		//			proj1.at<float>(i, j) = 1.0;
		//		}
		//		if (j < proj1.cols - 1)
		//			proj2.at<float>(i, j) = R.at<float>(i, j);
		//		else
		//			proj2.at<float>(i, j) = t.at<float>(i, 0);
		//	}
		//}

		//Mat points3d;

		//triangulatePoints(proj1, proj2, matches1, matches2, points3d);

		//std::cout << points3d << std::endl;

		//return H;
	}

	//void mix(Mat &R, Mat &t, Mat &result)
	//{
	//	for (int j = 0; j < R.cols; j++)
	//	{
	//		for(int i = 0; i < R.rows; i++)
	//		{
	//		
	//		}
	//	}
	//}
	
private:
	// general parameters
	Mat intrinsic;
	Mat distCoeffs;
	bool calibrated;

	// for calibration
	int calib_samples;
	Size board_sz;

	Mat R;
	Mat t;
	Mat n;


};