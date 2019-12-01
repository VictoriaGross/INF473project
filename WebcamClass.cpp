#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

static enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
static enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

class WebcamClass
{
public:

	// memory
	vector<Mat>* captures;
	Mat cameraMatrix;
	Mat distCoeffs;
	
	

	WebcamClass(vector<Mat> &captures_)
	{
		captures = &captures_;
	}

	void clear()
	{
		captures->clear();
		destroyAllWindows();
	}

	// show undistorted images (needs cameraMatrix and distCoeffs filled by load_params() or calibrate_from_video(args))
	void show_undistorted()
	{
		VideoCapture capture(0);

		Mat image;
		Mat imageUndistorted;

		// Check if camera opened successfully
		if (!capture.isOpened()) {
			cout << "Error opening video stream" << endl;
			return;
		}
		for (;;)
		{
			capture >> image;
			undistort(image, imageUndistorted, cameraMatrix, distCoeffs);

			imshow("Original", image);
			imshow("Undistorted", imageUndistorted);
			int key = waitKey(1);
			if (key == 27)
				break;
		}

		capture.release();
		destroyWindow("Original");
		destroyWindow("Undistorted");
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

			// on SPACE key, take image
			if (key == 32)
			{
				captures->push_back(frame);
				cout << "Frame captured" << endl;
			}

			// on ESC key, close application
			else
				if (key == 27)
					break;

		}

		// 
		cap.release();
		destroyWindow("Webcam");

		// show captured images

		if (show)
		{
			for (int i = 0; i < captures->size(); i++)
			{
				Mat img = captures->at(i);
				string img_name = to_string(i);
				imshow(img_name, img);
			}
		}
	}

	void save_params()
	{
		FileStorage fs("../camera_params.xml", FileStorage::WRITE);

		fs << "intrinsic" << cameraMatrix;
		fs << "distCoeffs" << distCoeffs;

		fs.release();
	}

	void load_params()
	{
		FileStorage fs("../camera_params.xml", FileStorage::READ);

		fs["intrinsic"] >> cameraMatrix;
		fs["distCoeffs"] >> distCoeffs;

		fs.release();

	}


	//void print()
	//{
	//	std::cout << intrinsic << std::endl 
	//		<< distCoeffs << std::endl
	//		<< "Calibrated camera: " << calibrated << std::endl;
	//}


	//Mat pairwise_homography()//(Mat &I1, Mat &I2)
	//{
	//	// recover from memory
	//	Mat& I1 = captures[0];
	//	Mat& I2 = captures[1];

	//	Mat i1 = I1;
	//	Mat i2 = I2;

	//	// keypoints detector
	//	Ptr<ORB> D = ORB::create();
	//	//Ptr<AKAZE> D = AKAZE::create();

	//	vector<KeyPoint> m1, m2;
	//	Mat desc1, desc2;

	//	D->detectAndCompute(i1, Mat(), m1, desc1);
	//	D->detectAndCompute(i2, Mat(), m2, desc2);

	//	// simple k-NN matcher
	//	BFMatcher M(NORM_L2/*,true*/);

	//	vector<DMatch> matches;
	//	M.match(desc1, desc2, matches);

	//	vector<Point2f> matches1, matches2;

	//	for (int i = 0; i < matches.size(); i++) { // ad only nearest-neighbor
	//		matches1.push_back(m1[matches[i].queryIdx].pt);
	//		matches2.push_back(m2[matches[i].trainIdx].pt);
	//	}

	//	Mat mask;
	//	Mat H = findHomography(matches1, matches2, RANSAC, 3, mask);

	//	// inlier computation
	//	vector<DMatch> inliers;
	//	for (int i = 0; i < matches.size(); i++)
	//		if (mask.at<uchar>(i, 0) != 0)
	//			inliers.push_back(matches[i]);


	//	for (int i = 0; i < inliers.size(); i++)
	//	{
	//		cam0pnts.push_back(m1[inliers[i].queryIdx].pt);
	//		cam1pnts.push_back(m2[inliers[i].trainIdx].pt);
	//	}

	//	return H;
	//}

	//Mat projected_points(Mat H, vector<Point2f> cam0pnts, vector<Point2f> cam1pnts)
	//{
	//	Mat proj1(3, 4, CV_32FC1);
	//	Mat proj2(3, 4, CV_32FC1);

	//	vector<Mat> Rs, Ts;
	//	decomposeHomographyMat(H, intrinsic, Rs, Ts, noArray()); // buggg

	//	for (int i = 0; i < proj1.rows; i++)
	//	{
	//		for (int j = 0; j < proj1.cols; j++)
	//		{
	//			if (i == j)
	//			{
	//				proj1.at<float>(i, j) = 1.0;
	//			}
	//			if (j < proj1.cols - 1)
	//				proj2.at<float>(i, j) = Rs[0].at<float>(i, j);
	//			else
	//				proj2.at<float>(i, j) = Ts[0].at<float>(i, 0);
	//		}
	//	}
	//	
	//	


	//	// You fill them, both with the same size...

	//	// You can pick any of the following 2 (your choice)
	//	// cv::Mat pnts3D(1,cam0pnts.size(),CV_64FC4);
	//	Mat pnts3D(4, cam0pnts.size(), CV_64F);

	//	triangulatePoints(proj1, proj2, cam0pnts, cam1pnts, pnts3D);

	//	cam0pnts.clear();
	//	cam1pnts.clear();

	//	return pnts3D;

	//	
	//	
	//}


	// functions from samples/cpp/calibration.cpp
	double computeReprojectionErrors(
		const vector<vector<Point3f> >& objectPoints,
		const vector<vector<Point2f> >& imagePoints,
		const vector<Mat>& rvecs, const vector<Mat>& tvecs,
		const Mat& cameraMatrix, const Mat& distCoeffs,
		vector<float>& perViewErrors)
	{
		vector<Point2f> imagePoints2;
		int i, totalPoints = 0;
		double totalErr = 0, err;
		perViewErrors.resize(objectPoints.size());

		for (i = 0; i < (int)objectPoints.size(); i++)
		{
			projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
				cameraMatrix, distCoeffs, imagePoints2);
			err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);
			int n = (int)objectPoints[i].size();
			perViewErrors[i] = (float)sqrt(err * err / n);
			totalErr += err * err;
			totalPoints += n;
		}

		return sqrt(totalErr / totalPoints);
	}

	void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType = CHESSBOARD)
	{
		corners.resize(0);

		switch (patternType)
		{
		case CHESSBOARD:
		case CIRCLES_GRID:
			for (int i = 0; i < boardSize.height; i++)
				for (int j = 0; j < boardSize.width; j++)
					corners.push_back(Point3f(float(j * squareSize),
						float(i * squareSize), 0));
			break;

		case ASYMMETRIC_CIRCLES_GRID:
			for (int i = 0; i < boardSize.height; i++)
				for (int j = 0; j < boardSize.width; j++)
					corners.push_back(Point3f(float((2 * j + i % 2) * squareSize),
						float(i * squareSize), 0));
			break;

		default:
			CV_Error(Error::StsBadArg, "Unknown pattern type\n");
		}
	}

	bool runCalibration(vector<vector<Point2f> > imagePoints,
		Size imageSize, Size boardSize, Pattern patternType,
		float squareSize, float aspectRatio,
		float grid_width, bool release_object,
		int flags, Mat& cameraMatrix, Mat& distCoeffs,
		vector<Mat>& rvecs, vector<Mat>& tvecs,
		vector<float>& reprojErrs,
		vector<Point3f>& newObjPoints,
		double& totalAvgErr)
	{
		cameraMatrix = Mat::eye(3, 3, CV_64F);
		if (flags & CALIB_FIX_ASPECT_RATIO)
			cameraMatrix.at<double>(0, 0) = aspectRatio;

		distCoeffs = Mat::zeros(8, 1, CV_64F);

		vector<vector<Point3f> > objectPoints(1);
		calcChessboardCorners(boardSize, squareSize, objectPoints[0], patternType);
		objectPoints[0][boardSize.width - 1].x = objectPoints[0][0].x + grid_width;
		newObjPoints = objectPoints[0];

		objectPoints.resize(imagePoints.size(), objectPoints[0]);

		double rms;
		int iFixedPoint = -1;
		if (release_object)
			iFixedPoint = boardSize.width - 1;
		rms = calibrateCameraRO(objectPoints, imagePoints, imageSize, iFixedPoint,
			cameraMatrix, distCoeffs, rvecs, tvecs, newObjPoints,
			flags | CALIB_FIX_K3 | CALIB_USE_LU);
		printf("RMS error reported by calibrateCamera: %g\n", rms);

		bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

		if (release_object) {
			cout << "New board corners: " << endl;
			cout << newObjPoints[0] << endl;
			cout << newObjPoints[boardSize.width - 1] << endl;
			cout << newObjPoints[boardSize.width * (boardSize.height - 1)] << endl;
			cout << newObjPoints.back() << endl;
		}

		objectPoints.clear();
		objectPoints.resize(imagePoints.size(), newObjPoints);
		totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
			rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

		return ok;
	}

	bool readStringList(const string& filename, vector<string>& l)
	{
		l.resize(0);
		FileStorage fs(filename, FileStorage::READ);
		if (!fs.isOpened())
			return false;
		size_t dir_pos = filename.rfind('/');
		if (dir_pos == string::npos)
			dir_pos = filename.rfind('\\');
		FileNode n = fs.getFirstTopLevelNode();
		if (n.type() != FileNode::SEQ)
			return false;
		FileNodeIterator it = n.begin(), it_end = n.end();
		for (; it != it_end; ++it)
		{
			string fname = (string)*it;
			if (dir_pos != string::npos)
			{
				string fpath = samples::findFile(filename.substr(0, dir_pos + 1) + fname, false);
				if (fpath.empty())
				{
					fpath = samples::findFile(fname);
				}
				fname = fpath;
			}
			else
			{
				fname = samples::findFile(fname);
			}
			l.push_back(fname);
		}
		return true;
	}

	bool runAndSave(const string& outputFilename,
		const vector<vector<Point2f> >& imagePoints,
		Size imageSize, Size boardSize, Pattern patternType, float squareSize,
		float grid_width, bool release_object,
		float aspectRatio, int flags, Mat& cameraMatrix,
		Mat& distCoeffs, bool writeExtrinsics, bool writePoints, bool writeGrid)
	{
		vector<Mat> rvecs, tvecs;
		vector<float> reprojErrs;
		double totalAvgErr = 0;
		vector<Point3f> newObjPoints;

		bool ok = runCalibration(imagePoints, imageSize, boardSize, patternType, squareSize,
			aspectRatio, grid_width, release_object, flags, cameraMatrix, distCoeffs,
			rvecs, tvecs, reprojErrs, newObjPoints, totalAvgErr);
		printf("%s. avg reprojection error = %.7f\n",
			ok ? "Calibration succeeded" : "Calibration failed",
			totalAvgErr);

		if (ok)
			save_params();
		return ok;
	}
	
	void calibrate_from_video(Pattern pattern_, int size_w, int size_h, int frames_)
	{
		Size boardSize, imageSize;
		float squareSize = 1.0;
		float aspectRatio = 1.0;
		string outputFilename;
		string inputFilename = "";

		int i, nframes;
		bool writeExtrinsics, writePoints;
		bool undistortImage = false;
		int flags = 0;
		VideoCapture capture;
		bool flipVertical = false;
		bool showUndistorted = false;
		int delay;
		clock_t prevTimestamp = 0;
		int mode = DETECTION;
		int cameraId = 0;
		vector<vector<Point2f> > imagePoints;
		vector<string> imageList;
		Pattern pattern = pattern_;

		boardSize.width = size_w;
		boardSize.height = size_h;
		nframes = frames_;
		delay = 100;
		writePoints = false;
		writeExtrinsics = true;
		bool writeGrid = false;
		flags |= CALIB_FIX_PRINCIPAL_POINT;
		outputFilename = "../camera_params.xml";
		int winSize = 11;
		float grid_width = squareSize * (boardSize.width - 1);
		bool release_object = false;

		// beginning of calibration

		capture.open(cameraId);

		if (!capture.isOpened() && imageList.empty())
			cerr << "Could not initialize video capture" << endl;
		if (!imageList.empty())
			nframes = (int)imageList.size();

		namedWindow("Image View", 1);

		for (i = 0;; i++)
		{
			Mat view, viewGray;
			bool blink = false;

			if (capture.isOpened())
			{
				Mat view0;
				capture >> view0;
				view0.copyTo(view);
			}
			else if (i < (int)imageList.size())
				view = imread(imageList[i], 1);

			if (view.empty())
			{
				if (imagePoints.size() > 0)
					runAndSave(outputFilename, imagePoints, imageSize,
						boardSize, pattern, squareSize, grid_width, release_object, aspectRatio,
						flags, cameraMatrix, distCoeffs,
						writeExtrinsics, writePoints, writeGrid);
				break;
			}

			imageSize = view.size();

			if (flipVertical)
				flip(view, view, 0);

			vector<Point2f> pointbuf;
			cvtColor(view, viewGray, COLOR_BGR2GRAY);

			bool found;
			switch (pattern)
			{
			case CHESSBOARD:
				found = findChessboardCorners(view, boardSize, pointbuf,
					CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
				break;
			case CIRCLES_GRID:
				found = findCirclesGrid(view, boardSize, pointbuf);
				break;
			case ASYMMETRIC_CIRCLES_GRID:
				found = findCirclesGrid(view, boardSize, pointbuf, CALIB_CB_ASYMMETRIC_GRID);
				break;
			default:
				cerr << "Unknown pattern type\n";
			}

			// improve the found corners' coordinate accuracy
			if (pattern == CHESSBOARD && found) cornerSubPix(viewGray, pointbuf, Size(winSize, winSize),
				Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.0001));

			if (mode == CAPTURING && found &&
				(!capture.isOpened() || clock() - prevTimestamp > delay * 1e-3 * CLOCKS_PER_SEC))
			{
				imagePoints.push_back(pointbuf);
				prevTimestamp = clock();
				blink = capture.isOpened();
			}

			if (found)
				drawChessboardCorners(view, boardSize, Mat(pointbuf), found);

			string msg = mode == CAPTURING ? "100/100" :
				mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
			int baseLine = 0;
			Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
			Point textOrigin(view.cols - 2 * textSize.width - 10, view.rows - 2 * baseLine - 10);

			if (mode == CAPTURING)
			{
				if (undistortImage)
					msg = format("%d/%d Undist", (int)imagePoints.size(), nframes);
				else
					msg = format("%d/%d", (int)imagePoints.size(), nframes);
			}

			putText(view, msg, textOrigin, 1, 1,
				mode != CALIBRATED ? Scalar(0, 0, 255) : Scalar(0, 255, 0));

			if (blink)
				bitwise_not(view, view);

			if (mode == CALIBRATED && undistortImage)
			{
				Mat temp = view.clone();
				undistort(temp, view, cameraMatrix, distCoeffs);
			}

			imshow("Image View", view);
			char key = (char)waitKey(capture.isOpened() ? 50 : 500);

			if (key == 27)
				break;

			if (key == 'u' && mode == CALIBRATED)
				undistortImage = !undistortImage;

			if (capture.isOpened() && key == 'g')
			{
				mode = CAPTURING;
				imagePoints.clear();
			}

			if (mode == CAPTURING && imagePoints.size() >= (unsigned)nframes)
			{
				if (runAndSave(outputFilename, imagePoints, imageSize,
					boardSize, pattern, squareSize, grid_width, release_object, aspectRatio,
					flags, cameraMatrix, distCoeffs,
					writeExtrinsics, writePoints, writeGrid))
					mode = CALIBRATED;
				else
					mode = DETECTION;
				if (!capture.isOpened())
					break;
			}

			// Stop once calibration is finished
			if (mode == CALIBRATED)
				break;
		}

		capture.release();
		destroyWindow("Image View");
		cout << "Webcam is now calibrated\n";
	}
	
private:
	

};