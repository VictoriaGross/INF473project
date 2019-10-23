#include "opencv2/opencv.hpp"
#include <iostream>

using namespace std;
using namespace cv;

const int capture_number = 2;

int main(){
	// input video from webcam
	VideoCapture cap(0);

	// Check if camera opened successfully
	if (!cap.isOpened()) {
		cout << "Error opening video stream" << endl;
		return -1;
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
			imshow("Capture", frame);
		}
		// on ESC key, close application
		else if(key == 27 || capture_count == capture_number) break;

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

	waitKey();
	return 0;
}