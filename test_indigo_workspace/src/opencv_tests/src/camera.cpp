#include "opencv2/highgui/highgui.hpp"
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char **argv) {

	VideoCapture cap(0);

	if(!cap.isOpened()) {
		cout << "Cannot open the video file" << endl;
		return -1;
	}

	double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
	double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

	namedWindow("MyVideo", CV_WINDOW_AUTOSIZE);

	for(;;) {
		Mat frame;
		bool bSuccess = cap.read(frame);

		if(!bSuccess) {
			break;
		}

		imshow("MyVideo", frame);

		if(waitKey(30) == 27) {
			cout << "Esc pressed" << endl;
			break;
		}
	}
	return 0;
} // End function main()
