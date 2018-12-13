#include <pigpio.h>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <math.h>
using namespace std;
using namespace cv;
#define LED 23
#define WIDTH 500
#define HEIGHT 500

Vec3f lastDetectedHole;

void displayInfo(string data) {
	cout << data << endl;
}

void setup() {
	displayInfo("Konfigurowanie programu");
	gpioInitialise();
	gpioSetMode(23, PI_OUTPUT);
}

void turnLed(bool status) {
	gpioWrite(LED, status);
}

vector<Vec3f> sortEdgePoints(vector<Vec3f> input) {
	vector<Vec3f> output;
	double max = 0;
	int maxid;
	double min = 2500;
	int minid;
	double sum;
	int tmp;
	for (int i = 0; i < 4; i++) {
		sum = input[i][0] + input[i][1];
		if (sum > max) {
			max = sum;
			maxid = i;
		}
		if (sum < min) {
			min = sum;
			minid = i;
		}
	}
	//left=top point
	output.push_back(input[minid]);
	for (int i = 0; i < 4; i++) {
		if (i == minid || i == maxid) {
			continue;
		}
		if (input[i][0] > input[minid][0] && input[i][1] < input[maxid][1]) {
			//right-top point
			output.push_back(input[i]);
		}
		else {
			//left-bottom
			tmp = i;
		}
	}
	output.push_back(input[maxid]);
	output.push_back(input[tmp]);
	return output;

}

vector<Vec3f> calibrateTarget(VideoCapture cap) {
	Mat frame1;
	Mat frame2;
	Mat diff;
	vector<Vec3f> circles;

	do {
		turnLed(true);
		usleep(1000000);
		cap >> frame1;
		turnLed(false);
		usleep(1000000);
		cap >> frame2;
		absdiff(frame1, frame2, diff);

		cvtColor(diff, diff, COLOR_BGR2GRAY);
		GaussianBlur(diff, diff, Size(7, 7), 1.5, 1.5);
		Canny(diff, diff, 0, 30, 3);

		GaussianBlur(diff, diff, Size(7, 7), 4, 4);
		HoughCircles(diff, circles, CV_HOUGH_GRADIENT, 1, 100, 30, 20, 10, 20);
		displayInfo("Kalibrowanie tarczy, wykrytych punktów: " + to_string(circles.size()));
	} while (circles.size() != 4);

	Point c(200, 100);
	int r = 50;

	circles = sortEdgePoints(circles);

	//Display detected circles
	for (size_t i = 0; i < circles.size(); i++) {
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		if (i == 1) {
			//yellow
			circle(frame1, center, 3, Scalar(0, 255, 255), -1, 8, 0);
		}
		if (i == 2) {
			//red
			circle(frame1, center, 3, Scalar(0, 0, 255), -1, 8, 0);
		}
		if (i == 3) {
			//blue
			circle(frame1, center, 3, Scalar(255, 0, 0), -1, 8, 0);
		}
		circle(frame1, center, radius, Scalar(0, 0, 255), 3, 8, 0);
	}

	namedWindow("target", 1);
	for (;;)
	{
		imshow("target", frame1); //frame is captured, edge is edited Map(edges detection)
		if (waitKey(30) >= 0) break;
	}


	return circles;
	//points order: top-left, top-right, bottom-right, bottom-left (clockwise)
}

Mat trasnformImage(Mat src, vector<Vec3f> points) {
	Mat dst = Mat(HEIGHT, WIDTH, src.type());
	Point2f inputQuad[4];
	Point2f outputQuad[4];
	Mat lambda(2, 4, CV_32FC1);
	lambda = Mat::zeros(src.rows, src.cols, src.type());

	inputQuad[0] = Point2f(points[0][0], points[0][1]);
	inputQuad[1] = Point2f(points[1][0], points[1][1]);
	inputQuad[2] = Point2f(points[2][0], points[2][1]);
	inputQuad[3] = Point2f(points[3][0], points[3][1]);

	outputQuad[0] = Point2f(0, 0);
	outputQuad[1] = Point2f(WIDTH - 1, 0);
	outputQuad[2] = Point2f(WIDTH - 1, HEIGHT - 1);
	outputQuad[3] = Point2f(0, HEIGHT - 1);


	lambda = getPerspectiveTransform(inputQuad, outputQuad);
	warpPerspective(src, dst, lambda, dst.size());

	//Display image       

	namedWindow("target", 1);
	for (;;)
	{
		imshow("target", dst); //frame is captured, edge is edited Map(edges detection)
		if (waitKey(30) >= 0) break;
	}

	return dst;
}

bool FindHole(Mat frame) {
	vector<Vec3f> circles;

	Mat org = frame; //this var is for presentation purpose only!

	cvtColor(frame, frame, COLOR_BGR2GRAY);
	GaussianBlur(frame, frame, Size(7, 7), 1.5, 1.5);
	Canny(frame, frame, 0, 30, 3);

	GaussianBlur(frame, frame, Size(7, 7), 4, 4);
	HoughCircles(frame, circles, CV_HOUGH_GRADIENT, 1, 100, 30, 20, 10, 20);

	//Display detected circles
	for (size_t i = 0; i < circles.size(); i++) {
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		circle(org, center, 3, Scalar(0, 255, 0), -1, 8, 0);
		circle(org, center, radius, Scalar(0, 0, 255), 3, 8, 0);
	}

	namedWindow("target", 1);
	for (;;)
	{
		imshow("target", org);
		if (waitKey(30) >= 0) break;
	}


	if (circles.size() > 0) {
		lastDetectedHole = circles[0];
		return true;
	}
	else {
		return false;
	}

}

double CalculateValue(double radius) {
	return 10.0;
}

vector<double> CalculateHoleCoords() {
	double radius;
	double sinus;
	double quarter;
	double center_X = WIDTH / 2;
	double center_Y = HEIGHT / 2;


	//obliczanie promienia
	radius = sqrt(pow(lastDetectedHole[0] - center_X,2) + pow(lastDetectedHole[1] - center_Y,2));

		if (radius > 0) {
			//ustalenie ćwiartki
			if (lastDetectedHole[0] > center_X && lastDetectedHole[1] < center_Y) {
				quarter = 1;
			}
			if (lastDetectedHole[0] < center_X && lastDetectedHole[1] < center_Y) {
				quarter = 2;
			}
			if (lastDetectedHole[0] < center_X && lastDetectedHole[1] > center_Y) {
				quarter = 3;
			}
			if (lastDetectedHole[0] > center_X && lastDetectedHole[1] > center_Y) {
				quarter = 4;
			}
			
			//obliczanie sinusa
			sinus = abs(lastDetectedHole[1] - center_Y) / radius;
		} else {
			sinus = 0;
			quarter = 1;
		}

	double value = CalculateValue(radius);

	vector<double> result;
	result.push_back(value);
	result.push_back(radius);
	result.push_back(sinus);
	result.push_back(quarter);

	return result;
}

void destroyer() {
	gpioTerminate();
}

int main(int, char**)
{
	vector<Vec3f> cornerPoints;
	Mat emptyTarget;
	Mat actualFrame;
	Mat frame;

	setup();

	VideoCapture cap(0); // open the default camera
	if (!cap.isOpened()) {  // check if we "succeeded
		displayInfo("Opening camera error");
		return -1;
	}

	cornerPoints = calibrateTarget(cap);
	//TODO make sure points are in counterclock order!
	cap >> frame;
	emptyTarget = trasnformImage(frame, cornerPoints);

	do {
		cap >> frame;
		actualFrame = trasnformImage(frame, cornerPoints);
		if (FindHole(actualFrame)) {
			cout << lastDetectedHole[0] << endl << lastDetectedHole[1] << endl;
			//TODO wywołaj calculateHoleCoords()
		}

	} while (true);



	destroyer();
	return 0;
}
