#include <pigpio.h>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
using namespace std;
using namespace cv;
#define LED 23

void displayInfo(string data){
	cout << data << endl;
}

void setup(){
	displayInfo("Konfigurowanie programu");
	gpioInitialise();
	gpioSetMode(23, PI_OUTPUT);
}

void turnLed(bool status){
	gpioWrite(LED, status);
}

vector<Vec3f> calibrateTarget(VideoCapture cap){
	Mat frame1;
	Mat frame2;
	Mat diff;
	vector<Vec3f> circles;
	
	do{
	turnLed(true);
	usleep(1000000);
	cap >> frame1;
	turnLed(false);
	usleep(1000000);
	cap >> frame2;
	absdiff(frame1, frame2, diff);
	
	cvtColor(diff, diff, COLOR_BGR2GRAY);
    GaussianBlur(diff, diff, Size(7,7), 1.5, 1.5);
    Canny(diff, diff, 0, 30, 3);
        
    GaussianBlur(diff, diff, Size(7,7), 4,4);
	HoughCircles(diff, circles, CV_HOUGH_GRADIENT, 1, 100, 30, 20, 10, 20);
	displayInfo("Kalibrowanie tarczy, wykrytych punktów: "+ to_string(circles.size()));
	}while(circles.size()!=4);
	
	//Display detected circles
	for(size_t i = 0;i<circles.size(); i++){
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		if(i ==1 ){
			//yellow
			circle(frame1, center, 3, Scalar(0,255,255), -1, 8, 0);
		}
		if(i ==2 ){
			//red
			circle(frame1, center, 3, Scalar(0,0,255), -1, 8, 0);
		}
		if(i ==3 ){
			//blue
			circle(frame1, center, 3, Scalar(255,0,0), -1, 8, 0);
		}
		circle(frame1, center, radius, Scalar(0,0,255), 3, 8, 0);
	}
        
    namedWindow("edges",1);
    for(;;)
    { 
        imshow("edges", frame1); //frame is captured, edge is edited Map(edges detection)
        if(waitKey(30) >= 0) break;
    }


    return circles;
    //points order: top-left, top-right, bottom-right, bottom-left (clockwise)
}

Mat trasnformImage(Mat src, vector<Vec3f> points){
	Mat dst = src;
	Point2f inputQuad[4];
	Point2f outputQuad[4];
	Mat lambda(2,4, CV_32FC1);
	lambda = Mat::zeros(src.rows, src.cols, src.type());
	
	inputQuad[0] = Point2f(points[0][0], points[0][1]);
	inputQuad[1] = Point2f(points[1][0], points[1][1]);
	inputQuad[2] = Point2f(points[2][0], points[2][1]);
	inputQuad[3] = Point2f(points[3][0], points[3][1]);
	
	outputQuad[0] = Point2f(0,0);
	outputQuad[1] = Point2f(src.cols-1,0);
	outputQuad[2] = Point2f(src.cols-1,src.rows-1);
	outputQuad[3] = Point2f(0,src.rows-1);

	
	lambda = getPerspectiveTransform(inputQuad, outputQuad);
	warpPerspective(src, dst, lambda, dst.size());
	
	//Display image       
    namedWindow("edges",1);
    for(;;)
    { 
        imshow("edges", dst); //frame is captured, edge is edited Map(edges detection)
        if(waitKey(30) >= 0) break;
    }
    
    return dst;
}

void destroyer(){
	gpioTerminate();
}

int main(int, char**)
{
	vector<Vec3f> cornerPoints;
	Mat emptyTarget;
	Mat frame;
	
	setup();
	
	VideoCapture cap(0); // open the default camera
    if(!cap.isOpened()){  // check if we succeeded
        return -1;
	}
    
    cornerPoints = calibrateTarget(cap);
    cap >> frame;
    emptyTarget = trasnformImage(frame, cornerPoints);
    
	
	
	destroyer();
    return 0;
}
