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

void calibrateTarget(VideoCapture cap){
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
		circle(frame1, center, 3, Scalar(0,255,0), -1, 8, 0);
		circle(frame1, center, radius, Scalar(0,0,255), 3, 8, 0);
	}
        
    namedWindow("edges",1);
    for(;;)
    { 
        imshow("edges", frame1); //frame is captured, edge is edited Map(edges detection)
        if(waitKey(30) >= 0) break;
    }
}

void destroyer(){
	gpioTerminate();
}

int main(int, char**)
{
	
	setup();
	
	VideoCapture cap(0); // open the default camera
    if(!cap.isOpened()){  // check if we succeeded
        return -1;
	}
    
    calibrateTarget(cap);
    
	
	
	destroyer();
    return 0;
}
