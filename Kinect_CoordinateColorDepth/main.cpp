#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string>

#include <opencv2/opencv.hpp>
#include "myKinect.h"

int main(int argc, char *argv[]){

	kinect::init();
	int c;

	//cout << "10•bŒã‚©‚çŠJŽn‚µ‚Ü‚·..." << endl;
	//Sleep(10000);

	while(true){
		kinect::updateColorFrame();
		kinect::updateDepthFrame();
		//kinect::updateIRFrame();
		//kinect::coordinateDepthColor();
		//kinect::coordinateColorDepth();

		kinect::draw();

		//10•b–ˆ‚ÉŽB‰e
		//Sleep(10000);
		//cout << kinect::image_idx << "–‡–Ú" << endl;
 	//	kinect::saveColor();
		//kinect::saveDepth_PNG();

		//kinect::image_idx++;


		c = waitKey(30);
		if(c == 27) break; //esc key
		if(c == 32) //space key
		{

 			kinect::saveColor();
			kinect::saveDepth_PNG();

			kinect::image_idx++;

			cout << kinect::image_idx << endl;

		}

	}
	exit(0);
}