#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <stdint.h>
#include <memory>
#include <fstream>
#include <iomanip>

#include <opencv2/opencv.hpp>
#include <kinect.h>
#include "ComPtr.h"

using namespace cv;
using namespace std;

#define ERROR_CHECK( ret )  \
	if ( (ret) != S_OK ) {    \
	std::stringstream ss;	\
	ss << "failed " #ret " " << std::hex << ret << std::endl;			\
	throw std::runtime_error( ss.str().c_str() );			\
	}

namespace kinect{
	int colorWidth;
	int colorHeight;
	int depthWidth;
	int depthHeight;

	IKinectSensor* kinect = nullptr;
	IColorFrameReader* colorFrameReader = nullptr;
	unsigned int colorBytesPerPixel = 4;
	vector<BYTE> colorBuffer;
	unsigned int bufferSize; 
	IDepthFrameReader* depthFrameReader = nullptr;
	vector<UINT16> depthBuffer;

	int minDepth;
	int maxDepth;
	UINT16 minDepthReliableDistance;
	UINT16 maxDepthReliableDistance;

	ICoordinateMapper *coordinateMapper;
	vector<ColorSpacePoint> colorPoints;
	vector<DepthSpacePoint> depthPoints;

	CameraIntrinsics depthcameraIntrinsics;

	//表示用Mat(8bit)
	Mat colorImage;
	Mat colorImage_half;
	Mat depthImage;
	Mat aveDepthImage;

	//保存用Mat(16bit)
	Mat rawBuffer; //多分depthの生データ
	Mat aveDepth;

	std::string colorWinName = "color window";
	std::string depthWinName = "depth window";
	std::string aveDepthWinName = "average depth window";

	int image_idx = 0;
	std::string outdir = "./capture";

	void init();
	void initColorFrame();
	void initDepthFrame();

	void updateColorFrame();
	void updateDepthFrame();

	bool isValidColorFrameRange(float x, float y);
	bool isValidDepthFrameRange(float x, float y);
	bool isValidDepthRange(int index);

	void draw();

	void saveDepth_JPG();
	void saveDepth_PNG();
	void saveColor();


	//Depthの移動平均法による精度向上
    const int nFrame = 100;      // 移動平均を行うフレーム数 30なら30FPSなので１秒分  100フレーム位が良好
    const double Kd = 1 / (double)nFrame; // 移動平均を除算を使わず乗算で求めるための係数
    const int Dx = 512;          // 画面イメージの水平方向画素数
    const int Dy = 424;          // 画面イメージの垂直方向画素数
    const int dByte = 4;         // XRGB形式なので４バイト
    const int dPixels = Dx * Dy; // 1フレーム分の画素数
    int ptr = 0;                 // 移動平均を行う為のデータ格納ポインタ

	//vector<UINT16> DataIn;       // Kinectからデプスを取得するためのバッファ（１フレーム分）[dPixels] = depthBuffer
    vector<UINT16> nDepthBuffer; // nFrame分のデプスバッファ [dPixels * nFrame]
    long *Sum = new long[dPixels]; // nFrame分の移動加算値を格納する為のバッファ[dPixels]

	//平均結果のDepth
	unsigned short* aveDepthData = new ushort[dPixels];
	
	void FIFOFilter();

	void init(){
		try{
			ERROR_CHECK(GetDefaultKinectSensor(&kinect));
			ERROR_CHECK(kinect->Open());
			BOOLEAN isOpen = false;
			ERROR_CHECK(kinect->get_IsOpen(&isOpen));
			if(!isOpen) throw runtime_error("Kinect cannot open.");

			initColorFrame();
			initDepthFrame();

			namedWindow(colorWinName);
			namedWindow(depthWinName);
			namedWindow(aveDepthWinName);

		}catch(exception& ex){
			cout << ex.what() << endl;
		}
	}


	void initColorFrame(){
		ComPtr<IColorFrameSource> colorFrameSource;
		ERROR_CHECK(kinect->get_ColorFrameSource(&colorFrameSource));
		ERROR_CHECK(colorFrameSource->OpenReader(&colorFrameReader));

		ComPtr<IFrameDescription> colorFrameDescription;
		ERROR_CHECK( colorFrameSource->CreateFrameDescription( 
			ColorImageFormat::ColorImageFormat_Bgra, &colorFrameDescription ) );
		ERROR_CHECK( colorFrameDescription->get_Width( &colorWidth ) );
		ERROR_CHECK( colorFrameDescription->get_Height( &colorHeight ) );
		ERROR_CHECK( colorFrameDescription->get_BytesPerPixel( &colorBytesPerPixel ) );

		colorBuffer.resize(colorWidth * colorHeight * colorBytesPerPixel);
	}

	void initDepthFrame(){
		ComPtr<IDepthFrameSource> depthFrameSource;
		ERROR_CHECK( kinect->get_DepthFrameSource( &depthFrameSource ) );
		ERROR_CHECK( depthFrameSource->OpenReader( &depthFrameReader ) );

		ComPtr<IFrameDescription> depthFrameDescription;
		ERROR_CHECK( depthFrameSource->get_FrameDescription( &depthFrameDescription ) );
		ERROR_CHECK( depthFrameDescription->get_Width( &depthWidth ) );
		ERROR_CHECK( depthFrameDescription->get_Height( &depthHeight ) );

		ERROR_CHECK( depthFrameSource->get_DepthMinReliableDistance( &minDepthReliableDistance ) );
		ERROR_CHECK( depthFrameSource->get_DepthMaxReliableDistance( &maxDepthReliableDistance ) );
		minDepth = minDepthReliableDistance;
		maxDepth = maxDepthReliableDistance;

		depthBuffer.resize(depthWidth * depthHeight);
		bufferSize = depthWidth * depthHeight * sizeof( unsigned short );

        for (int i = 0; i < dPixels; i++) { Sum[i] = 0; } //移動加算バッファを０クリア
		nDepthBuffer.resize(dPixels * nFrame);

	}


void updateColorFrame(){
	ComPtr<IColorFrame> colorFrame;
	auto ret = colorFrameReader->AcquireLatestFrame( &colorFrame );
	if ( ret != S_OK ){
		return;
	}

	ERROR_CHECK( colorFrame->CopyConvertedFrameDataToArray(
		colorBuffer.size(), &colorBuffer[0], ColorImageFormat::ColorImageFormat_Bgra ) );

		Mat re(colorHeight, colorWidth, CV_8UC4, &colorBuffer[0]);
		cvtColor(re, colorImage, CV_RGBA2RGB);


}


void saveColor()
{
	std::ostringstream s;
	string filename_jpg, filename_png, filename;

	s.fill('0');
	s << std::right << std::setw(4) << image_idx;
	filename = s.str() + "-c1.png";
	filename_jpg = "colorCamera" + s.str() + ".jpg";
	filename_png = "colorCamera" + s.str() + ".png";

	//JPG
	//cv::imwrite(outdir + "/" + filename_jpg, colorImage);
	cv::imwrite(outdir + "/" + filename, colorImage);
	//PNG
	//cv::imwrite(outdir + "/" + filename_png, colorImage);

}

void saveDepth_JPG()
{
	std::ostringstream s;
	string filename;

	s.fill('0');
	s << std::right << std::setw(4) << image_idx;
	filename = s.str() + "-d1.jpg";

	cv::imwrite(outdir + "/" + filename, depthImage);

}

void saveDepth_PNG()
{
	std::ostringstream s;
	string filename;

	s.fill('0');
	s << std::right << std::setw(4) << image_idx;
	filename = s.str() + "-d1.png";

	//cv::imwrite(outdir + "/" + filename, rawBuffer);
	cv::imwrite(outdir + "/" + filename, aveDepth);

}


void updateDepthFrame(){
	ComPtr<IDepthFrame> depthFrame;
	auto ret = depthFrameReader->AcquireLatestFrame( &depthFrame );
	if ( ret != S_OK ){
		return;
	}

	ERROR_CHECK( depthFrame->CopyFrameDataToArray( depthBuffer.size(), &depthBuffer[0] ) );

	Mat raw(depthHeight, depthWidth, CV_16UC1, (unsigned short*)&depthBuffer[0]);
	raw.convertTo(depthImage, CV_8UC1, 255.0f / 8000, 0.0f); //表示するために8bitに変換する
	rawBuffer = raw.clone(); //保存用16bit

	//imshow("rawDepth", rawBuffer);

//		int center = depthWidth * (depthHeight / 2) + depthWidth/2;
//		cout << "depth["  << center << "]: " << 
//			depthBuffer[center]  << "mm ---- " << getRawDepthValue(depthBuffer[center]) << endl;

	//移動平均
	FIFOFilter();
	//除算->データ格納
	for(int i = 0; i < dPixels; i++)
	{
		aveDepthData[i] = (unsigned short)(Sum[i]*Kd); //移動加算値Sum[i]から移動平均値kを求める
	}


	Mat ave(depthHeight, depthWidth, CV_16UC1, aveDepthData);
	ave.convertTo(aveDepthImage, CV_8UC1, 255.0f / 8000, 0.0f); //表示するために8bitに変換する
	aveDepth = ave.clone(); //保存用16bit

	//imshow("aveDepth", aveDepthImage);
}

void FIFOFilter()
{
    int j = dPixels * ptr;
    for (int i = 0; i < dPixels; i++)
    {
		Sum[i] += (long)depthBuffer[i] - (long)nDepthBuffer[j + i]; // 移動加算値Sum[i]の変化成分のみ修正
		nDepthBuffer[j + i] = depthBuffer[i]; // 新規データDataIn[i]をバッファに格納
		//cout << "Sum[" << i << "]: " << Sum[i] << endl;

    }
    ptr++;
    if (ptr == nFrame) { ptr = 0; } //【バッファポインタ更新】
}

void draw(){
	if(colorImage.data != nullptr) {
		Size half(colorWidth/2, colorHeight/2);
		colorImage_half = colorImage.clone();
		resize(colorImage_half, colorImage_half,half);
		imshow(colorWinName, colorImage_half);
	}
	if(depthImage.data != nullptr) imshow(depthWinName, depthImage);
	if(aveDepthImage.data != nullptr) imshow(aveDepthWinName, aveDepthImage);
}

bool isValidColorFrameRange(float x, float y){
	return ((0 <= x) && (x < colorWidth)) && ((0 <= y) && (y < colorHeight));
}

bool isValidDepthFrameRange(float x, float y){
	return ((0 <= x) && (x < depthWidth)) && ((0 <= y) && (y < depthHeight));
}

bool isValidDepthRange( int index )
{
	return (minDepth <= depthBuffer[index]) && (depthBuffer[index] <= maxDepth);
}

}