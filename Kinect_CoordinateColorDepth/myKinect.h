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

	//�\���pMat(8bit)
	Mat colorImage;
	Mat colorImage_half;
	Mat depthImage;
	Mat aveDepthImage;

	//�ۑ��pMat(16bit)
	Mat rawBuffer; //����depth�̐��f�[�^
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


	//Depth�̈ړ����ϖ@�ɂ�鐸�x����
    const int nFrame = 100;      // �ړ����ς��s���t���[���� 30�Ȃ�30FPS�Ȃ̂łP�b��  100�t���[���ʂ��ǍD
    const double Kd = 1 / (double)nFrame; // �ړ����ς����Z���g�킸��Z�ŋ��߂邽�߂̌W��
    const int Dx = 512;          // ��ʃC���[�W�̐���������f��
    const int Dy = 424;          // ��ʃC���[�W�̐���������f��
    const int dByte = 4;         // XRGB�`���Ȃ̂łS�o�C�g
    const int dPixels = Dx * Dy; // 1�t���[�����̉�f��
    int ptr = 0;                 // �ړ����ς��s���ׂ̃f�[�^�i�[�|�C���^

	//vector<UINT16> DataIn;       // Kinect����f�v�X���擾���邽�߂̃o�b�t�@�i�P�t���[�����j[dPixels] = depthBuffer
    vector<UINT16> nDepthBuffer; // nFrame���̃f�v�X�o�b�t�@ [dPixels * nFrame]
    long *Sum = new long[dPixels]; // nFrame���̈ړ����Z�l���i�[����ׂ̃o�b�t�@[dPixels]

	//���ό��ʂ�Depth
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

        for (int i = 0; i < dPixels; i++) { Sum[i] = 0; } //�ړ����Z�o�b�t�@���O�N���A
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
	raw.convertTo(depthImage, CV_8UC1, 255.0f / 8000, 0.0f); //�\�����邽�߂�8bit�ɕϊ�����
	rawBuffer = raw.clone(); //�ۑ��p16bit

	//imshow("rawDepth", rawBuffer);

//		int center = depthWidth * (depthHeight / 2) + depthWidth/2;
//		cout << "depth["  << center << "]: " << 
//			depthBuffer[center]  << "mm ---- " << getRawDepthValue(depthBuffer[center]) << endl;

	//�ړ�����
	FIFOFilter();
	//���Z->�f�[�^�i�[
	for(int i = 0; i < dPixels; i++)
	{
		aveDepthData[i] = (unsigned short)(Sum[i]*Kd); //�ړ����Z�lSum[i]����ړ����ϒlk�����߂�
	}


	Mat ave(depthHeight, depthWidth, CV_16UC1, aveDepthData);
	ave.convertTo(aveDepthImage, CV_8UC1, 255.0f / 8000, 0.0f); //�\�����邽�߂�8bit�ɕϊ�����
	aveDepth = ave.clone(); //�ۑ��p16bit

	//imshow("aveDepth", aveDepthImage);
}

void FIFOFilter()
{
    int j = dPixels * ptr;
    for (int i = 0; i < dPixels; i++)
    {
		Sum[i] += (long)depthBuffer[i] - (long)nDepthBuffer[j + i]; // �ړ����Z�lSum[i]�̕ω������̂ݏC��
		nDepthBuffer[j + i] = depthBuffer[i]; // �V�K�f�[�^DataIn[i]���o�b�t�@�Ɋi�[
		//cout << "Sum[" << i << "]: " << Sum[i] << endl;

    }
    ptr++;
    if (ptr == nFrame) { ptr = 0; } //�y�o�b�t�@�|�C���^�X�V�z
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