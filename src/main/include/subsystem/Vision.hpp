#pragma once

#include <RJVisionPipeline.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <cameraserver/CameraServer.h>
#include <iostream>
#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Timer.h>

using namespace frc;
using namespace cv;
using namespace grip;
using namespace std;

class Vision
{
private:
  RJVisionPipeline VP;
  double centerX = 0.0;
  int imNum = 1;

	SendableChooser<std::string> chooseCam;
	const std::string camera0 = "Vision";
	const std::string camera1 = "Wideangle";
  const std::string none = "None";

public:
  cs::UsbCamera cam0 = CameraServer::GetInstance()->StartAutomaticCapture("Camera 0", 0);
  cs::UsbCamera cam1 = CameraServer::GetInstance()->StartAutomaticCapture("Camera 1", 1);

  cs::CvSink sink0;
  cs::CvSink cvSink;

  cs::CvSource outputStreamStd;
  cs::CvSource cameraToggleStream;
  std::string path = "/u/vision" + std::to_string(imNum) + ".jpg";
  bool CVT = false;

  typedef struct
  {
    double cmd = -3.14, distance = -3.14;
    bool data = false;
  } returnData;
  returnData data;

  Timer time;
  int contourNum = 0;
  returnData Run();
  void Init();
  void CVMode(bool On);
  cv::Point centerOfContour(std::vector<cv::Point> contour);
  std::vector<cv::Point> contourCenters(std::vector<std::vector<cv::Point>> contours);
  std::vector<std::vector<cv::Point>> lines;
  std::vector<cv::Point> singleContour(std::vector<std::vector<cv::Point>> contours, int numero);
  void DrawLine(Vec4f elLine, cv::Mat img);
  double vectorAngle(Vec4f vector);
  //double contourAngle(std::vector<cv::Point> contour); //Degrees

  double prevError = 0;
  double sumError = 0;

  struct contourData
  {
    double x, angle;
    int numero;
  };

  std::vector<contourData> contourDataVector;

  Vision();

  void UpdateSmartdash();

  int currentCam = 0;
  const int camCount = 2;
  void HumanVisionToggle();
};