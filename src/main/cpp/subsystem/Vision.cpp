#include "subsystem/Vision.hpp"

Vision::Vision()
{
	// NOOP
}

void Vision::Init()
{
	cam0.SetFPS(20);
	cam0.SetResolution(160, 120);

	cam1.SetFPS(20);
	cam1.SetResolution(160, 120);

	outputStreamStd = CameraServer::GetInstance()->PutVideo("Vision", 160, 120);

	chooseCam.SetDefaultOption(none, none);
	chooseCam.AddOption(camera0, camera0);
	chooseCam.AddOption(camera1, camera1);
	frc::SmartDashboard::PutData("Selected Camera", &chooseCam);
	HumanVisionToggle();
}

void Vision::HumanVisionToggle()
{
	currentCam++;
	currentCam %= camCount;
	switch (currentCam)
	{
	case 0:
	{
		frc::SmartDashboard::PutString("CameraSelection", cam0.GetName());
		break;
	}
	case 1:
	{
		frc::SmartDashboard::PutString("CameraSelection", cam1.GetName());
		break;
	}
	}
}

Vision::returnData Vision::Run()
{
	auto cam = cam0;
	if (chooseCam.GetSelected() == camera1)
	{
		cam = cam1;
	}

	time.Start();
	time.Reset();

	cam.SetFPS(20);

	if (CVT)
	{
		cam.SetExposureManual(15); //20 worked well
		cam.SetBrightness(-300);
		cam.SetWhiteBalanceManual(4500);
	}
	else
	{
		cam.SetExposureAuto();
		cam.SetWhiteBalanceAuto();
		cam.SetBrightness(40);
	}

	if (CVT)
	{
		cvSink = cam == cam0 ? CameraServer::GetInstance()->GetVideo("Camera 0") : CameraServer::GetInstance()->GetVideo("Camera 1");
		cv::Mat source;
		cvSink.GrabFrame(source);
		if (source.empty())
		{
			return data;
		}
		VP.Process(source);

		cout << "A: " << time.Get() << endl;
		contourDataVector.clear();
		contourNum = 0;
		auto contours = *VP.GetFilterContoursOutput();
		auto contourSize = contours.size();
		cout << contourSize << endl;
		auto centers = contourCenters(*VP.GetFilterContoursOutput());
		vector<vector<Point>> hulls(contours.size());
		for (auto contour : contours)
		{
			Vec4f line;
			contourData cD;
			fitLine(contour, line, CV_DIST_L2, 0, 0.01, 0.01);
			double lineAngle = vectorAngle(line);
			cD.x = centers[contourNum].x;
			cD.angle = lineAngle;
			cD.numero = contourNum;
			convexHull(Mat(contours[contourNum]), hulls[contourNum], false);
			cout << "ANGLE" << lineAngle << endl;
			if (!(lineAngle < 50 || lineAngle > 130))
			{
				contourDataVector.push_back(cD);
				DrawLine(line, source);
				putText(source, to_string(contourNum), cvPoint((centers[contourNum].x + 10), (centers[contourNum].y + 5)), FONT_HERSHEY_COMPLEX_SMALL, 0.6, cvScalar(255, 255, 255), 1, CV_AA);
			}
			contourNum++;
		}
		if (contourNum == 0 || contourNum == 1)
		{
			data.cmd = -3.14;
			return data;
		}
		// for(auto hull : hulls){
		// 	cout << "------->HULL SIZE<------- :" << hull.size() << endl;
		// }
		int centerContour = -1;
		int centerContourPair = -1;
		if (contourDataVector.size() > 0)
		{
			for (int i = 0; i < contourDataVector.size(); i++)
			{
				if (i == 0)
				{
					centerContour = i;
				}
				else if (abs(contourDataVector[i].x - 80) < abs(contourDataVector[centerContour].x - 80))
				{
					centerContour = i;
				}
			}

			bool centerContourLeft = (contourDataVector[centerContour].angle > 90.0);
			for (int i = 0; i < contourDataVector.size(); i++)
			{
				if (centerContourLeft)
				{
					if ((centerContourPair == -1) && (i != centerContour) && (contourDataVector[i].x > contourDataVector[centerContour].x) && (contourDataVector[i].angle < 90))
					{
						centerContourPair = i;
					}
					else if ((contourDataVector[i].x > contourDataVector[centerContour].x) && (i != centerContour) && (contourDataVector[i].angle < 90) && (contourDataVector[i].x < contourDataVector[centerContourPair].x))
					{
						centerContourPair = i;
					}
				}
				else if (!centerContourLeft)
				{
					if ((centerContourPair == -1) && (i != centerContour) && (contourDataVector[i].x < contourDataVector[centerContour].x) && (contourDataVector[i].angle > 90))
					{
						centerContourPair = i;
					}
					else if ((contourDataVector[i].x < contourDataVector[centerContour].x) && (i != centerContour) && (contourDataVector[i].angle > 90) && (contourDataVector[i].x > contourDataVector[centerContourPair].x))
					{
						centerContourPair = i;
					}
				}
			}
		}
		//fitLine(singleContour(*VP.GetFilterContoursOutput(), 1), line2, CV_DIST_L2, 0, 0.01, 0.01);
		// DrawLine(line, *VP.GetHslThresholdOutput());
		// DrawLine(line2, *VP.GetHslThresholdOutput());
		//drawContours( source, *VP.GetFilterContoursOutput(), -1, Scalar(148, 148, 0), 2);
		cout << "SIZE" << contourDataVector.size() << endl;
		if (centerContour > -1)
		{
			drawContours(source, hulls, contourDataVector[centerContour].numero, Scalar(148, 148, 0), 2);
		}
		if (centerContourPair > -1)
		{
			drawContours(source, hulls, contourDataVector[centerContourPair].numero, Scalar(148, 148, 0), 2);
		}
		cv::Point position;
		double distance;

		if (centerContour > -1 && centerContourPair > -1)
		{
			distance = (contourDataVector[centerContourPair].x - contourDataVector[centerContour].x);
			position.x = (contourDataVector[centerContourPair].x + contourDataVector[centerContour].x) / 2;
			position.y = (centers[contourDataVector[centerContourPair].numero].y + centers[contourDataVector[centerContourPair].numero].y) / 2;
			drawMarker(source, position, Scalar(255, 255, 255), 1, 10, 2);
		}

		outputStreamStd.PutFrame(source);
		imwrite(path, *VP.GetHslThresholdOutput());

		cv::Mat frame0;
		cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
		cvSink.GrabFrame(frame0);

		int imNum2 = imNum + 1;
		imwrite("/u/vision" + std::to_string(imNum2) + ".jpg", frame0);
		imNum += 2;
		// double realCenter = 0;

		// for (auto c : centers)
		// {
		// 	realCenter += c.x;
		// }
		// realCenter /= centers.size();

		// if ((realCenter != 80) && (realCenter > 0))
		// {
		// 	double error = (realCenter - 80) / 160;

		// 	cout << "F: " << time.Get() << endl;
		// 	return error;
		// }
		if ((position.x > 0))
		{
			// const double kD = 0.0001;
			// const double kP = 0.007;
			const double kP = 0.005;
			const double kI = 0.00001;
			const double kD = 0.000; //0.0002
			double error = (position.x - 80.0);

			if (abs(error) < 20)
			{
				sumError += error /  0.02;
			}
			else
			{
				sumError = 0;
			}
			frc::SmartDashboard::PutNumber("Eye", sumError);
			frc::SmartDashboard::PutNumber("Arror", error);
			double deltaError = (error - prevError) / 0.02;
			prevError = error;
			double cmd = error * kP + sumError * kI + deltaError * kD;
			frc::SmartDashboard::PutNumber("Command", cmd);
			cout << "F: " << time.Get() << endl;
			data.distance = distance;
			data.cmd = cmd;
			data.data = true;
			return data;
		}
	}

	// if(!frame0.empty()){
	// 	//VP.Process(frame0);
	// 	cs::CvSource outputStreamStd = CameraServer::GetInstance()->PutVideo("HSLThresh", 160, 120);
	// 	outputStreamStd.PutFrame(frame0/**VP.GetHslThresholdOutput()*/);
	// }
	return data;
}

void Vision::CVMode(bool On)
{
	CVT = On;
}

cv::Point Vision::centerOfContour(std::vector<cv::Point> contour)
{
	double totalx = 0.0; //given a contour, outputs its center
	double totaly = 0.0;
	for (int d = 0; d < contour.size(); d++)
	{
		totalx += contour[d].x;
		totaly += contour[d].y;
	}
	cv::Point pt;
	pt.x = totalx / contour.size();
	pt.y = totaly / contour.size();
	return pt;
}

std::vector<cv::Point> Vision::contourCenters(std::vector<std::vector<cv::Point>> contours)
{
	std::vector<cv::Point> centers; //given a vector of contours, outputs a vector consisting of their centers
	double totalx;
	double totaly;
	for (int c = 0; c < contours.size(); c++)
	{
		centers.push_back(centerOfContour(contours[c]));
	}
	return centers;
}

// double Vision::contourAngle(std::vector<cv::Point> contour){
// 	double angle;
// 	double contourSize
// 	double deltaY = (contour[contour.size()-1].y - contour[0].y) / (contour[contour.size()-1].x - contour[0].x);
// 	double deltaX = (contour)
// 	angle = atan2(slope, ) * (180 / 3.141592653589);
// 	return angle;
// }

std::vector<cv::Point> Vision::singleContour(std::vector<std::vector<cv::Point>> contours, int numero)
{
	return contours[numero];
}

void Vision::DrawLine(Vec4f vector, cv::Mat img)
{
	double theMult = max(120, 160);
	// calculate start point
	CvPoint startPoint;
	startPoint.x = vector[2] - theMult * vector[0]; // x0
	startPoint.y = vector[3] - theMult * vector[1]; // y0
	// calculate end point
	CvPoint endPoint;
	endPoint.x = vector[2] + theMult * vector[0]; //x[1]
	endPoint.y = vector[3] + theMult * vector[1]; //y[1]

	// draw overlay of bottom vectors on image
	cv::line(img, startPoint, endPoint, Scalar(0, 0, 255), 2);
	//line(img, startPoint, endPoint, Scalar(0, 0, 255), 3, 8, 0);
}

double Vision::vectorAngle(Vec4f vector)
{
	double angle = atan2(vector[1], vector[0]);
	angle = angle * (180 / 3.141592653589793238);
	if (angle < 0)
	{
		angle += 180;
	}
	else if (angle >= 180 && angle < 360)
	{
		angle -= 180;
	}
	return angle;
}
void Vision::UpdateSmartdash()
{
	frc::SmartDashboard::PutData("Selected Camera", &chooseCam);
}