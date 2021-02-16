#pragma once

#include <iostream>
#include "frc/smartdashboard/SmartDashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include <cmath>
#include <frc/Timer.h>

namespace vision
{

/*
* RJVisionPipeline class.
*/

class RJVisionPipeline
{
private:
	const double cameraAngle = 32;
	const double dh = 63.0; //distance between camera lens and quarter-way up the goal

	std::shared_ptr<NetworkTable> table;
	std::shared_ptr<NetworkTable> ballTable;
	std::vector<double> pnpPoints;
	double dy, dx, tv, pipe, pnpDist;
	bool pipeSwitchOS = false;
	int pipeSwitchCt = 0;
	frc::Timer pipeSwitch;
	frc::Timer lightOn;

	const double redConfig[2][3] = {{240.0, 50.0, 165.0}, {200.0, 150.0, 120.0}}; //(x, y)
	const double blueConfig[2][3] = {{1.0, 2.0, 3.0}, {1.0, 2.0, 3.0}}; //(x, y)

public:
	typedef struct
	{
		double distance, angle;
		bool filled = false;
	} visionData;

	enum ShotType
	{
		Two = 0, 
		Three
	};

	enum Pipe
	{
		TwoClose = 0,
		ThreeClose,
		ThreeFar,
		LongShot
	};

	enum SearchConfig
	{
		Red = 0,
		Blue,
		Err
	};

	RJVisionPipeline();
	void Init();
	void Periodic();
	RJVisionPipeline::visionData Run(int shotType);
	void UpdateSmartDash();
	void SetPipeline(double pipeline);
	double GetPipeline();
	void TakePicture(bool pic);
	double DistEstimation();
	double DeterminePipeline(int shotType);
	void Reset();
	SearchConfig GetConfig();
};
} // namespace vision
