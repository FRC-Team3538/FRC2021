#pragma once

#include <iostream>
#include "frc/smartdashboard/Smartdashboard.h"
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
	const double cameraAngle = 16;
	const double dh = 44; //distance between camera lens and quarter-way up the goal

	std::shared_ptr<NetworkTable> table;
	std::vector<double> pnpPoints;
	double dy, dx, tv, pipe, pnpDist;
	bool pipeSwitchOS = false;
	frc::Timer pipeSwitch;

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

	RJVisionPipeline();
	void Init();
	void Periodic();
	RJVisionPipeline::visionData Run(int shotType);
	void UpdateSmartDash();
	void SetPipeline(double pipeline);
	double DistEstimation();
	double DeterminePipeline(int shotType);
	void Reset();
};
} // namespace vision
