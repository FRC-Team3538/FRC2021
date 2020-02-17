#include "subsystem/RJVisionPipeline.hpp"

using namespace nt;

namespace vision
{

RJVisionPipeline::RJVisionPipeline()
{
}

void RJVisionPipeline::Init()
{
	table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
	SetPipeline(0.0);
	table->PutNumber("ledMode", 1.0);
}

void RJVisionPipeline::Periodic()
{
	dx = -(table->GetNumber("tx", 0.0));
	dy = table->GetNumber("ty", 0.0);
	tv = table->GetNumber("tv", 0.0);
	pipe = table->GetNumber("pipeline", 0.0);
	pnpPoints = table->GetNumberArray("camtran", std::vector<double>());
}

RJVisionPipeline::visionData RJVisionPipeline::Run(int shotType) //Returns telemetry and changes pipeline
{
	if (table->GetNumber("ledMode", 0.0) != 3.0)
	{
		table->PutNumber("ledMode", 3.0);
		lightOn.Reset();
		lightOn.Start();
	}
	RJVisionPipeline::visionData telemetry;

	SetPipeline(shotType);

	if (pnpPoints.size() >= 5)
	{
		pnpDist = sqrt(pow(pnpPoints[0], 2) + pow((pnpPoints[2] - 29), 2));
	}

	if (((pipeSwitch.Get() < 1.0) && pipeSwitchOS))
	{
		telemetry.angle = 420.0;
		telemetry.distance = -1.0;
		telemetry.filled = false;
	}
	else if ((pipeSwitch.Get() > 1.0 || !pipeSwitchOS) && (lightOn.Get() > 0.3))
	{
		if (tv == 1.0)
		{
			telemetry.angle = dx;
			telemetry.distance = (shotType == 0) ? DistEstimation() : pnpDist;
			telemetry.filled = true;
		}
		else
		{
			telemetry.angle = 420.0;
			telemetry.distance = -1.0;
			telemetry.filled = false;
		}
	}
	return telemetry;
}

void RJVisionPipeline::SetPipeline(double pipeline)
{
	if (pipeline != GetPipeline())
	{
		table->PutNumber("pipeline", pipeline);
		table->PutNumber("camMode", 0.0);
		pipeSwitch.Reset();
		pipeSwitch.Start();
		pipeSwitchOS = true;
	}
	else
	{
		pipeSwitchOS = false;
	}
}

double RJVisionPipeline::GetPipeline()
{
	return table->GetNumber("getpipe", 0.0);
}

void RJVisionPipeline::TakePicture(bool pic)
{
	pic ? table->PutNumber("snapshot", 1.0) : table->PutNumber("snapshot", 0.0);
}

void RJVisionPipeline::UpdateSmartDash()
{
	frc::SmartDashboard::PutNumber("dx", dx);
	frc::SmartDashboard::PutNumber("Vision Dist", DistEstimation());
	if (pnpPoints.size() >= 5)
	{
		frc::SmartDashboard::PutNumber("pnpDist", sqrt(pow(pnpPoints[0], 2) + pow((pnpPoints[2] - 29), 2)));
	}
}

void RJVisionPipeline::Reset()
{
	table->PutNumber("ledMode", 1.0);
	SetPipeline(0.0);
	pipeSwitchOS = false;
	pipeSwitchCt = 0;
}

double RJVisionPipeline::DistEstimation()
{
	double dist = dh / (tan((dy + cameraAngle) * (3.1415 / 180.0)));
	return dist;
}

double RJVisionPipeline::DeterminePipeline(int shotType)
{
	switch (shotType)
	{
	case 0:
	{
		if ((DistEstimation() < 400.0) && (DistEstimation() > 170.0))
		{
			return 3.0;
		}
		else
		{
			return 0.0;
		}

		break;
	}

	case 1:
	{
		if (DistEstimation() > 130.0)
		{
			return 2.0;
		}
		else
		{
			return 1.0;
		}
		break;
	}
	case 2:
	{
		return 4.0;
	}
	default:
	{
		std::cout << "AHHHHHHHHHHHHHHHHHHHHHHHHHHHH" << std::endl;
		return 0.0;
		//SOMETHING WENT VERY WRONG!!!
	}
	}
}
} // namespace vision
