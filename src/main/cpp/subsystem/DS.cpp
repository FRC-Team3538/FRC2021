#include <subsystem/DS.hpp>

DS::DS()
{

	chooseController.SetDefaultOption(sPS4, sPS4);
	chooseController.AddOption(sXBX, sXBX);

	chooseDriveLimit.AddOption(sLimit, sLimit);
	chooseDriveLimit.SetDefaultOption(sUnlimitted, sUnlimitted);
}
void DS::SmartDash() {
	frc::SmartDashboard::PutData("_SelectedController", &chooseController);
	frc::SmartDashboard::PutData("_DriveLimits", &chooseDriveLimit);
}