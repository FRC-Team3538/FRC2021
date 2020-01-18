#include <subsystem/DS.hpp>

DS::DS()
{

	chooseController.SetDefaultOption(sPS4, sPS4);
	chooseController.AddOption(sXBX, sXBX);

}

void DS::SmartDash() {
	frc::SmartDashboard::PutData("_SelectedController", &chooseController);

	//  Controller Type
	if (chooseController.GetSelected() == sXBX)
	{
		Driver.SetControllerType(UniversalController::ControllerType::kXbox);
		Operator.SetControllerType(UniversalController::ControllerType::kXbox);
	} else {
		Driver.SetControllerType(UniversalController::ControllerType::kPS4);
		Operator.SetControllerType(UniversalController::ControllerType::kPS4);
	}
}