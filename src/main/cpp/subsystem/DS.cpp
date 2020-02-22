#include <subsystem/DS.hpp>

DS::DS()
{

	chooseController.SetDefaultOption(sPS4, sPS4);
	chooseController.AddOption(sXBX, sXBX);
	chooseController.AddOption(sPS4Driver, sXboxOperator);
	chooseController.AddOption(sXboxDriver, sPS4Operator);
	

}

void DS::SmartDash() {
	frc::SmartDashboard::PutData("_SelectedController", &chooseController);

	//  Controller Type
	if (chooseController.GetSelected() == sXBX)
	{
		Driver.SetControllerType(UniversalController::ControllerType::kXbox);
		Operator.SetControllerType(UniversalController::ControllerType::kXbox);
	} if (chooseController.GetSelected() == sPS4)
		{
		Driver.SetControllerType(UniversalController::ControllerType::kPS4);
		Operator.SetControllerType(UniversalController::ControllerType::kPS4);
	}
	if (chooseController.GetSelected() == sPS4Driver)
	{
		Driver.SetControllerType(UniversalController::ControllerType::kPS4);
		Operator.SetControllerType(UniversalController::ControllerType::kXbox);
	} if (chooseController.GetSelected() == sXboxDriver){
		Driver.SetControllerType(UniversalController::ControllerType::kXbox);
		Operator.SetControllerType(UniversalController::ControllerType::kPS4);
	}
}