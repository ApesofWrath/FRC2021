#include "Debug.h"
#include "Robot.h"
#include <iostream>
#include <networktables/NetworkTableValue.h>


frc::ShuffleboardTab *s_DriveTab;
frc::SuppliedValueWidget<double> *drive_debug::s_TargetYawPos;
nt::NetworkTableEntry drive_debug::kP_fl, drive_debug::kP_fr, drive_debug::kP_bl, drive_debug::kP_br;

void initDebugging() {
   s_DriveTab = &frc::Shuffleboard::GetTab("Drive");
//   drive_debug::s_TargetYawPos = &s_DriveTab->AddNumber("Current Yaw Target Position", []() -> double { return s_Nova->GetSwerveDrive()->GetCurrentTargetYawPosition(); });

   std::cout << "Starting to create config shuffleboard shit\n";


   try {

      wpi::StringMap<std::shared_ptr<nt::Value> > wprops;
      wprops.insert({"min", nt::Value::MakeDouble(0)});
      wprops.insert({"max", nt::Value::MakeDouble(1)});


      frc::SimpleWidget kp_fl_sw = s_DriveTab->AddPersistent("kP FL", 0.1f).WithWidget(frc::BuiltInWidgets::kNumberSlider).WithSize(2, 1).WithProperties(wprops);
      frc::SimpleWidget kp_fr_sw = s_DriveTab->AddPersistent("kP FR", 0.1f).WithWidget(frc::BuiltInWidgets::kNumberSlider).WithSize(2, 1).WithProperties(wprops);
      frc::SimpleWidget kp_bl_sw = s_DriveTab->AddPersistent("kP BL", 0.1f).WithWidget(frc::BuiltInWidgets::kNumberSlider).WithSize(2, 1).WithProperties(wprops);
      frc::SimpleWidget kp_br_sw = s_DriveTab->AddPersistent("kP BR", 0.1f).WithWidget(frc::BuiltInWidgets::kNumberSlider).WithSize(2, 1).WithProperties(wprops);
      drive_debug::kP_fl = kp_fl_sw.GetEntry();
      drive_debug::kP_fr = kp_fr_sw.GetEntry();
      drive_debug::kP_bl = kp_bl_sw.GetEntry();
      drive_debug::kP_br = kp_br_sw.GetEntry();
   } catch (std::exception& e) {
      std::cout << "Error: " << e.what() << "\n";
   }

   std::cout << "Added widgets, about to get network table entries\n";




   std::cout << "Finished creating config shuffleboard shit\n";
}