#pragma once

#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>

extern frc::ShuffleboardTab* s_DriveTab;

void initDebugging();

namespace drive_debug {
    extern frc::SuppliedValueWidget<double> *s_TargetYawPos;

    extern nt::NetworkTableEntry kP_fl, kP_fr, kP_bl, kP_br;
}
