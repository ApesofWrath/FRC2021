#include "Drive/SwerveDrive.h"
#include <cmath>

#include <frc/smartdashboard/SmartDashboard.h>
#include "Debug.h"


double shortest(double dst, double lst) {
    double err = std::fmod((dst - lst), apes::PI);
    if (std::fmod((dst - lst),apes::PI * 2) >apes::PI) {
        err -=apes::PI;
    }
    return lst + err;
}

SwerveModule_Falcon::SwerveModule_Falcon(SwerveModule module, PIDSettings drive, PIDSettings yaw) {
    driveMotor = new TalonFX(module.driveMotor);
    driveMotor->Config_kP(0, drive.kP);
    driveMotor->Config_kI(0, drive.kI);
    driveMotor->Config_kD(0, drive.kD);

    yawMotor = new TalonFX(module.yawMotor);
     yawMotor->Config_kP(0, yaw.kP);
    yawMotor->Config_kI(0, yaw.kI);
    yawMotor->Config_kD(0, yaw.kD);

}

void SwerveModule_Falcon::ConfigPIDYaw(double p, double i, double d) {
    yawMotor->Config_kP(0, p);
    yawMotor->Config_kI(0, i);
    yawMotor->Config_kD(0, d);
}

void SwerveModule_Falcon::ConfigPIDDrive(double p, double i, double d) {
    driveMotor->Config_kP(0, p);
    driveMotor->Config_kI(0, i);
    driveMotor->Config_kD(0, d);
}


void SwerveModule_Falcon::Reverse() {
    isReversed = true;

    yawMotor->SetInverted(true);
    driveMotor->SetInverted(true);
    
}

const double L = 73.66;
const double W = 73.66;

SwerveDrive::SwerveDrive(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft, SwerveModule backRight) : 
            m_FrontLeft(frontLeft, SWERVE_CONFIG_PID.kFrontLeftDrive, SWERVE_CONFIG_PID.kFrontLeftYaw),
            m_FrontRight(frontRight, SWERVE_CONFIG_PID.kFrontRightDrive, SWERVE_CONFIG_PID.kFrontRightYaw),
            m_BackLeft(backLeft, SWERVE_CONFIG_PID.kBackLeftDrive, SWERVE_CONFIG_PID.kBackLeftYaw),
            m_BackRight(backRight, SWERVE_CONFIG_PID.kBackRightDrive, SWERVE_CONFIG_PID.kBackRightYaw) {

    // m_AHRS = new AHRS(frc::SerialPort::kUSB);
    frc::SmartDashboard::PutNumber("Kp Yaw FL", SWERVE_CONFIG_PID.kFrontLeftYaw.kP);
    frc::SmartDashboard::PutNumber("Kp Yaw FR", SWERVE_CONFIG_PID.kFrontRightYaw.kP);
    frc::SmartDashboard::PutNumber("Kp Yaw BL", SWERVE_CONFIG_PID.kBackLeftYaw.kP);
    frc::SmartDashboard::PutNumber("Kp Yaw BR", SWERVE_CONFIG_PID.kBackRightYaw.kP);

    
}
double lastAngle = 0;
double lastAngle_fr = 0;
double lastAngle_fl = 0;
double lastAngle_br = 0;
double lastAngle_bl = 0;
void SwerveDrive::Update(frc::Joystick* joy, frc::Joystick* joyWheel) {
    // if (joy->GetTriggerPressed()) {
    //     m_Strafe = !m_Strafe;
    // }

    // // if (m_Strafe) {
    // frc::SmartDashboard::PutNumber("!w1 angle ", m_FrontLeft.yawMotor->GetSelectedSensorPosition());
    // frc::SmartDashboard::PutNumber("!w2 angle ", m_FrontRight.yawMotor->GetSelectedSensorPosition());
    // frc::SmartDashboard::PutNumber("!w3 angle ", m_BackLeft.yawMotor->GetSelectedSensorPosition());
    // frc::SmartDashboard::PutNumber("!w4 angle ", m_BackRight.yawMotor->GetSelectedSensorPosition());

    double kp_y_fl = 0.1;
    double kp_y_fr = 0.1;
    double kp_y_bl = 0.1;
    double kp_y_br = 0.1;

    m_FrontLeft.yawMotor->Config_kP(0, kp_y_fl);
    m_FrontRight.yawMotor->Config_kP(0, kp_y_fr);
    m_BackLeft.yawMotor->Config_kP(0, kp_y_bl);
    m_BackRight.yawMotor->Config_kP(0, kp_y_br);

    m_FrontLeft.driveMotor->Config_kP(0, kp_y_fl);
    m_FrontRight.driveMotor->Config_kP(0, kp_y_fr);
    m_BackLeft.driveMotor->Config_kP(0, kp_y_bl);
    m_BackRight.driveMotor->Config_kP(0, kp_y_br);

    // // SetAllTargetAngle(PI * joy->GetThrottle());

    // // //BasicPower(joy->GetY() * (float)joy->GetThrottle(), joy->GetX() * (float)joy->GetThrottle());
    // double x = joy->GetX();
    // frc::SmartDashboard::PutNumber("!x ", x);
    // if (abs(x) < 0.05) x = 0;
    // double y = -joy->GetY();
    // frc::SmartDashboard::PutNumber("y ", y);
    // if (abs(y) < 0.05) y = 0;

    // double angle = std::atan2(y, x) +apes::PI / 2.0;

    // // angle =apes::PI / 2.0;
    // double mag = std::abs(std::sqrt(x * x + y * y));
    // if (angle <apes::PI / 2.0 && lastAngle > 3 *apes::PI / 2.0) {
    //     angle += 2 *apes::PI;
    // }

    // if (lastAngle <apes::PI / 2.0 && angle > 3 *apes::PI / 2.0) {
    //     angle -= 2 *apes::PI;
    // }


    // lastAngle = angle;
    // frc::SmartDashboard::PutNumber("!angle ", angle);
    // frc::SmartDashboard::PutNumber("!mag ", mag);
    // SetAllTargetAngle(angle);
    // SetDriveTargetVelocity(mag);

    // m_FrontLeft.driveMotor->Set(ControlMode::PercentOutput, 0.2);
    // m_FrontLeft.yawMotor->Set(ControlMode::PercentOutput, 0.2);

    // m_FrontRight.driveMotor->Set(ControlMode::PercentOutput, 0.2);
    // m_FrontRight.yawMotor->Set(ControlMode::PercentOutput, 0.2);

    // m_BackLeft.driveMotor->Set(ControlMode::PercentOutput, 0.2);
    // m_BackLeft.yawMotor->Set(ControlMode::PercentOutput, 0.2);

    // m_BackRight.driveMotor->Set(ControlMode::PercentOutput, 0.2);
    // m_BackRight.yawMotor->Set(ControlMode::PercentOutput, 0.2);



    // frc::SmartDashboard::PutNumber("Target Strafe Angle", angle);
    // frc::SmartDashboard::PutNumber("Target Speed", mag);
    // frc::SmartDashboard::PutNumber("Joystick X", x);
    // frc::SmartDashboard::PutNumber("Joystick Y", y);
    // } else {

    double vx = joy->GetX() * 100;
    if (abs(vx) < 0.05) vx = 0;
    double vy = -joy->GetY() * 100;
    if (abs(vy) < 0.05) vy = 0;

    double omega = joyWheel->GetX() * apes::PI;

    double A = vx - omega * (L / 2.0);
    double B = vx + omega * (L / 2.0);
    double C = vy - omega * (W / 2.0);
    double D = vy + omega * (W / 2.0);

    { // Wheel 1 (FR)
        double speed = std::sqrt(B * B + C * C);
        double angle = shortest(std::atan2(B, C), lastAngle_fr);
        lastAngle_fr = angle;
        frc::SmartDashboard::PutNumber("!w1 speed ", speed);
        frc::SmartDashboard::PutNumber("!w1 angle ", angle);

        m_FrontRight.driveMotor->Set(ControlMode::Velocity, speed * 100);
        m_FrontRight.yawMotor->Set(ControlMode::Position, angle * ANGLE_TO_TICKS_SWERVE_YAW);
    }

    { // Wheel 2 (FL)
        double speed = std::sqrt(B * B + D * D);
        double angle = shortest(std::atan2(B, D), lastAngle_fl);
        lastAngle_fl = angle;
        frc::SmartDashboard::PutNumber("!w2 speed ", speed);
        frc::SmartDashboard::PutNumber("!w2 angle ", angle);
        
        m_FrontLeft.driveMotor->Set(ControlMode::Velocity, speed * 100);
        m_FrontLeft.yawMotor->Set(ControlMode::Position, angle * ANGLE_TO_TICKS_SWERVE_YAW);
    }

    { // Wheel 3 (BL)
        double speed = std::sqrt(A * A + D * D);
        double angle = shortest(std::atan2(A, D), lastAngle_bl);
        lastAngle_bl = angle;

        frc::SmartDashboard::PutNumber("!w3 speed ", speed);
        frc::SmartDashboard::PutNumber("!w3 angle ", angle);

        m_BackLeft.driveMotor->Set(ControlMode::Velocity, speed * 100);
        m_BackLeft.yawMotor->Set(ControlMode::Position, angle * ANGLE_TO_TICKS_SWERVE_YAW);
    }

    { // Wheel 4 (BR)
        double speed = std::sqrt(A * A + C * C);
        double angle = shortest(std::atan2(A, C), lastAngle_br);
        lastAngle_br = angle;

        frc::SmartDashboard::PutNumber("!w4 speed ", speed);
        frc::SmartDashboard::PutNumber("!w4 angle ", angle);
        
        m_BackRight.driveMotor->Set(ControlMode::Velocity, speed * 100);
        m_BackRight.yawMotor->Set(ControlMode::Position, angle * ANGLE_TO_TICKS_SWERVE_YAW);
    }

    // }

}



void SwerveDrive::SetMovement(double speed, double movementDirection, double facing) {
    double vx = std::cos(movementDirection);

    if (abs(vx) < 0.05) vx = 0;
    double vy = std::sin(movementDirection);
    if (abs(vy) < 0.05) vy = 0;

    double omega = facing;

    double A = vx - omega * (L / 2.0);
    double B = vx + omega * (L / 2.0);
    double C = vy - omega * (W / 2.0);
    double D = vy + omega * (W / 2.0);

    { // Wheel 1 (FR)
        double speed = std::sqrt(B * B + C * C);
        double angle = std::atan2(B, C);


        m_FrontRight.driveMotor->Set(ControlMode::Velocity, speed);
        m_FrontRight.yawMotor->Set(ControlMode::Position, (angle / (2 * apes::PI)) * 2048);
    }

    { // Wheel 2 (FL)
        double speed = std::sqrt(B * B + D * D);
        double angle = std::atan2(B, D);


        m_FrontLeft.driveMotor->Set(ControlMode::Velocity, speed);
        m_FrontLeft.yawMotor->Set(ControlMode::Position, (angle / (2 * apes::PI)) * 2048);
    }

    { // Wheel 3 (BL)
        double speed = std::sqrt(A * A + D * D);
        double angle = std::atan2(A, D);


        m_BackLeft.driveMotor->Set(ControlMode::Velocity, speed);
        m_BackLeft.yawMotor->Set(ControlMode::Position, (angle / (2 * apes::PI)) * 2048);
    }

    { // Wheel 4 (BR)
        double speed = std::sqrt(A * A + C * C);
        double angle = std::atan2(A, C);


        m_BackRight.driveMotor->Set(ControlMode::Velocity, speed);
        m_BackRight.yawMotor->Set(ControlMode::Position, (angle / (2 * apes::PI)) * 2048);
    }
}


void SwerveDrive::StopAll() {
    StopDrive();
    StopYaw();
}

void SwerveDrive::StopYaw() {
    m_FrontLeft.yawMotor->Set(ControlMode::PercentOutput, 0);
    m_FrontRight.yawMotor->Set(ControlMode::PercentOutput, 0);
    m_BackLeft.yawMotor->Set(ControlMode::PercentOutput, 0);
    m_BackRight.yawMotor->Set(ControlMode::PercentOutput, 0);
}

void SwerveDrive::StopDrive() {
    m_FrontLeft.driveMotor->Set(ControlMode::PercentOutput, 0);
    m_FrontRight.driveMotor->Set(ControlMode::PercentOutput, 0);
    m_BackLeft.driveMotor->Set(ControlMode::PercentOutput, 0);
    m_BackRight.driveMotor->Set(ControlMode::PercentOutput, 0);    
}

void SwerveDrive::BasicPower(float drivePower, float yawPower) {
    m_FrontLeft.driveMotor->Set(ControlMode::PercentOutput, drivePower);
    m_FrontRight.driveMotor->Set(ControlMode::PercentOutput, drivePower);
    m_BackLeft.driveMotor->Set(ControlMode::PercentOutput, drivePower);
    m_BackRight.driveMotor->Set(ControlMode::PercentOutput, drivePower);

    m_FrontLeft.yawMotor->Set(ControlMode::PercentOutput, yawPower);
    m_FrontRight.yawMotor->Set(ControlMode::PercentOutput, yawPower);
    m_BackLeft.yawMotor->Set(ControlMode::PercentOutput, yawPower);
    m_BackRight.yawMotor->Set(ControlMode::PercentOutput, yawPower);
}

void SwerveDrive::ZeroEncoders() {
    m_FrontLeft.driveMotor->SetSelectedSensorPosition(0);
    m_FrontRight.driveMotor->SetSelectedSensorPosition(0);
    m_BackLeft.driveMotor->SetSelectedSensorPosition(0);
    m_BackRight.driveMotor->SetSelectedSensorPosition(0);

    m_FrontLeft.yawMotor->SetSelectedSensorPosition(0);
    m_FrontRight.yawMotor->SetSelectedSensorPosition(0);
    m_BackLeft.yawMotor->SetSelectedSensorPosition(0);
    m_BackRight.yawMotor->SetSelectedSensorPosition(0);
}

void SwerveDrive::SetAllTargetAngle(double angle) {
    double targetPosition = angle * ANGLE_TO_TICKS_SWERVE_YAW;
    m_CurrentTargetYawPosition = targetPosition;

    double curPos_fl = m_FrontLeft.yawMotor->GetSelectedSensorPosition();
    double curPos_fr = m_FrontRight.yawMotor->GetSelectedSensorPosition();
    double curPos_bl = m_BackLeft.yawMotor->GetSelectedSensorPosition();
    double curPos_br = m_BackRight.yawMotor->GetSelectedSensorPosition();

    // frc::SmartDashboard::PutNumber("Target Yaw Position", targetPosition);
    // frc::SmartDashboard::PutNumber("Current Position - Front Left", curPos_fl);
    // frc::SmartDashboard::PutNumber("Current Position - Front Right", curPos_fr);
    // frc::SmartDashboard::PutNumber("Current Position - Back Left", curPos_bl);
    // frc::SmartDashboard::PutNumber("Current Position - Back Right", curPos_br);

    m_FrontLeft.yawMotor->Set(ControlMode::Position, targetPosition);
    m_FrontRight.yawMotor->Set(ControlMode::Position, targetPosition);
    m_BackLeft.yawMotor->Set(ControlMode::Position, targetPosition);
    m_BackRight.yawMotor->Set(ControlMode::Position, targetPosition);
}

void SwerveDrive::SetDriveTargetVelocity(double speed) {
    double targetPosition = speed * TICKS_PER_ROTATION_FALCON * 2;;

    m_FrontLeft.driveMotor->Set(ControlMode::Velocity, targetPosition);
    m_FrontRight.driveMotor->Set(ControlMode::Velocity, targetPosition);
    m_BackLeft.driveMotor->Set(ControlMode::Velocity, targetPosition);
    m_BackRight.driveMotor->Set(ControlMode::Velocity, targetPosition);

}

void SwerveDrive::SetDriveTargetDistance(double distance) { //distance is in meters
    double targetPosition = distance * (TICKS_PER_ROTATION_FALCON) * (SWERVE_MOD_DRIVE_RATIO) / (METER_TO_INCHES / WHEEL_CIRCUMFERENCE) ;
    
    m_FrontLeft.driveMotor->Set(ControlMode::Position, targetPosition + m_FrontLeft.driveMotor->GetSelectedSensorPosition());
    m_FrontRight.driveMotor->Set(ControlMode::Position, targetPosition + m_FrontRight.driveMotor->GetSelectedSensorPosition());
    m_BackLeft.driveMotor->Set(ControlMode::Position, targetPosition + m_BackLeft.driveMotor->GetSelectedSensorPosition());
    m_BackRight.driveMotor->Set(ControlMode::Position, targetPosition + m_BackRight.driveMotor->GetSelectedSensorPosition());

}

// 2048 * 7.39 = 4pi inches
// 4pi inches = 0.319185813494 meters (divide it by 39.3701)
// 7.39:1 drive 1 rot of wheel
// 4pi inches for one rot of wheel 

std::array<double, 4> SwerveDrive::GetAllDrivePositions(){

    // std::vector<double> 

    return {m_FrontLeft.driveMotor->GetSelectedSensorPosition(), 
        m_FrontRight.driveMotor->GetSelectedSensorPosition(), 
        m_BackLeft.driveMotor->GetSelectedSensorPosition(), 
        m_BackRight.driveMotor->GetSelectedSensorPosition()};
}

void SwerveDrive::SetSwerveStates(std::array<frc::SwerveModuleState, 4> desired_states) {
    m_FrontLeft.driveMotor->Set(ControlMode::Velocity, METERS_PER_WHEEL_ROTATION * TICKS_PER_ROTATION_FALCON * (double) (desired_states[0].speed));
    m_FrontRight.driveMotor->Set(ControlMode::Velocity, METERS_PER_WHEEL_ROTATION * TICKS_PER_ROTATION_FALCON * (double) (desired_states[1].speed));
    m_BackLeft.driveMotor->Set(ControlMode::Velocity, METERS_PER_WHEEL_ROTATION * TICKS_PER_ROTATION_FALCON * (double) (desired_states[2].speed));
    m_BackRight.driveMotor->Set(ControlMode::Velocity, METERS_PER_WHEEL_ROTATION * TICKS_PER_ROTATION_FALCON * (double) (desired_states[3].speed));

    m_FrontLeft.yawMotor->Set(ControlMode::Position,  (double) (units::radian_t) (desired_states[0].angle.Degrees()));
    m_FrontRight.yawMotor->Set(ControlMode::Position, (double) (units::radian_t) (desired_states[1].angle.Degrees()));
    m_BackLeft.yawMotor->Set(ControlMode::Position, (double) (units::radian_t) (desired_states[2].angle.Degrees()));
    m_BackRight.yawMotor->Set(ControlMode::Position, (double) (units::radian_t) (desired_states[3].angle.Degrees()));
}