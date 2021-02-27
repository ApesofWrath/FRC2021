#pragma once

#include "rev/ColorSensorV3.h"
#include "rev/CANSparkMax.h"
#include <ctre/Phoenix.h>
#include "Macros.h"

#define COLOR_WHEEL_ROTATIONS 14.95996502
#define COLOR_SPEED 60.31857895
#define ARM_POSITION 0.1136957341

enum Colors {
  RED    = 0,
  BLUE   = 1,
  YELLOW = 2,
  GREEN  = 3,
  WHITE  = 4
};


Colors ColorFromFRCColor(frc::Color);

class ControlPanel {

public:
    // TalonSRX* talon;
    rev::CANSparkMax *controlPanelArm, *controlPanelRotator;
    rev::CANEncoder *controlPanelArmEncoder, *controlPanelRotatorEncoder;
    rev::CANPIDController *controlPanelArmPID, *controlPanelRotatorPID;
    std::string getColor(Colors c);

    ControlPanel();

    const int controlPanelArmSpark = 31; 
    const int controlPanelRotatorSpark = 32;

    // const int rotCount = 3;
    // const float rotRatio = 10; //Change to actual ratio @ comp

    const float rotationModeRot = (COLOR_WHEEL_ROTATIONS/(2*PI))*63;
    const int controlPanelGoSpeed = (COLOR_SPEED/(2*PI))*60;
    const int controlPanelStopSpeed = 0;
    const float armRaisePos = (ARM_POSITION/(2*PI))*63;
    float controlPanelArmStartPos;
    float controlPanelWheelStartPos;



    // const int CONTROL_PANEL_TALON = 1000;
    const float CONTROL_PANEL_SPEEN_ON = 1.0f;
    enum States {
        IDLE,
        POSITION_MODE,
        ROTATION_MODE, 
        HUMAN_LOAD
    };

    int rotationsCompleted = 0;
    Colors desiredColor = Colors::WHITE;

    void StateMachine();
    inline void PositionMode() { state = States::POSITION_MODE; };
    inline void RotationMode() { state = States::ROTATION_MODE; };
    inline void IdleMode() { state = States::IDLE; };
    
    inline bool HasReachedPosition(Colors detectedColor) { return detectedColor == desiredColor; };
    
    void Rotate();
    void Stop();



    inline void ResetRotations() { rotationsCompleted = 0; };
    inline void DesireColor(Colors color) { desiredColor = color; };


    static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
    rev::ColorSensorV3 m_colorSensor{i2cPort};
    frc::Color detectedColor;

    

    States state;
    States last_state;
    
};