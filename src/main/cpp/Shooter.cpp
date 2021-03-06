#include <Shooter.h>

#define MOTOR_ROTATION_PER_HALF_BELT_ROTATION 26.1799387799

    const int INIT_STATE = 0;
    const int INTAKE_STATE = 1;
    const int STOP_STATE = 2;
    const int SHOOT_STATE = 3;
    const int WAITING_STATE = 4;
    const int REVERSE_STATE = 5;

    double shootkP = 10.8, shootkI = 0, shootkD = 0.0, shootkIz = 0, shootkFF = 0, shootkMaxOutput = 0.3, shootkMinOutput = -0.3;
    double beltkP = 10.8, beltkI = 0, beltkD = 0.0, beltkIz = 0, beltkFF = 0, beltkMaxOutput = 0.3, beltkMinOutput = -0.3;

    Shooter::Shooter() {
        shooter_state = INIT_STATE;

        // beltNEO = new TalonSRX(TALON_ID_0); //1 labeled on talon
        // topWNEO = new TalonSRX(TALON_ID_1); //2 labeled on talon
        // botWNEO = new TalonSRX(TALON_ID_2); //3 labeled on talon
// *beltNEO, topWNEO, *botWNEO;
        beltNEO = new rev::CANSparkMax(beltSpark, rev::CANSparkMax::MotorType::kBrushless);
        beltPID = new rev::CANPIDController(beltNEO->GetPIDController());
        beltEncoder = new rev::CANEncoder(beltNEO->GetEncoder());
        topWNEO = new rev::CANSparkMax(topRollerSpark, rev::CANSparkMax::MotorType::kBrushless);
        topWPID = new rev::CANPIDController(topWNEO->GetPIDController());
        topWEncoder = new rev::CANEncoder(topWNEO->GetEncoder());
        botWNEO = new rev::CANSparkMax(bottomRollerSpark, rev::CANSparkMax::MotorType::kBrushless);
        botWPID = new rev::CANPIDController(botWNEO->GetPIDController());
        botWEncoder = new rev::CANEncoder(topWNEO->GetEncoder());
        
        topWPID->SetP(shootkP);
        topWPID->SetI(shootkI);
        topWPID->SetD(shootkD);
        topWPID->SetIZone(shootkIz);
        topWPID->SetFF(shootkFF);
        topWPID->SetOutputRange(shootkMinOutput, shootkMaxOutput);

        botWPID->SetP(shootkP);
        botWPID->SetI(shootkI);
        botWPID->SetD(shootkD);
        botWPID->SetIZone(shootkIz);
        botWPID->SetFF(shootkFF);
        botWPID->SetOutputRange(shootkMinOutput, shootkMaxOutput);

        beltPID->SetP(beltkP);
        beltPID->SetI(beltkI);
        beltPID->SetD(beltkD);
        beltPID->SetIZone(beltkIz);
        beltPID->SetFF(beltkFF);
        beltPID->SetOutputRange(beltkMinOutput, beltkMaxOutput);

        beltNEO->BurnFlash();
        topWNEO->BurnFlash();
        botWNEO->BurnFlash();

    }

    void Shooter::ShooterStateMachine(){
            
            switch(shooter_state) {
                case INIT_STATE:
                frc::SmartDashboard::PutString("Shooter ", "init");
                shooter_state = STOP_STATE;
                break;
                case INTAKE_STATE:
                frc::SmartDashboard::PutString("Shooter ", "intake");
                Intake();
                last_shooter_state = INTAKE_STATE;
                break;
                
                case STOP_STATE:
                frc::SmartDashboard::PutString("Shooter ", "stop");
                Stop();
                last_shooter_state = STOP_STATE;
                break;
                
                case SHOOT_STATE:
                frc::SmartDashboard::PutString("Shooter ", "shoot");
                Shoot();
                last_shooter_state = SHOOT_STATE;
                break;

                case WAITING_STATE:
                frc::SmartDashboard::PutString("Shooter ", "waiting");
                Waiting();
                last_shooter_state = WAITING_STATE;
                break;
                case REVERSE_STATE:
                frc::SmartDashboard::PutString("Shooter ", "reverse");
                Reverse();
                last_shooter_state = REVERSE_STATE_H;
                break;
            }

    }

    void Shooter::Shoot(){
        beltNEO->Set(1);
        topWNEO->Set(0.5);
        botWNEO->Set(1);
        // topWPID->SetReference(topShootSpeed, rev::ControlType::kVelocity); //5676 is max free spin rpm
        // botWPID->SetReference(bottomShootSpeed, rev::ControlType::kVelocity);
    }

    void Shooter::Intake(){                
        beltPOS = beltEncoder->GetPosition();
        beltPID->SetReference(beltPOS + 10, rev::ControlType::kPosition);
        topWNEO->Set(0);
        botWNEO->Set(0);
    }

    void Shooter::Stop(){
        beltNEO->Set(0);
        topWNEO->Set(0);
        botWNEO->Set(0);
    }

    void Shooter::Waiting(){
        beltNEO->Set(0);
        topWNEO->Set(0);
        botWNEO->Set(0);
    }

    void Shooter::Reverse() {  
        beltNEO->Set(-0.2);
        topWNEO->Set(0);
        botWNEO->Set(0);
    }