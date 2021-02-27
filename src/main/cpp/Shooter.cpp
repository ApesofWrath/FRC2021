#include <Shooter.h>

    

    Shooter::Shooter() {
        shooter_state = INIT_STATE;

        rightIndexerSpark = new rev::CANSparkMax(0, rev::CANSparkMax::MotorType::kBrushless);
        leftIndexerSpark = new rev::CANSparkMax(0, rev::CANSparkMax::MotorType::kBrushless);

        beltSpark = new rev::CANSparkMax(0, rev::CANSparkMax::MotorType::kBrushless);

        topWTalon = new WPI_TalonFX(topWTalonID);
        bottomWTalon = new WPI_TalonFX(bottomWTalonID);

        ConfigureTalon(topWTalon,       0, 0, 10, false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, NeutralMode::Brake);
        ConfigureTalon(bottomWTalon,    0, 0, 10, false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, NeutralMode::Brake);

        beltPID = new rev::CANPIDController(beltSpark->GetPIDController());
        beltEncoder = new rev::CANEncoder(beltSpark->GetEncoder());
        
        rightIndexerPID = new rev::CANPIDController(rightIndexerSpark->GetPIDController());
        rightIndexerEncoder = new rev::CANEncoder(rightIndexerSpark->GetEncoder());

        leftIndexerPID = new rev::CANPIDController(leftIndexerSpark->GetPIDController());
        leftIndexerEncoder = new rev::CANEncoder(leftIndexerSpark->GetEncoder());

        ConfigureSpark(beltSpark, beltPID,  beltkP, beltkI, beltkD, beltkIz, beltkFF, beltkMinOutput, beltkMaxOutput, rev::CANSparkMax::IdleMode::kBrake);
        ConfigureSpark(leftIndexerSpark, leftIndexerPID,    indexerkP, indexerkI, indexerkD, indexerkIz, indexerkFF, indexerkMaxOutput, indexerkMinOutput, rev::CANSparkMax::IdleMode::kBrake);
        ConfigureSpark(rightIndexerSpark, rightIndexerPID,  indexerkP, indexerkI, indexerkD, indexerkIz, indexerkFF, indexerkMinOutput, indexerkMaxOutput, rev::CANSparkMax::IdleMode::kBrake);
        leftIndexerSpark->Follow(*rightIndexerSpark, true);
    }

    void Shooter::ShooterStateMachine(){
            
            switch(shooter_state) {
                case INIT_STATE:
                frc::SmartDashboard::PutString("Shooter ", "init");
                Init();
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
                frc::SmartDashboard::PutString("Shooter", "reverse");
                Reverse();
                last_shooter_state = REVERSE_STATE;
                break;
            }

    }

    void Shooter::Init(){
        
    }

    void Shooter::Shoot(){
        
        rightIndexerPID->SetReference(indexerShootSpeed, rev::ControlType::kVelocity);
        topWTalon->Set(ControlMode::MotionMagic, topWTalon->GetSelectedSensorPosition(0) + 668); // need to config, when get testing equip finalize method of moving wheels
        bottomWTalon->Set(ControlMode::MotionMagic, bottomWTalon->GetSelectedSensorPosition(0) + 668); // need to config, when get testing equip finalize method of moving wheels
        beltPID->SetReference(beltSpeed, rev::ControlType::kVelocity); // look into smart velocity
   
    }

    void Shooter::Intake(){                
        rightIndexerPID->SetReference(indexerShootSpeed, rev::ControlType::kVelocity);
        topWTalon->Set(ControlMode::PercentOutput, 0.0);
        bottomWTalon->Set(ControlMode::PercentOutput, 0.0);
        beltSpark->Set(0);

    }

    void Shooter::Stop(){
 
        rightIndexerSpark->Set(0);
        topWTalon->Set(ControlMode::PercentOutput, 0.0);
        bottomWTalon->Set(ControlMode::PercentOutput, 0.0);
        beltSpark->Set(0);
    }

    void Shooter::Waiting(){ 
        rightIndexerSpark->Set(0.1);
        topWTalon->Set(ControlMode::PercentOutput, 0.0);
        topWTalon->SetSelectedSensorPosition(0, 0, 10); // zero after shooting
        bottomWTalon->Set(ControlMode::PercentOutput, 0.0);
        bottomWTalon->SetSelectedSensorPosition(0, 0, 10); // zero after shooting
        beltSpark->Set(0);
    }

    void Shooter::Reverse() {  
        rightIndexerSpark->Set(-1.0);
        topWTalon->Set(ControlMode::PercentOutput, -1.0);
        bottomWTalon->Set(ControlMode::PercentOutput, -1.0);
        beltSpark->Set(-1.0);
    }

    void Shooter::ConfigureSpark(rev::CANSparkMax *spark, rev::CANPIDController *PIDController, 
        double kP, double kI, double kD, double kIz, double kFF, double minOut, 
        double maxOut, rev::CANSparkMax::IdleMode idleMode)
    {
        PIDController->SetP(kP);
        PIDController->SetI(kI);
        PIDController->SetD(kD);
        PIDController->SetIZone(kIz);
        PIDController->SetFF(kFF);
        PIDController->SetOutputRange(minOut, maxOut);

        spark->SetIdleMode(idleMode);

        spark->BurnFlash();
    }

    void Shooter::ConfigureTalon(WPI_TalonFX *talon, int pidSlot, int pidType, int timeoutMs, bool inverted, 
        int nominalForward, int nominalReverse, int peakForward, int peakReverse, int profileSlot, 
        int kF, int kP, int kI, int kD, int cruiseVelocity, int acceleration, NeutralMode brakeMode)
    {
        talon->ConfigFactoryDefault();
        talon->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, pidType);\
        talon->SetInverted(inverted);
        talon->ConfigNominalOutputForward(nominalForward, timeoutMs);
        talon->ConfigNominalOutputReverse(nominalReverse, timeoutMs);
        talon->ConfigPeakOutputForward(peakForward, timeoutMs);
        talon->ConfigPeakOutputReverse(peakReverse, timeoutMs);
        talon->SelectProfileSlot(pidSlot, pidType);
        talon->Config_kF(pidSlot, kF, timeoutMs);
        talon->Config_kP(pidSlot, kP, timeoutMs);
        talon->Config_kI(pidSlot, kI, timeoutMs);
        talon->Config_kD(pidSlot, kD, timeoutMs);
        talon->ConfigMotionCruiseVelocity(cruiseVelocity);
        talon->ConfigMotionAcceleration(acceleration);
        talon->SetNeutralMode(brakeMode);

        talon->SetSelectedSensorPosition(0, pidType, timeoutMs);
    }