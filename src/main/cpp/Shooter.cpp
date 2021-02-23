#include <Shooter.h>

#define MOTOR_ROTATION_PER_HALF_BELT_ROTATION 26.1799387799

    double shootkP = 10.8, shootkI = 0, shootkD = 0.0, shootkIz = 0, shootkFF = 0, shootkMaxOutput = 0.3, shootkMinOutput = -0.3; // Not tuned yet
    double beltkP = 10.8, beltkI = 0, beltkD = 0.0, beltkIz = 0, beltkFF = 0, beltkMaxOutput = 0.3, beltkMinOutput = -0.3;
    
    

    Shooter::Shooter() {
        shooter_state = INIT_STATE;

        indexerTalon = new TalonSRX(indexerTalonID);

        topWTalon = new WPI_TalonFX(topWTalonID);
        bottomWTalon = new WPI_TalonFX(bottomWTalonID);

        indexerTalon->SetNeutralMode(NeutralMode::Coast);

        topWTalon->SetNeutralMode(NeutralMode::Brake);
        bottomWTalon->SetNeutralMode(NeutralMode::Brake);

        frontBeltNEO = new rev::CANSparkMax(0, rev::CANSparkMax::MotorType::kBrushless);
        backBeltNEO = new rev::CANSparkMax(0, rev::CANSparkMax::MotorType::kBrushless);

        ConfigureTalon(topWTalon, 0, 0, 10, false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        ConfigureTalon(bottomWTalon, 0, 0, 10, false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

        frontBeltPID = new rev::CANPIDController(frontBeltNEO->GetPIDController());
        backBeltPID = new rev::CANPIDController(backBeltNEO->GetPIDController());

        frontBeltEncoder = new rev::CANEncoder(frontBeltNEO->GetEncoder());
        backBeltEncoder = new rev::CANEncoder(backBeltNEO->GetEncoder());

        ConfigureSpark(frontBeltNEO, frontBeltPID, beltkP, beltkI, beltkD, beltkIz, beltkFF, beltkMinOutput, beltkMaxOutput, rev::CANSparkMax::IdleMode::kBrake);
        ConfigureSpark(backBeltNEO, backBeltPID, beltkP, beltkI, beltkD, beltkIz, beltkFF, beltkMinOutput, beltkMaxOutput, rev::CANSparkMax::IdleMode::kBrake);

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
            }

    }

    void Shooter::Init(){
        
    }

    void Shooter::Shoot(){
        // beltNEO->Set(1);
        // topWNEO->Set(0.5);
        // botWNEO->Set(1);
        // topWPID->SetReference(topShootSpeed, rev::ControlType::kVelocity); //5676 is max free spin rpm
        // botWPID->SetReference(bottomShootSpeed, rev::ControlType::kVelocity);

        indexerTalon->Set(ControlMode::PercentOutput, 1.0);
        topWTalon->Set(ControlMode::MotionMagic, topWTalon->GetSelectedSensorPosition(0) + 668); // need to config, when get testing equip finalize method of moving wheels
        bottomWTalon->Set(ControlMode::MotionMagic, bottomWTalon->GetSelectedSensorPosition(0) + 668); // need to config, when get testing equip finalize method of moving wheels
        frontBeltPID->SetReference(frontBeltSpeed, rev::ControlType::kVelocity); // look into smart velocity
        backBeltPID->SetReference(backBeltSpeed, rev::ControlType::kVelocity);  //5676 is max free spin rpm

    }

    void Shooter::Intake(){                
        // beltPOS = beltEncoder->GetPosition();
        // beltPID->SetReference(beltPOS + 10, rev::ControlType::kPosition);
        // topWNEO->Set(0);
        // botWNEO->Set(0);
        indexerTalon->Set(ControlMode::PercentOutput, 1.0);
        topWTalon->Set(ControlMode::PercentOutput, 0.0);
        bottomWTalon->Set(ControlMode::PercentOutput, 0.0);
        frontBeltNEO->Set(0);
        backBeltNEO->Set(0);

    }

    void Shooter::Stop(){
        // beltNEO->Set(0);
        // topWNEO->Set(0);
        // botWNEO->Set(0);
        indexerTalon->Set(ControlMode::PercentOutput, 0.0);
        topWTalon->Set(ControlMode::PercentOutput, 0.0);
        bottomWTalon->Set(ControlMode::PercentOutput, 0.0);
        frontBeltNEO->Set(0);
        backBeltNEO->Set(0);
    }

    void Shooter::Waiting(){ // slower than shooting
        // beltNEO->Set(0);
        // topWNEO->Set(0);
        // botWNEO->Set(0);

        indexerTalon->Set(ControlMode::PercentOutput, 0.1);
        topWTalon->Set(ControlMode::PercentOutput, 0.0);
        topWTalon->SetSelectedSensorPosition(0, 0, 10); // zero after shooting
        bottomWTalon->Set(ControlMode::PercentOutput, 0.0);
        bottomWTalon->SetSelectedSensorPosition(0, 0, 10); // zero after shooting
        frontBeltNEO->Set(0);
        backBeltNEO->Set(0);
    }

    void Shooter::Reverse() {  //not needed i hope
        // beltNEO->Set(-0.2);
        // topWNEO->Set(0);
        // botWNEO->Set(0);
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
        int kF, int kP, int kI, int kD, int cruiseVelocity, int acceleration)
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

        talon->SetSelectedSensorPosition(0, pidType, timeoutMs);
    }