#include <Shooter.h>

    

    Shooter::Shooter() {
        
        // talon1 = new TalonFX(11);
        // ConfigureTalon(talon1, 0, 0, 10, false, 0.10976, 0.221, 0, 0, ctre::phoenix::motorcontrol::NeutralMode::Coast);
        
        shooter_state = INIT_STATE;

        right_indexer_spark = new rev::CANSparkMax(9, rev::CANSparkMax::MotorType::kBrushless);
        left_indexer_spark = new rev::CANSparkMax(12, rev::CANSparkMax::MotorType::kBrushless);

        belt_spark = new rev::CANSparkMax(27, rev::CANSparkMax::MotorType::kBrushless);

        top_wheel_spark = new rev::CANSparkMax(28, rev::CANSparkMax::MotorType::kBrushless);
        bottom_wheel_spark = new rev::CANSparkMax(29, rev::CANSparkMax::MotorType::kBrushless);


        // ConfigureTalon(top_wheel_spark,       0, 0, 10, false, 0, 0, 0, 0, NeutralMode::Brake);
        // ConfigureTalon(bottom_wheel_spark,    0, 0, 10, false, 0, 0, 0, 0, NeutralMode::Brake);


        belt_PID = new rev::CANPIDController(belt_spark->GetPIDController());
        belt_encoder = new rev::CANEncoder(belt_spark->GetEncoder());
        
        right_indexer_PID = new rev::CANPIDController(right_indexer_spark->GetPIDController());
        right_indexer_encoder = new rev::CANEncoder(right_indexer_spark->GetEncoder());

        left_indexer_PID = new rev::CANPIDController(left_indexer_spark->GetPIDController());
        left_indexer_encoder = new rev::CANEncoder(left_indexer_spark->GetEncoder());

        top_wheel_PID = new rev::CANPIDController(top_wheel_spark->GetPIDController());
        top_wheel_encoder = new rev::CANEncoder(top_wheel_spark->GetEncoder());
       
        bottom_wheel_PID = new rev::CANPIDController(bottom_wheel_spark->GetPIDController());
        bottom_wheel_encoder = new rev::CANEncoder(bottom_wheel_spark->GetEncoder());



        ConfigureSpark(top_wheel_spark, top_wheel_PID,
            k_P_shooter, k_I_shooter, k_D_shooter, k_Iz_shooter, k_FF_shooter, -1, 1, rev::CANSparkMax::IdleMode::kCoast);
        ConfigureSpark(bottom_wheel_spark, bottom_wheel_PID,
            k_P_shooter, k_I_shooter, k_D_shooter, k_Iz_shooter, k_FF_shooter, -1, 1, rev::CANSparkMax::IdleMode::kCoast);

        ConfigureSpark(belt_spark, belt_PID,
            k_P_belt, k_I_belt, k_D_belt, k_Iz_belt, k_FF_belt, k_min_out_belt, k_max_out_belt, rev::CANSparkMax::IdleMode::kCoast);
        ConfigureSpark(left_indexer_spark, left_indexer_PID,
            k_P_indexer, k_I_indexer, k_D_indexer, k_Iz_indexer, k_FF_indexer, k_max_out_indexer, k_min_out_indexer, rev::CANSparkMax::IdleMode::kCoast);
        ConfigureSpark(right_indexer_spark, right_indexer_PID,
            k_P_indexer, k_I_indexer, k_D_indexer, k_Iz_indexer, k_FF_indexer, k_max_out_indexer, k_min_out_indexer, rev::CANSparkMax::IdleMode::kCoast);
        

        // the holy zeroes
        top_wheel_spark->Set(0);
        bottom_wheel_spark->Set(0);
        belt_spark->Set(0);

        belt_encoder->SetPosition(0);
        belt_PID->SetReference(0, rev::ControlType::kPosition);

        top_wheel_encoder->SetPosition(0);
        top_wheel_PID->SetReference(0, rev::ControlType::kPosition);

        bottom_wheel_encoder->SetPosition(0);
        bottom_wheel_PID->SetReference(0, rev::ControlType::kPosition);
        left_indexer_spark->Follow(*right_indexer_spark, true);
    }

    void Shooter::ShooterStateMachine(){
        frc::SmartDashboard::PutData("Desired Shooting Distance", &distance_chooser);     
        
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
                
                case FAR_SHOOT_STATE:
                frc::SmartDashboard::PutString("Shooter ", "far shoot");
                FarShoot();
                last_shooter_state = FAR_SHOOT_STATE;
                break;

                case MEDIUM_SHOOT_STATE:
                frc::SmartDashboard::PutString("Shooter ", "medium shoot");
                MediumShoot();
                last_shooter_state = MEDIUM_SHOOT_STATE;
                break;
                
                case CLOSE_SHOOT_STATE:
                frc::SmartDashboard::PutString("Shooter ", "close shoot");
                CloseShoot();
                last_shooter_state = CLOSE_SHOOT_STATE;
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

    void Shooter::Init() {
        distance_chooser.SetDefaultOption("Close", CLOSE);
        distance_chooser.AddOption("Close",   Distances::CLOSE);
        distance_chooser.AddOption("Medium",  Distances::MEDIUM);
        distance_chooser.AddOption("Far",     Distances::FAR);
    }

    void Shooter::FarShoot() {
        // top_wheel_PID->SetReference(-5750, rev::ControlType::kVelocity);
        // top_wheel_spark->Set(.1);

        // bottom_wheel_PID->SetReference(5750, rev::ControlType::kVelocity);
        // bottom_wheel_spark->Set(.8);

        // belt_PID->SetReference(-5650, rev::ControlType::kVelocity); 
        // belt_spark->Set(-.96);
        // talon1->Set(ControlMode::Velocity, 682);
        // talon1->Set(ControlMode::PercentOutput, .1);

        // right_indexer_PID->SetReference(indexer_shoot_speed, rev::ControlType::kVelocity);
    }

    void Shooter::MediumShoot() {
        // talon1->Set(ControlMode::Velocity, 0);
        // talon1->Set(ControlMode::PercentOutput, 0);

        // top_wheel_spark->Set(ControlMode::Velocity, top_shoot_speed * medium_modifier); // need to config, when get testing equip 
        // bottom_wheel_spark->Set(ControlMode::Velocity, bottom_shoot_speed * medium_modifier); 
        // belt_PID->SetReference(belt_speed, rev::ControlType::kVelocity); 
        // right_indexer_PID->SetReference(indexer_shoot_speed, rev::ControlType::kVelocity);
    
        //  top_wheel_PID->SetReference(-5750, rev::ControlType::kVelocity);
        // top_wheel_spark->Set(.1);

        // bottom_wheel_PID->SetReference(5750, rev::ControlType::kVelocity);
        // bottom_wheel_spark->Set(.1);

        // belt_PID->SetReference(-5650, rev::ControlType::kVelocity); 
        // belt_spark->Set(-.96);
    }

    void Shooter::CloseShoot() {
        right_indexer_PID->SetReference(indexer_shoot_speed, rev::ControlType::kVelocity);
        // top_wheel_spark->Set(ControlMode::Velocity, top_shoot_speed * close_modifier);
        // bottom_wheel_spark->Set(ControlMode::Velocity, bottom_shoot_speed * close_modifier);
        belt_PID->SetReference(belt_speed, rev::ControlType::kVelocity); 
   
    }

    void Shooter::Intake() {                
        right_indexer_PID->SetReference(indexer_shoot_speed, rev::ControlType::kVelocity);
        // top_wheel_spark->Set(ControlMode::PercentOutput, 0.0);
        // bottom_wheel_spark->Set(ControlMode::PercentOutput, 0.0);
        belt_spark->Set(0);

    }

    void Shooter::Stop() {
        right_indexer_spark->Set(0.2);
        // top_wheel_spark->Set(ControlMode::PercentOutput, 0.0);
        // bottom_wheel_spark->Set(ControlMode::PercentOutput, 0.0);
        // top_wheel_spark->Set(0);
        // bottom_wheel_spark->Set(0);
        // belt_spark->Set(0);
    }

    void Shooter::Waiting() { 
        right_indexer_spark->Set(0.1);
        top_wheel_spark->Set(0.0);
        bottom_wheel_spark->Set(0.0);
        belt_spark->Set(0);
    }

    void Shooter::Reverse() {  
        right_indexer_spark->Set(-1.0);
        // top_wheel_spark->Set(ControlMode::PercentOutput, -1.0);
        // bottom_wheel_spark->Set(ControlMode::PercentOutput, -1.0);
        belt_spark->Set(-1.0);
    }

    void Shooter::ConfigureSpark(rev::CANSparkMax *spark, rev::CANPIDController *PIDController, 
        double kP, double kI, double kD, double kIz, double kFF, double minOut, 
        double maxOut, rev::CANSparkMax::IdleMode idleMode) {
        
        PIDController->SetP(kP);
        PIDController->SetI(kI);
        PIDController->SetD(kD);
        PIDController->SetIZone(kIz);
        PIDController->SetFF(kFF);
        // PIDController->SetOutputRange(minOut, maxOut);

        spark->SetIdleMode(idleMode);

        spark->BurnFlash();
    }

    void Shooter::ConfigureTalon(TalonFX *talon, int pidSlot, int pidType, int timeoutMs, bool inverted,  
        int kF, int kP, int kI, int kD, NeutralMode brakeMode) {
        talon->ConfigFactoryDefault();
        
        talon->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);

        talon->SetInverted(inverted);
        talon->Config_kF(pidSlot, kF, timeoutMs);
        talon->Config_kP(pidSlot, kP, timeoutMs);
        talon->Config_kI(pidSlot, kI, timeoutMs);
        talon->Config_kD(pidSlot, kD, timeoutMs);
        talon->SetNeutralMode(brakeMode);

        talon->ConfigNominalOutputForward(0);
        talon->ConfigNominalOutputReverse(0);
        talon->ConfigPeakOutputForward(1);
        talon->ConfigPeakOutputReverse(-1);

        talon->SetSelectedSensorPosition(0, pidType, timeoutMs);
    }