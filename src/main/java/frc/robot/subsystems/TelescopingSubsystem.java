package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.drivetrain.SwerveTeleop;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

// import com.ctre.phoenix6.motorcontrol.ControlFrame;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
// import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;




public class TelescopingSubsystem extends SubsystemBase {

    private TalonFX motor = new TalonFX(Constants.TelescopingSubsystem.MOTOR_ID);
    DigitalInput input = new DigitalInput(Constants.TelescopingSubsystem.DIO_PORT);
    // Counter counter = new Counter(input);
    boolean click;

    double encoderCount = motor.getPosition().getValue();
    double startingEncoderCount = encoderCount;
    double speed = 0.3;
    private static final double kP = 0.61; // 0.84
    private static final double kI = 0.00025;
    private static final double kD = 0;
    private static final double kF = 0.4;  // 0.4

    private static final double kVelocity = 40_000.0;       // 62_000.0
    private static final double kAcceleration = 30_000.0;   // 44_000.0

    private static final double MIN_POSITION = -1_500.0;
    private static final double MAX_POSITION = 230_000.0; // 200_000
    public static final double FIRST_RUNG_HEIGHT = 130_000.0; // 90_000
    public static final double ABOVE_RUNG_HEIGHT = 25_000.0; // 25_000
    public static final double FULL_EXTEND = MAX_POSITION;

    private double targetPosition = 0;
    
    double rotorPos;

    public TelescopingSubsystem () {
        // motor.configFactoryDefault();

        var motorConfig = new TalonFXConfiguration();

        motor.getConfigurator().apply(motorConfig);


        // motor.setControlFramePeriod(ControlFrame.Control_3_General, 100);
        // motor.setInverted(TalonFXInvertType.CounterClockwise);
        // resetMotors();
        //set motor to brake
        // motor.OverrideBrakeDurNeutral

        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        motor.getConfigurator().apply(motorConfig);

        DutyCycleOut duty = new DutyCycleOut(1, true, true);
        motor.setControl(duty);
    

        // motor.setInverted(TalonFXInvertType.CounterClockwise);
        // encoderCount = (double) motor.getRotorPosition();

        var rotorPosSignal = motor.getRotorPosition();
        rotorPos = rotorPosSignal.getValue();
        rotorPosSignal.waitForUpdate(0.020);
        encoderCount = rotorPos;


        var slot0Configs = new Slot0Configs();
        slot0Configs.kV = kF;
        slot0Configs.kP = kP;
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;

        motor.getConfigurator().apply(slot0Configs, 0.050);

        // motor.configMotionCruiseVelocity(kVelocity);
        // motor.configMotionAcceleration(kAcceleration);

        // var motionMagicConfigs = talonFXConfigs.MotionMagicConfigs;
        // motionMagicConfigs.MotionMagicCruiseVelocity = 195.3125; // 80 rps cruise velocity
        // motionMagicConfigs.MotionMagicAcceleration = 146.48; 

        MotionMagicConfigs mm = new MotionMagicConfigs();
        mm.MotionMagicCruiseVelocity = 195.3125; // 5 rotations per second cruise
        mm.MotionMagicAcceleration = 146.48; // Take approximately 0.5 seconds to reach max vel
        motorConfig.MotionMagic = mm;


        CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();
        m_currentLimits.SupplyCurrentLimit = 60; // Limit to 1 amps
        m_currentLimits.SupplyCurrentThreshold = 35;
        m_currentLimits.SupplyTimeThreshold = 0.1;
        m_currentLimits.SupplyCurrentLimitEnable = true;
    
        motorConfig.CurrentLimits = m_currentLimits;

        motor.getConfigurator().apply(motorConfig);


        //current limit burn out the motor at states
        // SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
        //     Constants.SwerveDrivetrain.DRIVE_ENABLE_CURRENT_LIMIT, 
        //     Constants.SwerveDrivetrain.DRIVE_CONTINUOUS_CL, 
        //     Constants.SwerveDrivetrain.DRIVE_PEAK_CL, 
        //     Constants.SwerveDrivetrain.DRIVE_PEAK_CURRENT_DURATION);

        // motor.configSupplyCurrentLimit(driveSupplyLimit);

        // click = input.get();

        // resetMotors();
        dashboard();
    }

        /* 
    * ####################### 
    *   Motor Functions
    * #######################
    */
    public void resetMotors () {
        motor.setRotorPosition(0.0);
        System.out.println("reset" + motor.getRotorPosition());
    }

    public void forward () {
        // motor.set(TalonFXControlMode.PercentOutput, -Constants.TelescopingSubsystem.DEFAULT_SPEED);
        motor.set(-Constants.TelescopingSubsystem.DEFAULT_SPEED);
        encoderCount = rotorPos;
    }

    public void backward() {
        motor.set(Constants.TelescopingSubsystem.DEFAULT_SPEED);
        encoderCount = rotorPos;
    }

    public void stop() {
        motor.set(0);
        encoderCount = rotorPos;
    }

    public void printStuff(){
        System.out.println("Starting encoder count: " + startingEncoderCount);
        System.out.println("Current encoder count: " + rotorPos);
    }

    public void runHigh(){
        // targetPosition = 190000;
        // motor.set(TalonFXControlMode.MotionMagic, 180000); //145k too low standish, 193k
        PositionDutyCycle position = new PositionDutyCycle(180000);
        motor.setControl(position);
        
    }

    public void runHighAuto(){
        // targetPosition = 161290;
        // motor.set(TalonFXControlMode.MotionMagic, 193000); // 142500 143800
        PositionDutyCycle position = new PositionDutyCycle(193000);
        motor.setControl(position);

    }

    public void runMid(){
        // targetPosition = 87191;
        // motor.set(TalonFXControlMode.MotionMagic, 80000); // 88332
        PositionDutyCycle position = new PositionDutyCycle(80000);
        motor.setControl(position);

    }

    // public void runHumanPlayer(){
    //     motor.set(TalonFXControlMode.MotionMagic, 30000);
    // }

    public void runHP(){
        // motor.set(TalonFXControlMode.MotionMagic, 40000); //35000 post-states
        PositionDutyCycle position = new PositionDutyCycle(40000);
        motor.setControl(position);

    }


    public void runZero(){
        // targetPosition = 1;
        // motor.set(TalonFXControlMode.PercentOutput, -Constants.TelescopingSubsystem.DEFAULT_SPEED);
        // System.out.println("000");
        PositionDutyCycle position = new PositionDutyCycle(0);
        motor.setControl(position);

    }

    public void antiLukeFeature(){
        if(encoderCount<0){
            stop();
        }
        else if(encoderCount>235000){
            stop();
        }
    }

    public void testSwitch(){
        click = input.get();
        if(click == false){
            resetMotors();
            // System.out.println("CLICKED");

            PositionDutyCycle zero = new PositionDutyCycle(0);
            PositionDutyCycle position = new PositionDutyCycle(2500);

            motor.setControl(zero);
            motor.setControl(position);

            // motor.set(TalonFXControlMode.PercentOutput, 0);
            // motor.set(TalonFXControlMode.MotionMagic, 2500);
            // stop();
        }
    }

    public void testRunZero(){
        PositionDutyCycle zero = new PositionDutyCycle(0);

        motor.setControl(zero);

        // motor.set(TalonFXControlMode.MotionMagic, 2500);
    }

    private void dashboard () {
        // ShuffleboardTab tab = Shuffleboard.getTab("Intake");
        // tab.add(this);
        encoderCount = motor.getPosition().getValue();
        SmartDashboard.putNumber("Telescope motor", encoderCount);
        // SmartDashboard.putNumber("Telescope target", targetPosition);

        // tab.addString("XFactor State",this::getXFactorStateName);
    }

    public void periodic(){
        testSwitch();
        // antiLukeFeature();
        click = input.get();
        dashboard();

    }
    
}
