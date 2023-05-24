package frc.robot.subsystems;

import frc.robot.Constants;
import frc.lib.math.Conversions;


// import com.ctre.phoenix.motorcontrol.ControlFrame;
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
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.jni.ControlJNI;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.unmanaged.Unmanaged;


public class IntakeSubsystem extends SubsystemBase {

    Conversions conversions = new Conversions();

    private DoubleSolenoid solenoid;
    private TalonFX motor = new TalonFX(Constants.IntakeSubsystem.MOTOR_ID);
    double encoderCount = motor.getPosition().getValue();


    public IntakeSubsystem (PneumaticHub hub) {
        solenoid = hub.makeDoubleSolenoid(Constants.IntakeSubsystem.SolenoidId1, Constants.IntakeSubsystem.SolenoidId2);
        motor.getConfigurator().apply(new TalonFXConfiguration());
        //could not convert to pro - next line
        // motor.setControlFramePeriod(ControlJNI.Control_3_General, 100);
        motor.setInverted(false);
        
        //current limit burn out the motor at states
        TalonFXConfiguration intakeMotorConfig   = new TalonFXConfiguration();
        motor.CurrentLimits.StatorCurrentLimitEnable = false;

        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.SwerveDrivetrain.DRIVE_ENABLE_CURRENT_LIMIT, 
            Constants.SwerveDrivetrain.DRIVE_CONTINUOUS_CL, 
            Constants.SwerveDrivetrain.DRIVE_PEAK_CL, 
            Constants.SwerveDrivetrain.DRIVE_PEAK_CURRENT_DURATION);

        intakeMotorConfig.supplyCurrLimit = driveSupplyLimit;
        motor.configAllSettings(intakeMotorConfig);


        //set motor to brake
        this.motor.setNeutralMode(Constants.SwerveDrivetrain.DRIVE_NEUTRAL_MODE);
        dashboard();
    }

    public void intake (double x) {
        //previously x was defaultSpeed
        motor.set(TalonFXControlMode.PercentOutput, x);
        // System.out.println("Encoder: " + encoderCount);
    }

    public void intakeDefSpeed(){
        if(Constants.CandleSubsystem.cone == true){
            motor.set(TalonFXControlMode.PercentOutput, -Constants.IntakeSubsystem.CONE_DEFAULT_INTAKE_SPEED);
        }
        else{
            motor.set(TalonFXControlMode.PercentOutput, Constants.IntakeSubsystem.CUBE_DEFAULT_INTAKE_SPEED);
        }

    }

    public void intakeSlowSpeed(){
        if(Constants.CandleSubsystem.cone == true){
            motor.set(TalonFXControlMode.PercentOutput, 0.2);
        }
        else{
            motor.set(TalonFXControlMode.PercentOutput, -0.2);
        }

    }

    public void tapAuto(){ //to not drop the cone at start of auto
        motor.set(TalonFXControlMode.PercentOutput, -0.2);
    }

    public void tapAutoCube(){ //to not drop the cone at start of auto
        motor.set(TalonFXControlMode.PercentOutput, 0.1);
    }


    public void outtakeDefSpeed(){
        if(Constants.CandleSubsystem.cone == true){
            motor.set(TalonFXControlMode.PercentOutput, Constants.IntakeSubsystem.CONE_DEFAULT_OUTTAKE_SPEED);
        }
        else{
            motor.set(TalonFXControlMode.PercentOutput, -Constants.IntakeSubsystem.CUBE_DEFAULT_OUTTAKE_SPEED);
        }
    }

    public void pneumaticRetract(){
        solenoid.set(Value.kReverse);
        System.out.println("pneumatic");
    }

    public void pneumaticExtract(){
        solenoid.set(Value.kForward);
        System.out.println("pneumatic2");

    }

    public void cubeSlow(){
        Constants.IntakeSubsystem.CUBE_DEFAULT_OUTTAKE_SPEED = 0.3;
    }
    
    public void cubeFast(){
        Constants.IntakeSubsystem.CUBE_DEFAULT_OUTTAKE_SPEED = 1.0;
    }


    public void intakeRPM(double rpm){        
        //gearratio is second number, set it equal to 1 for now
        motor.set(TalonFXControlMode.Velocity, Conversions.RPMToFalcon(rpm/2, 1));
    }

    public void outtake () {
        motor.set(TalonFXControlMode.PercentOutput, -0.2); //-0.45
    }

    public void stop () {
        motor.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    private void dashboard () {
        // ShuffleboardTab tab = Shuffleboard.getTab("Intake");
        // tab.add(this);
        // tab.addString("XFactor State",this::getXFactorStateName);
    }

    
}