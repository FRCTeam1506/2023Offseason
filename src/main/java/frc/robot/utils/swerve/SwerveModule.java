package frc.robot.utils.swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.robot.Constants;
import frc.robot.Robot;

// import com.ctre.phoenix.ErrorCode;
// import com.ctre.phoenix6.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.DemandType;
// import com.ctre.phoenix6.motorcontrol.can.TalonFX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;



public class SwerveModule {
    public int moduleNumber;

    private TalonFX driveMotor;
    private TalonFX angleMotor;
    private CANcoder angleEncoder;
    
    private double angleOffset;

    private double lastAngle;

    public double CANcoderInitTime = 0.0;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SwerveDrivetrain.FF_kS, Constants.SwerveDrivetrain.FF_kV, Constants.SwerveDrivetrain.FF_kA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;

        this.angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        this.angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Drive Motor Config */
        this.driveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        /* Angle Motor Config */
        this.angleMotor = new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();

        this.lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);    // Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveDrivetrain.MAX_SPEED;
            driveMotor.set(percentOutput);
            // this.driveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.SwerveDrivetrain.WHEEL_CIRCUMFERENCE, Constants.SwerveDrivetrain.DRIVE_GEAR_RATIO);
            // driveMotor.set(velocity);
            VelocityDutyCycle vel = new VelocityDutyCycle(velocity);
            // this.driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
            driveMotor.setControl(vel);
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveDrivetrain.MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle.getDegrees();   // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        // this.angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, Constants.SwerveDrivetrain.ANGLE_GEAR_RATIO)); 

        PositionDutyCycle position = new PositionDutyCycle(Conversions.degreesToFalcon(angle, Constants.SwerveDrivetrain.ANGLE_GEAR_RATIO));
        angleMotor.setControl(position);

        this.lastAngle = angle;
    }

     private void waitForCanCoder(){
        /*
         * Wait for CanCoder. (up to 1000ms)
         *
         * preventing race condition during program startup
         */

         //ABSOLUTELY BROKE THIS FEATURE CTRE PHOENIX 6 BECAUSE getLastError() no longer exists
        // for (int i = 0; i < 100; ++i) {
        //     angleEncoder.getAbsolutePosition();
        //     if (angleEncoder.getLastError() == ErrorCode.OK) {
        //         break;
        //     }
        //     Timer.delay(0.010);            
        //     CANcoderInitTime += 10;
        // }

        Timer.delay(0.500);            

    }


    public void resetToAbsolute() {
        waitForCanCoder();
        //not needed because in phoenix pro/6, the motors automatically reset to absolute upon initialization
        // double absolutePosition = Conversions.degreesToFalcon(this.getCanCoder().getDegrees() - angleOffset, Constants.SwerveDrivetrain.ANGLE_GEAR_RATIO);
        // this.angleMotor.setSelectedSensorPosition(absolutePosition); //statuscode import maybe? for for the method above this one
    }

    private void configAngleEncoder() {
        var motorConfig = new CANcoderConfiguration();
        angleEncoder.getConfigurator().apply(motorConfig);

        // this.angleEncoder.configFactoryDefault();
        // angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCANCoderConfig);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANCoderConfig);
    }

    private void configAngleMotor() {
        // this.angleMotor.configFactoryDefault();

        var motorConfig = new TalonFXConfiguration();
        angleMotor.getConfigurator().apply(motorConfig);

        // this.angleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleTalonFXConfig);

        angleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleTalonFXConfig);

        this.angleMotor.setInverted(Constants.SwerveDrivetrain.ANGLE_MOTOR_INVERTED);
        // this.angleMotor.setNeutralMode(Constants.SwerveDrivetrain.ANGLE_NEUTRAL_MODE);

        DutyCycleOut duty = new DutyCycleOut(1, true, true);
        angleMotor.setControl(duty);

        Timer.delay(.1);
        resetToAbsolute();
    }

    private void configDriveMotor() {        
        // this.driveMotor.configFactoryDefault();

        var motorConfig = new TalonFXConfiguration();
        driveMotor.getConfigurator().apply(motorConfig);

        // this.driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveTalonFXConfig);
        driveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleTalonFXConfig);

        this.driveMotor.setInverted(Constants.SwerveDrivetrain.DRIVE_MOTOR_INVERTED);

        // this.driveMotor.setNeutralMode(Constants.SwerveDrivetrain.DRIVE_NEUTRAL_MODE);
        DutyCycleOut duty = new DutyCycleOut(1, true, true);
        driveMotor.setControl(duty);

        // this.driveMotor.setSelectedSensorPosition(0); not necessary in phoenix 6
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(this.driveMotor.getSelectedSensorVelocity(), Constants.SwerveDrivetrain.WHEEL_CIRCUMFERENCE, Constants.SwerveDrivetrain.DRIVE_GEAR_RATIO);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(this.angleMotor.getSelectedSensorPosition(), Constants.SwerveDrivetrain.ANGLE_GEAR_RATIO));
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition(){
        var rotorPosSignal = driveMotor.getRotorPosition();
        var rotorPos = rotorPosSignal.getValue();

        // return new SwerveModulePosition(
        //     Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(), Constants.SwerveDrivetrain.WHEEL_CIRCUMFERENCE, Constants.SwerveDrivetrain.DRIVE_GEAR_RATIO), 
        //     getAngle()
        // );

        return new SwerveModulePosition(
            Conversions.falconToMeters(rotorPos, Constants.SwerveDrivetrain.WHEEL_CIRCUMFERENCE, Constants.SwerveDrivetrain.DRIVE_GEAR_RATIO), 
            getAngle()
        );

    }

    public Rotation2d getAngle() {
        var rotorPosSignal = angleMotor.getRotorPosition();
        var rotorPos = rotorPosSignal.getValue();

        return Rotation2d.fromDegrees(Conversions.falconToDegrees(rotorPos, Constants.SwerveDrivetrain.ANGLE_GEAR_RATIO));
    };
}
