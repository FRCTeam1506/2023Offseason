package frc.lib.util;

// import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;



// import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
// import com.ctre.phoenix.sensors.AbsoluteSensorRange;
// import com.ctre.phoenix.sensors.CANCoderConfiguration;
// import com.ctre.phoenix.sensors.SensorInitializationStrategy;
// import com.ctre.phoenix.sensors.SensorTimeBase;

import frc.robot.Constants;

public final class CTREConfigs {
    
    public TalonFXConfiguration swerveDriveTalonFXConfig;
    public TalonFXConfiguration swerveAngleTalonFXConfig;
    public CANcoderConfiguration swerveCANCoderConfig;

    public CTREConfigs () {
        this.swerveDriveTalonFXConfig   = new TalonFXConfiguration();
        this.swerveAngleTalonFXConfig   = new TalonFXConfiguration();
        this.swerveCANCoderConfig       = new CANcoderConfiguration();

        /* Swerve Angle Motor Configurations */
        // SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
        //     Constants.SwerveDrivetrain.ANGLE_ENABLE_CURRENT_LIMIT, 
        //     Constants.SwerveDrivetrain.ANGLE_CONTINUOUS_CL, 
        //     Constants.SwerveDrivetrain.ANGLE_PEAK_CL, 
        //     Constants.SwerveDrivetrain.ANGLE_PEAK_CURRENT_DURATION);

            CurrentLimitsConfigs angleSupplyLimit = new CurrentLimitsConfigs();
            angleSupplyLimit.SupplyCurrentLimit = 40; // Limit to 1 amps
            angleSupplyLimit.SupplyCurrentThreshold = 25;
            angleSupplyLimit.SupplyTimeThreshold = 0.1;
            angleSupplyLimit.SupplyCurrentLimitEnable = Constants.SwerveDrivetrain.ANGLE_ENABLE_CURRENT_LIMIT;
        
            swerveAngleTalonFXConfig.CurrentLimits = angleSupplyLimit;
    

        // this.swerveAngleTalonFXConfig.slot0.kP = Constants.SwerveDrivetrain.ANGLE_kP;
        // this.swerveAngleTalonFXConfig.slot0.kI = Constants.SwerveDrivetrain.ANGLE_kI;
        // this.swerveAngleTalonFXConfig.slot0.kD = Constants.SwerveDrivetrain.ANGLE_kD;
        // this.swerveAngleTalonFXConfig.slot0.kF = Constants.SwerveDrivetrain.ANGLE_kF;
        // this.swerveAngleTalonFXConfig.supplyCurrLimit = angleSupplyLimit;
        // this.swerveAngleTalonFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

        var angleSlot0 = new Slot0Configs();
        angleSlot0.kV = Constants.SwerveDrivetrain.ANGLE_kF;
        angleSlot0.kP = Constants.SwerveDrivetrain.ANGLE_kP;
        angleSlot0.kI = Constants.SwerveDrivetrain.ANGLE_kI;
        angleSlot0.kD = Constants.SwerveDrivetrain.ANGLE_kD;

        swerveAngleTalonFXConfig.Slot0 = angleSlot0;



        /* Swerve Drive Motor Configuration */
        // SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
        //     Constants.SwerveDrivetrain.DRIVE_ENABLE_CURRENT_LIMIT, 
        //     Constants.SwerveDrivetrain.DRIVE_CONTINUOUS_CL, 
        //     Constants.SwerveDrivetrain.DRIVE_PEAK_CL, 
        //     Constants.SwerveDrivetrain.DRIVE_PEAK_CURRENT_DURATION);

        CurrentLimitsConfigs driveSupplyLimit = new CurrentLimitsConfigs();
        driveSupplyLimit.SupplyCurrentLimit = 60; // Limit to 1 amps
        driveSupplyLimit.SupplyCurrentThreshold = 35;
        driveSupplyLimit.SupplyTimeThreshold = 0.1;
        driveSupplyLimit.SupplyCurrentLimitEnable = Constants.SwerveDrivetrain.ANGLE_ENABLE_CURRENT_LIMIT;
        
        swerveDriveTalonFXConfig.CurrentLimits = driveSupplyLimit;


        var driveSlot0 = new Slot0Configs();
        driveSlot0.kV = Constants.SwerveDrivetrain.DRIVE_kF;
        driveSlot0.kP = Constants.SwerveDrivetrain.DRIVE_kP;
        driveSlot0.kI = Constants.SwerveDrivetrain.DRIVE_kI;
        driveSlot0.kD = Constants.SwerveDrivetrain.DRIVE_kD;

        swerveDriveTalonFXConfig.Slot0 = driveSlot0;


        // this.swerveDriveTalonFXConfig.slot0.kP = Constants.SwerveDrivetrain.DRIVE_kP;
        // this.swerveDriveTalonFXConfig.slot0.kI = Constants.SwerveDrivetrain.DRIVE_kI;
        // this.swerveDriveTalonFXConfig.slot0.kD = Constants.SwerveDrivetrain.DRIVE_kD;
        // this.swerveDriveTalonFXConfig.slot0.kF = Constants.SwerveDrivetrain.DRIVE_kF;        
        // this.swerveDriveTalonFXConfig.supplyCurrLimit = driveSupplyLimit;
        // this.swerveDriveTalonFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        // this.swerveDriveTalonFXConfig.openloopRamp = Constants.SwerveDrivetrain.OPEN_LOOP_RAMP;
        // this.swerveDriveTalonFXConfig.closedloopRamp = Constants.SwerveDrivetrain.CLOSED_LOOP_RAMP;

        ClosedLoopRampsConfigs loop = new ClosedLoopRampsConfigs();
        loop.DutyCycleClosedLoopRampPeriod = Constants.SwerveDrivetrain.CLOSED_LOOP_RAMP;
        loop.VoltageClosedLoopRampPeriod = Constants.SwerveDrivetrain.CLOSED_LOOP_RAMP;

        OpenLoopRampsConfigs open = new OpenLoopRampsConfigs();
        open.DutyCycleOpenLoopRampPeriod = Constants.SwerveDrivetrain.OPEN_LOOP_RAMP;
        open.VoltageOpenLoopRampPeriod = Constants.SwerveDrivetrain.OPEN_LOOP_RAMP;

        swerveDriveTalonFXConfig.ClosedLoopRamps = loop;
        swerveDriveTalonFXConfig.OpenLoopRamps = open;

        
        /* Swerve CANCoder Configuration */ 
        // this.swerveCANCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        // this.swerveCANCoderConfig.sensorDirection = Constants.SwerveDrivetrain.CAN_CODER_INVERTED;
        // this.swerveCANCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        // this.swerveCANCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        swerveCANCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        swerveCANCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; //not sure, probably tho
        //know that the third one (boot to absolute position) was depreciated, but not sure about the fourth one(time)
    }

}
