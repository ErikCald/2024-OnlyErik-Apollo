package frc.robot.subsystems.swerve;

import static frc.lib.lib2706.ErrorCheck.configureSpark;
import static frc.lib.lib2706.ErrorCheck.errSpark;
import static frc.lib.lib2706.NTUtil.createDoublePublisherFast;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;

import frc.lib.lib2706.TunableDouble;
import frc.lib.lib2706.swerve.SwerveModuleConstants;
import frc.lib.lib3512.util.CANCoderUtil;
import frc.lib.lib3512.util.CANCoderUtil.CCUsage;
import frc.lib.lib3512.util.CANSparkMaxUtil;
import frc.lib.lib3512.util.CANSparkMaxUtil.Usage;
import frc.robot.Config.NTConfig;
import frc.robot.Config.SwerveConfig;
import frc.robot.Robot;
import frc.robot.subsystems.misc.ErrorTrackingSubsystem;

public class SwerveModule {
    private CANSparkMax steerMotor, driveMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedSteerEncoder;
    private CANCoder steerCancoder;

    private SparkPIDController driveController;
    private SparkPIDController steerController;

    private SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(
                    SwerveConfig.driveKS, SwerveConfig.driveKV, SwerveConfig.driveKA);

    private NetworkTable moduleTable;
    private DoublePublisher pubMeasuredSpeed, pubDesiredSpeed, pubSpeedError;
    private DoublePublisher pubMeasuredAngle, pubDesiredAngle, pubAngleError;
    private DoublePublisher pubCancoderAngle;

    private TunableDouble tunableSteerOffset;

    public SwerveModule(SwerveModuleConstants moduleConstants, String ModuleName) {
        /* Steer Encoder Config */
        steerCancoder = new CANCoder(moduleConstants.cancoderCanID);
        configSteerCancoder();

        /* Steer Motor Config */
        steerMotor = new CANSparkMax(moduleConstants.steerCanID, MotorType.kBrushless);
        integratedSteerEncoder = steerMotor.getEncoder();
        steerController = steerMotor.getPIDController();
        configSteerMotor();

        /* Drive Motor Config */
        driveMotor = new CANSparkMax(moduleConstants.driveCanID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getPIDController();
        configDriveMotor();

        /* Networktable Setup */
        moduleTable = NTConfig.swerveTable.getSubTable("SwerveModule" + ModuleName);

        pubMeasuredSpeed = createDoublePublisherFast(moduleTable, "Measured Speed (mps)");
        pubDesiredSpeed = createDoublePublisherFast(moduleTable, "Desired Speed (mps)");
        pubSpeedError = createDoublePublisherFast(moduleTable, " Speed Error (mps)");

        pubMeasuredAngle = createDoublePublisherFast(moduleTable, "Measured Angle (deg)");
        pubDesiredAngle = createDoublePublisherFast(moduleTable, "Desired Angle (deg)");
        pubAngleError = createDoublePublisherFast(moduleTable, "Angle Error (deg)");

        pubCancoderAngle = createDoublePublisherFast(moduleTable, "Cancoder Angle (deg)");

        tunableSteerOffset =
                new TunableDouble(
                        "Angle Offset (deg)",
                        moduleTable,
                        moduleConstants.steerOffset.getDegrees());

        /* Register CANSparkMaxs to log fault codes */
        ErrorTrackingSubsystem.getInstance().register(steerMotor, driveMotor);

        /* Reset the integrated steering encoder from absolute cancoder */
        resetToAbsolute();

        /* Burn settings to flash */
        burnFlash();
    }

    /**
     * Configures the steer CANCoder
     */
    private void configSteerCancoder() {
        steerCancoder.configFactoryDefault();
        CANCoderUtil.setCANCoderBusUsage(steerCancoder, CCUsage.kMinimal);
        steerCancoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    /**
     * Configures the steer motor
     */
    private void configSteerMotor() {
        configureSpark("Steer restore factory defaults", () -> steerMotor.restoreFactoryDefaults());
        CANSparkMaxUtil.setCANSparkMaxBusUsage(steerMotor, Usage.kAll);
        configureSpark(
                "Steer current limit",
                () -> steerMotor.setSmartCurrentLimit(SwerveConfig.steerContinuousCurrentLimit));
        steerMotor.setInverted(SwerveConfig.steerInvert);
        configureSpark("Steer idle mode", () -> steerMotor.setIdleMode(SwerveConfig.steerIdleMode));
        configureSpark(
                "Steer position conversion factor",
                () ->
                        integratedSteerEncoder.setPositionConversionFactor(
                                SwerveConfig.steerConvFactor));
        configureSpark(
                "Steer velocity conversion factor",
                () ->
                        integratedSteerEncoder.setVelocityConversionFactor(
                                SwerveConfig.steerVelConvFactor));
        configureSpark("Steer set P", () -> steerController.setP(SwerveConfig.steerKP));
        configureSpark("Steer set I", () -> steerController.setI(SwerveConfig.steerKI));
        configureSpark("Steer set D", () -> steerController.setD(SwerveConfig.steerKD));
        configureSpark("Steer set FF", () -> steerController.setFF(0.0));
        configureSpark(
                "Steer set pid wrap min", () -> steerController.setPositionPIDWrappingMinInput(0));
        configureSpark(
                "Steer set pid wrap max",
                () -> steerController.setPositionPIDWrappingMaxInput(2 * Math.PI));
        configureSpark(
                "Steer set pid wrap", () -> steerController.setPositionPIDWrappingEnabled(true));
        configureSpark(
                "Steer enable Volatage Compensation",
                () -> steerMotor.enableVoltageCompensation(SwerveConfig.voltageComp));
    }

    /*
     * Config the drive motor
     */
    private void configDriveMotor() {
        configureSpark("Drive factory defaults", () -> driveMotor.restoreFactoryDefaults());
        CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
        configureSpark(
                "Drive smart current limit",
                () -> driveMotor.setSmartCurrentLimit(SwerveConfig.driveContinuousCurrentLimit));
        driveMotor.setInverted(SwerveConfig.driveInvert);
        configureSpark(
                "Drive idle mode", () -> driveMotor.setIdleMode(SwerveConfig.driveNeutralMode));
        configureSpark(
                "Drive velocity conversion factor",
                () -> driveEncoder.setVelocityConversionFactor(SwerveConfig.driveConvVelFactor));
        configureSpark(
                "Drive position conversion factor",
                () -> driveEncoder.setPositionConversionFactor(SwerveConfig.driveConvPosFactor));
        configureSpark("Drive set P", () -> driveController.setP(SwerveConfig.driveKP));
        configureSpark("Drive set I", () -> driveController.setI(SwerveConfig.driveKI));
        configureSpark("Drive set D", () -> driveController.setD(SwerveConfig.driveKD));
        configureSpark("Drive set FF", () -> driveController.setFF(SwerveConfig.driveKFF));
        configureSpark(
                "Drive voltage comp",
                () -> driveMotor.enableVoltageCompensation(SwerveConfig.voltageComp));
        configureSpark("Drive set position", () -> driveEncoder.setPosition(0.0));
    }

    /**
     * Save the configurations from flash to EEPROM.
     */
    private void burnFlash() {
        try {
            Thread.sleep(200);
        } catch (Exception e) {
        }

        driveMotor.burnFlash();
        steerMotor.burnFlash();
    }

    /**
     * Resets the steer encoder position to the absolute position of the CanCoder.
     */
    public void resetToAbsolute() {
        integratedSteerEncoder.setPosition(getCanCoder().getRadians());
    }

    /**
     * Sets the desired state of the SwerveModule.
     *
     * @param desiredState The desired state of the SwerveModule.
     * @param isOpenLoop   A boolean indicating whether the control is open loop or not.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // Optimize the desired angle to rotate less than 90 degrees
        desiredState = SwerveModuleState.optimize(desiredState, getAngle());

        // Send the desired state to the motors
        setAngle(desiredState.angle);
        setSpeed(desiredState, isOpenLoop);

        // Publish the desired state to the network table
        pubDesiredAngle.accept(desiredState.angle.getDegrees());
        pubDesiredSpeed.accept(desiredState.speedMetersPerSecond);
        pubSpeedError.accept(desiredState.speedMetersPerSecond - getDriveVelocity());
        pubAngleError.accept(desiredState.angle.getDegrees() - getAngle().getDegrees());
    }

    /**
     * Sets the feedforward value for the swerve module.
     *
     * @param newFeedforward the new feedforward value to be set
     */
    public void setFeedforward(SimpleMotorFeedforward newFeedforward) {
        feedforward = newFeedforward;
    }

    /**
     * Sets the speed of the swerve module based on the desired state.
     *
     * @param desiredState The desired state of the swerve module.
     * @param isOpenLoop   A boolean indicating whether the control is open loop or closed loop.
     */
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        double speed =
                desiredState.speedMetersPerSecond * desiredState.angle.minus(getAngle()).getCos();

        if (isOpenLoop) {
            double percentOutput = speed / SwerveConfig.maxSpeed;
            driveMotor.set(percentOutput);
        } else {
            errSpark(
                    "Drive set FF",
                    driveController.setReference(
                            speed, ControlType.kVelocity, 0, feedforward.calculate(speed)));
        }
    }

    /**
     * Sets the angle of the swerve module.
     *
     * @param angle the desired angle of the swerve module
     */
    private void setAngle(Rotation2d angle) {
        errSpark(
                "Steer set reference",
                steerController.setReference(
                        angle.getRadians(), CANSparkBase.ControlType.kPosition));
    }

    /**
     * Returns the angle of the swerve module.
     *
     * @return the angle of the swerve module
     */
    private Rotation2d getAngle() {
        return new Rotation2d(integratedSteerEncoder.getPosition());
    }

    /**
     * Returns the velocity of the drive wheel.
     *
     * @return the velocity in meters per second
     */
    private double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Returns the current position of the drive encoder.
     *
     * @return the drive position in meters
     */
    private double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /**
     * Returns the rotation in degrees calculated from the CanCoder position
     * adjusted by the tunable steer offset.
     *
     * @return the rotation from the cancoder
     */
    private Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(
                steerCancoder.getAbsolutePosition() - tunableSteerOffset.get());
    }

    /**
     * Returns the velocity of the steering encoder.
     *
     * @return the velocity in radians per second
     */
    public double getSteeringVelocity() {
        return integratedSteerEncoder.getVelocity();
    }

    /**
     * Gets the current state of the swerve module.
     *
     * @return The SwerveModuleState object representing the current state.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getAngle());
    }

    /**
     * Returns the position of the swerve module.
     *
     * @return the position of the swerve module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getAngle());
    }

    /**
     * Updates the state of the SwerveModule periodically.
     * This method is called repeatedly to update the measured speed, angle, and Cancoder angle of the SwerveModule.
     */
    public void periodic() {
        pubMeasuredSpeed.accept(getDriveVelocity());
        pubMeasuredAngle.accept(getAngle().getDegrees());
        pubCancoderAngle.accept(getCanCoder().getDegrees());
    }

    /**
     * Checks if the swerve module is synchronized between the NEO encoder and cancoder.
     *
     * @return true if the module is synchronized, false otherwise
     */
    public boolean isModuleSynced() {
        // Calculate the angle error between the NEO encoder and cancoder
        double angleError = getAngle().getDegrees() - getCanCoder().getDegrees();

        // Wrap the angle to (-180, 180], get the absolute value, then check if the error is less
        // than the tolerance
        if (Math.abs(MathUtil.inputModulus(angleError, -180, 180)) < SwerveConfig.synchTolerance) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Stops the drive and steer motors of the swerve module.
     */
    public void stopMotors() {
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }
}
