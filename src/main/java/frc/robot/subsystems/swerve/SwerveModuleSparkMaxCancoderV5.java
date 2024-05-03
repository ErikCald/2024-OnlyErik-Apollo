package frc.robot.subsystems.swerve;

import static frc.lib.lib2706.ErrorCheck.configureSpark;
import static frc.lib.lib2706.ErrorCheck.errSpark;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.lib2706.controllers.PIDConfig;
import frc.lib.lib2706.swerve.SwerveModuleConstants;
import frc.lib.lib3512.util.CANSparkMaxUtil;
import frc.lib.lib3512.util.CANSparkMaxUtil.Usage;
import frc.robot.Config.CANID;
import frc.robot.Config.NTConfig;
import frc.robot.Config.SwerveConfig;
import frc.robot.subsystems.misc.SparkMaxManagerSubsystem;

/**
 * Represents a Swerve Module with SparkMax motor controllers and CANCoder for steering feedback.
 * This class provides methods for configuring and controlling the steer and drive motors,
 * as well as accessing the measured and desired speed and angle of the module.
 */
public class SwerveModuleSparkMaxCancoderV5 extends SwerveModuleAbstract {
    private CANSparkMax m_steerMotor, m_driveMotor;
    private SparkPIDController m_driveController, m_steerController;
    private RelativeEncoder m_driveEncoder, m_integratedSteerEncoder;
    private CANCoder m_steerCancoder;

    private boolean m_scheduleSyncEncoders = false;

    /**
     * Represents a Swerve Module with SparkMax motor controllers and CANCoder for steering feedback.
     * This class provides methods for configuring and controlling the steer and drive motors,
     * as well as accessing the measured and desired speed and angle of the module.
     *
     * @param moduleConstants The constants specific to this swerve module.
     * @param moduleName The name of the swerve module.
     */
    public SwerveModuleSparkMaxCancoderV5(
            SwerveModuleConstants moduleConstants, String moduleName) {
        super(moduleConstants, moduleName);

        /* Steer Encoder Config */
        m_steerCancoder = new CANCoder(moduleConstants.cancoderCanID);
        configSteerCancoder(moduleConstants.steerOffset);

        /* Steer Motor Config */
        m_steerMotor = new CANSparkMax(moduleConstants.steerCanID, MotorType.kBrushless);
        m_integratedSteerEncoder = m_steerMotor.getEncoder();
        m_steerController = m_steerMotor.getPIDController();
        configSteerMotor();

        /* Drive Motor Config */
        m_driveMotor = new CANSparkMax(moduleConstants.driveCanID, MotorType.kBrushless);
        m_driveEncoder = m_driveMotor.getEncoder();
        m_driveController = m_driveMotor.getPIDController();
        configDriveMotor();

        /* Setup tunable values. This will set the PIDF values for both the drive and steer */
        SwerveModuleAbstract.setupTunableValues(this);

        /* Register CANSparkMaxs to log fault codes */
        SparkMaxManagerSubsystem.getInstance()
                .register(NTConfig.swerveSparkmaxAlertGroup, m_driveMotor, m_steerMotor);

        /* Reset the integrated steering encoder from absolute cancoder */
        setNEOEncoderFromCancoder();
    }

    /**
     * Configures the steer CANCoder
     */
    private void configSteerCancoder(Rotation2d steerOffset) {
        CANCoderConfiguration swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = SwerveConfig.cancoderInvert;
        swerveCanCoderConfig.initializationStrategy =
                SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        m_steerCancoder.configFactoryDefault();
        m_steerCancoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 50);
        m_steerCancoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 50);
        m_steerCancoder.configAllSettings(swerveCanCoderConfig);
    }

    /**
     * Configures the steer motor
     */
    private void configSteerMotor() {
        configureSpark("Steer can timeout", () -> m_steerMotor.setCANTimeout(CANID.CANTIMEOUT_MS));
        configureSpark(
                "Steer restore factory defaults", () -> m_steerMotor.restoreFactoryDefaults());
        configureSpark(
                "Steer set can blocking", () -> m_steerMotor.setCANTimeout(CANID.CANTIMEOUT_MS));
        CANSparkMaxUtil.setCANSparkMaxBusUsage(m_steerMotor, Usage.kAll);
        configureSpark(
                "Steer current limit",
                () -> m_steerMotor.setSmartCurrentLimit(SwerveConfig.steerContinuousCurrentLimit));
        m_steerMotor.setInverted(SwerveConfig.steerInvert);
        configureSpark(
                "Steer idle mode", () -> m_steerMotor.setIdleMode(SwerveConfig.steerIdleMode));
        configureSpark(
                "Steer position conversion factor",
                () ->
                        m_integratedSteerEncoder.setPositionConversionFactor(
                                SwerveConfig.steerConvFactor));
        configureSpark(
                "Steer velocity conversion factor",
                () ->
                        m_integratedSteerEncoder.setVelocityConversionFactor(
                                SwerveConfig.steerVelConvFactor));
        configureSpark("Steer set P", () -> m_steerController.setP(SwerveConfig.steerKP));
        configureSpark("Steer set I", () -> m_steerController.setI(SwerveConfig.steerKI));
        configureSpark("Steer set D", () -> m_steerController.setD(SwerveConfig.steerKD));
        configureSpark("Steer set FF", () -> m_steerController.setFF(0.0));
        configureSpark(
                "Steer set pid wrap min",
                () -> m_steerController.setPositionPIDWrappingMinInput(-Math.PI));
        configureSpark(
                "Steer set pid wrap max",
                () -> m_steerController.setPositionPIDWrappingMaxInput(Math.PI));
        configureSpark(
                "Steer set pid wrap", () -> m_steerController.setPositionPIDWrappingEnabled(true));
        configureSpark(
                "Steer enable Volatage Compensation",
                () -> m_steerMotor.enableVoltageCompensation(SwerveConfig.steerVoltComp));
        configureSpark("Steer set can nonblocking", () -> m_steerMotor.setCANTimeout(0));
    }

    /*
     * Config the drive motor
     */
    private void configDriveMotor() {
        configureSpark("Drive can timeout", () -> m_driveMotor.setCANTimeout(CANID.CANTIMEOUT_MS));
        configureSpark(
                "Drive restore factory defaults", () -> m_driveMotor.restoreFactoryDefaults());
        configureSpark(
                "Drive set can blocking", () -> m_driveMotor.setCANTimeout(CANID.CANTIMEOUT_MS));
        CANSparkMaxUtil.setCANSparkMaxBusUsage(m_driveMotor, Usage.kAll);
        configureSpark(
                "Drive smart current limit",
                () -> m_driveMotor.setSmartCurrentLimit(SwerveConfig.driveContinuousCurrentLimit));
        m_driveMotor.setInverted(SwerveConfig.driveInvert);
        configureSpark(
                "Drive idle mode", () -> m_driveMotor.setIdleMode(SwerveConfig.driveIdleMode));
        configureSpark(
                "Drive velocity conversion factor",
                () -> m_driveEncoder.setVelocityConversionFactor(SwerveConfig.driveConvVelFactor));
        configureSpark(
                "Drive position conversion factor",
                () -> m_driveEncoder.setPositionConversionFactor(SwerveConfig.driveConvPosFactor));
        configureSpark("Drive set P", () -> m_driveController.setP(SwerveConfig.driveKP));
        configureSpark("Drive set I", () -> m_driveController.setI(SwerveConfig.driveKI));
        configureSpark("Drive set D", () -> m_driveController.setD(SwerveConfig.driveKD));
        configureSpark("Drive set FF", () -> m_driveController.setFF(SwerveConfig.driveKFF));
        configureSpark(
                "Drive voltage comp",
                () -> m_driveMotor.enableVoltageCompensation(SwerveConfig.driveVoltComp));
        configureSpark("Drive set position", () -> m_driveEncoder.setPosition(0.0));
    }

    /**
     * Updates the PID configuration for the drive controller.
     *
     * @param pid The PIDConfig object containing the new PID values.
     */
    protected void updateDrivePID(PIDConfig pid) {
        errSpark("Drive set P", m_driveController.setP(pid.kP, pid.pidSlot));
        errSpark("Drive set I", m_driveController.setI(pid.kI, pid.pidSlot));
        errSpark("Drive set D", m_driveController.setD(pid.kD, pid.pidSlot));
        errSpark("Drive set FF", m_driveController.setFF(pid.kF, pid.pidSlot));
        errSpark("Drive update iZone", m_driveController.setIZone(pid.iZone, pid.pidSlot));
    }

    /**
     * Updates the PID configuration for the steer controller.
     *
     * @param pid The PID configuration to update.
     */
    protected void updateSteerPID(PIDConfig pid) {
        errSpark("Steer set P", m_steerController.setP(pid.kP, pid.pidSlot));
        errSpark("Steer set I", m_steerController.setI(pid.kI, pid.pidSlot));
        errSpark("Steer set D", m_steerController.setD(pid.kD, pid.pidSlot));
        errSpark("Steer set FF", m_steerController.setFF(0, pid.pidSlot));
        errSpark("Steer update iZone", m_steerController.setIZone(pid.iZone, pid.pidSlot));
    }

    /**
     * Resets the steer encoder position to the absolute position of the CanCoder.
     */
    @Override
    public void resetToAbsolute() {
        m_scheduleSyncEncoders = true;
    }

    /**
     * Sets the NEO encoder position based on the CANCoder's radians value.
     */
    private void setNEOEncoderFromCancoder() {
        // double absolutePosition = getCanCoder().getRadians() -
        // Math.toRadians(tunableSteerOffset.get());

        m_integratedSteerEncoder.setPosition(getCanCoder().getRadians());
    }

    /**
     * Sets the desired state of the SwerveModule.
     *
     * @param desiredState The desired state of the SwerveModule.
     * @param isOpenLoop   A boolean indicating whether the control is open loop or not.
     */
    @Override
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // Optimize the desired angle to rotate less than 90 degrees
        desiredState = SwerveModuleState.optimize(desiredState, getAngle());

        // Send the desired state to the motors
        setAngle(desiredState.angle);
        setSpeed(desiredState, isOpenLoop);

        // Reset to absolute if scheduled and if the module is not about to move
        double angleError = desiredState.angle.getRadians() - getAngle().getRadians();
        boolean shouldSync =
                m_scheduleSyncEncoders
                        && Math.abs(desiredState.speedMetersPerSecond) < SwerveConfig.syncMPSTol
                        && Math.abs(MathUtil.angleModulus(angleError)) < SwerveConfig.syncRadTol
                        && Math.abs(getDriveVelocity()) < SwerveConfig.syncMPSTol;
        if (shouldSync) {
            setNEOEncoderFromCancoder();
        }
        m_scheduleSyncEncoders = false;

        // Publish the desired state to the network table
        pubDesiredAngle.accept(desiredState.angle.getDegrees());
        pubDesiredSpeed.accept(desiredState.speedMetersPerSecond);
        pubSpeedError.accept(desiredState.speedMetersPerSecond - getDriveVelocity());
        pubAngleError.accept(desiredState.angle.getDegrees() - getAngle().getDegrees());
    }

    /**
     * Sets the drive voltage and steering angle for the swerve module.
     * <p> This is great for characterizing the swerve chassis.
     *
     * @param driveVolts The drive voltage to be applied to the module.
     * @param steeringAngle The steering angle for the module.
     */
    @Override
    public void setVolts(double driveVolts, Rotation2d steeringAngle) {
        m_driveMotor.setVoltage(driveVolts);
        setAngle(steeringAngle);
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
            m_driveMotor.set(percentOutput);
        } else {
            // If we don't want to move and are not moving, then don't move!
            if (Math.abs(speed) < SwerveConfig.driveVelAllowableError
                    && Math.abs(getDriveVelocity()) < SwerveConfig.driveVelAllowableError) {
                m_driveMotor.stopMotor();
            } else {
                // We want to move
                errSpark(
                        "Drive set vel with FF",
                        m_driveController.setReference(
                                speed, ControlType.kVelocity, 0, s_driveFF.calculate(speed)));
            }
        }
    }

    /**
     * Sets the angle of the swerve module.
     *
     * @param angle the desired angle of the swerve module
     */
    private void setAngle(Rotation2d angle) {
        // If within a very small allowable error of the setpoint then don't move
        if (Math.abs(getAngle().getRadians() - angle.getRadians())
                < SwerveConfig.steerPosAllowableError) {
            m_steerMotor.stopMotor();
        } else {
            errSpark(
                    "Steer set angle",
                    m_steerController.setReference(
                            angle.getRadians(), CANSparkBase.ControlType.kPosition));
        }
    }

    /**
     * Returns the angle of the swerve module.
     *
     * @return the angle of the swerve module
     */
    private Rotation2d getAngle() {
        return new Rotation2d(m_integratedSteerEncoder.getPosition());
    }

    /**
     * Returns the velocity of the drive wheel.
     *
     * @return the velocity in meters per second
     */
    private double getDriveVelocity() {
        return m_driveEncoder.getVelocity();
    }

    /**
     * Returns the current position of the drive encoder.
     *
     * @return the drive position in meters
     */
    private double getDrivePosition() {
        return m_driveEncoder.getPosition();
    }

    /**
     * Returns the rotation in degrees calculated from the CanCoder position
     * adjusted by the tunable steer offset.
     *
     * @return the rotation from the cancoder
     */
    private Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(
                m_steerCancoder.getAbsolutePosition() - tunableSteerOffset.get());
    }

    /**
     * Gets the current state of the swerve module.
     *
     * @return The SwerveModuleState object representing the current state.
     */
    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getAngle());
    }

    /**
     * Returns the position of the swerve module.
     *
     * @return the position of the swerve module
     */
    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getAngle());
    }

    /**
     * Returns the velocity of the steering encoder.
     *
     * @return the velocity in radians per second
     */
    @Override
    public double getSteeringVelocity() {
        return m_integratedSteerEncoder.getVelocity();
    }

    /**
     * Updates the state of the SwerveModule periodically.
     * This method is called repeatedly to update the measured speed, angle, and Cancoder angle of the SwerveModule.
     */
    @Override
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
    @Override
    public boolean isModuleSynced() {
        // Calculate the angle error between the NEO encoder and cancoder
        double angleError = getAngle().getRadians() - getCanCoder().getRadians();

        // Wrap the angle to (-pi, pi], get the absolute value, then check if the error is less
        // than the tolerance
        if (Math.abs(MathUtil.angleModulus(angleError)) < SwerveConfig.syncMetersTol) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets the idle mode for the drive and steer motors.
     *
     * @param drive The idle mode for the drive motor.
     * @param steer The idle mode for the steer motor.
     */
    public void setIdleMode(IdleMode drive, IdleMode steer) {
        m_driveMotor.setIdleMode(drive);
        m_steerMotor.setIdleMode(steer);
    }

    /**
     * Stops the drive and steer motors of the swerve module.
     */
    @Override
    public void stopMotors() {
        m_driveMotor.stopMotor();
        m_steerMotor.stopMotor();
    }
}
