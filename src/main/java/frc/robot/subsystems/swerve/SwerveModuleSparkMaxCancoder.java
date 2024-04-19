package frc.robot.subsystems.swerve;

import static frc.lib.lib2706.ErrorCheck.configureSpark;
import static frc.lib.lib2706.ErrorCheck.errSpark;

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

import frc.lib.lib2706.PIDConfig;
import frc.lib.lib2706.TunableDouble;
import frc.lib.lib2706.swerve.SwerveModuleConstants;
import frc.lib.lib3512.util.CANSparkMaxUtil;
import frc.lib.lib3512.util.CANSparkMaxUtil.Usage;
import frc.lib.libYAGSL.SimplifiedCANCoder;
import frc.robot.Config.CANID;
import frc.robot.Config.SwerveConfig;
import frc.robot.subsystems.misc.ErrorTrackingSubsystem;

public class SwerveModuleSparkMaxCancoder extends SwerveModuleAbstract {
    private CANSparkMax steerMotor, driveMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedSteerEncoder;
    private SimplifiedCANCoder steerCancoder;

    private SparkPIDController driveController;
    private SparkPIDController steerController;

    private boolean scheduleSyncEncoders = false;

    /**
     * Represents a Swerve Module with SparkMax motor controllers and CANCoder for steering feedback.
     * This class provides methods for configuring and controlling the steer and drive motors,
     * as well as accessing the measured and desired speed and angle of the module.
     *
     * @param moduleConstants The constants specific to this swerve module.
     * @param moduleName The name of the swerve module.
     */
    public SwerveModuleSparkMaxCancoder(SwerveModuleConstants moduleConstants, String moduleName) {
        super(moduleConstants, moduleName);

        /* Steer Encoder Config */
        steerCancoder =
                new SimplifiedCANCoder(moduleConstants.cancoderCanID, moduleName + " CANCoder");
        configSteerCancoder(moduleConstants.steerOffset);

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

        /* Setup tunable values. This will set the PIDF values for both the drive and steer */
        SwerveModuleAbstract.setupTunableValues(this);

        /* Register CANSparkMaxs to log fault codes */
        ErrorTrackingSubsystem.getInstance().register(steerMotor, driveMotor);

        /* Reset the integrated steering encoder from absolute cancoder */
        setNEOEncoderFromCancoder();

        /* Burn settings to flash */
        burnFlash();
    }

    /**
     * Configures the steer CANCoder
     */
    private void configSteerCancoder(Rotation2d steerOffset) {
        steerCancoder.factoryDefault();
        steerCancoder.configure(SwerveConfig.cancoderInvert, SwerveConfig.cancoderUpdatePeriod);
        steerCancoder.setAbsoluteCancoderOffset(tunableSteerOffset.get());
    }

    /**
     * Configures the steer motor
     */
    private void configSteerMotor() {
        configureSpark("Steer can timeout", () -> steerMotor.setCANTimeout(CANID.CANTIMEOUT_MS));
        configureSpark("Steer restore factory defaults", () -> steerMotor.restoreFactoryDefaults());
        configureSpark(
                "Steer set can blocking", () -> steerMotor.setCANTimeout(CANID.CANTIMEOUT_MS));
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
                () -> steerMotor.enableVoltageCompensation(SwerveConfig.steerVoltComp));
        configureSpark("Steer set can nonblocking", () -> steerMotor.setCANTimeout(0));
    }

    /*
     * Config the drive motor
     */
    private void configDriveMotor() {
        configureSpark("Drive can timeout", () -> driveMotor.setCANTimeout(CANID.CANTIMEOUT_MS));
        configureSpark("Drive restore factory defaults", () -> driveMotor.restoreFactoryDefaults());
        configureSpark(
                "Drive set can blocking", () -> driveMotor.setCANTimeout(CANID.CANTIMEOUT_MS));
        CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
        configureSpark(
                "Drive smart current limit",
                () -> driveMotor.setSmartCurrentLimit(SwerveConfig.driveContinuousCurrentLimit));
        driveMotor.setInverted(SwerveConfig.driveInvert);
        configureSpark("Drive idle mode", () -> driveMotor.setIdleMode(SwerveConfig.driveIdleMode));
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
                () -> driveMotor.enableVoltageCompensation(SwerveConfig.driveVoltComp));
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
     * Updates the PID configuration for the drive controller.
     *
     * @param pid The PIDConfig object containing the new PID values.
     */
    protected void updateDrivePID(PIDConfig pid) {
        errSpark("Drive set P", driveController.setP(pid.kP, pid.pidSlot));
        errSpark("Drive set I", driveController.setI(pid.kI, pid.pidSlot));
        errSpark("Drive set D", driveController.setD(pid.kD, pid.pidSlot));
        errSpark("Drive set FF", driveController.setFF(pid.kF, pid.pidSlot));
        errSpark("Drive update iZone", driveController.setIZone(pid.iZone, pid.pidSlot));
    }

    /**
     * Updates the PID configuration for the steer controller.
     *
     * @param pid The PID configuration to update.
     */
    protected void updateSteerPID(PIDConfig pid) {
        errSpark("Steer set P", steerController.setP(pid.kP, pid.pidSlot));
        errSpark("Steer set I", steerController.setI(pid.kI, pid.pidSlot));
        errSpark("Steer set D", steerController.setD(pid.kD, pid.pidSlot));
        errSpark("Steer set FF", steerController.setFF(0, pid.pidSlot));
        errSpark("Steer update iZone", steerController.setIZone(pid.iZone, pid.pidSlot));
    }

    /**
     * Resets the steer encoder position to the absolute position of the CanCoder.
     */
    @Override
    public void resetToAbsolute() {
        scheduleSyncEncoders = true;
    }

    /**
     * Sets the NEO encoder position based on the CANCoder's radians value.
     */
    private void setNEOEncoderFromCancoder() {
        integratedSteerEncoder.setPosition(getCanCoder().getRadians());
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
                scheduleSyncEncoders
                        && Math.abs(desiredState.speedMetersPerSecond) < SwerveConfig.syncMPSTol
                        && Math.abs(MathUtil.angleModulus(angleError)) < SwerveConfig.syncRadTol;
        if (shouldSync) {
            setNEOEncoderFromCancoder();
        }
        scheduleSyncEncoders = false;

        // Publish the desired state to the network table
        pubDesiredAngle.accept(desiredState.angle.getDegrees());
        pubDesiredSpeed.accept(desiredState.speedMetersPerSecond);
        pubSpeedError.accept(desiredState.speedMetersPerSecond - getDriveVelocity());
        pubAngleError.accept(desiredState.angle.getDegrees() - getAngle().getDegrees());
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
                            speed, ControlType.kVelocity, 0, s_driveFF.calculate(speed)));
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
        return integratedSteerEncoder.getVelocity();
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

        TunableDouble.ifChanged(
                hashCode(),
                () -> steerCancoder.setAbsoluteCancoderOffset(tunableSteerOffset.get()),
                tunableSteerOffset);
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
        driveMotor.setIdleMode(drive);
        steerMotor.setIdleMode(steer);
    }

    /**
     * Stops the drive and steer motors of the swerve module.
     */
    @Override
    public void stopMotors() {
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }
}
