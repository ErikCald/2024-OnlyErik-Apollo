// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.lib2706.PIDConfig;
import frc.lib.lib2706.swerve.SwerveModuleConstants;
import frc.robot.Config.GeneralConfig;
import frc.robot.Config.SwerveConfig;

/** Add your docs here. */
public class SwerveModuleSim extends SwerveModuleAbstract {
    private final DCMotorSim m_driveMotorSim, m_steerMotorSim;
    private final PIDController m_drivePID, m_steerPID;

    private final Rotation2d m_steerStartingAngle;
    private final SlewRateLimiter m_driveVoltsLimiter = new SlewRateLimiter(2.5);

    private boolean m_steerBrakeMode, m_driveBrakeMode;

    public SwerveModuleSim(SwerveModuleConstants constants, String name) {
        super(constants, name);

        m_steerStartingAngle = constants.steerOffset;

        m_driveMotorSim = new DCMotorSim(DCMotor.getNEO(1), SwerveConfig.driveGearRatio, 0.025);
        m_steerMotorSim = new DCMotorSim(DCMotor.getNEO(1), SwerveConfig.steerGearRatio, 0.004);

        m_drivePID =
                new PIDController(SwerveConfig.driveKP, SwerveConfig.driveKI, SwerveConfig.driveKD);
        m_steerPID =
                new PIDController(SwerveConfig.steerKP, SwerveConfig.steerKI, SwerveConfig.steerKD);
        m_steerPID.enableContinuousInput(-Math.PI, Math.PI);

        m_driveBrakeMode = SwerveConfig.driveIdleMode == IdleMode.kBrake;
        m_steerBrakeMode = SwerveConfig.steerIdleMode == IdleMode.kBrake;
    }

    /**
     * Updates the drive and steer motor simulations periodically.
     */
    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            stopMotors();
        }

        m_driveMotorSim.update(GeneralConfig.loopPeriodSecs);
        m_steerMotorSim.update(GeneralConfig.loopPeriodSecs);

        pubMeasuredSpeed.accept(getDriveVelocity());
        pubMeasuredAngle.accept(getAngle().getDegrees());
        pubCancoderAngle.accept(getAngle().getDegrees());
    }

    /**
     * Updates the drive PID controller with the specified PID configuration.
     *
     * @param pid The PID configuration to update the drive PID controller with.
     */
    protected void updateDrivePID(PIDConfig pid) {
        m_drivePID.setPID(pid.kP, pid.kI, pid.kD);
        m_drivePID.setIZone(pid.iZone);
    }

    /**
     * Updates the PID parameters for the steering control of the swerve module.
     *
     * @param pid The PIDConfig object containing the new PID parameters.
     */
    protected void updateSteerPID(PIDConfig pid) {
        m_steerPID.setPID(pid.kP, pid.kI, pid.kD);
        m_steerPID.setIZone(pid.iZone);
    }

    /**
     * Resets the steer encoder position to the absolute position of the CanCoder.
     *
     * Do nothing for simulation.
     */
    public void resetToAbsolute() {}

    /**
     * Sets the desired state of the SwerveModule.
     *
     * @param desiredState The desired state of the SwerveModule.
     * @param isOpenLoop   A boolean indicating whether the control is open loop or not.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getAngle());

        double steerVolts =
                m_steerPID.calculate(getAngle().getRadians(), desiredState.angle.getRadians());

        double driveVolts = 0;
        if (isOpenLoop) {
            driveVolts =
                    desiredState.speedMetersPerSecond
                            / SwerveConfig.maxSpeed
                            * SwerveConfig.driveVoltComp;
        } else {
            driveVolts =
                    m_drivePID.calculate(getDriveVelocity(), desiredState.speedMetersPerSecond)
                            + s_driveFF.calculate(desiredState.speedMetersPerSecond);
        }

        driveVolts =
                MathUtil.clamp(driveVolts, -SwerveConfig.driveVoltComp, SwerveConfig.driveVoltComp);
        steerVolts =
                MathUtil.clamp(steerVolts, -SwerveConfig.steerVoltComp, SwerveConfig.steerVoltComp);

        if (!m_driveBrakeMode && DriverStation.isDisabled()) {
            driveVolts = m_driveVoltsLimiter.calculate(driveVolts);
        }

        m_driveMotorSim.setInputVoltage(driveVolts);
        m_steerMotorSim.setInputVoltage(steerVolts);

        pubDesiredSpeed.accept(desiredState.speedMetersPerSecond);
        pubDesiredAngle.accept(desiredState.angle.getDegrees());
        pubSpeedError.accept(desiredState.speedMetersPerSecond - getDriveVelocity());
        pubAngleError.accept(desiredState.angle.getDegrees() - getAngle().getDegrees());
    }

    /**
     * Returns the angle of the swerve module.
     *
     * @return the angle of the swerve module
     */
    private Rotation2d getAngle() {
        return new Rotation2d(m_steerMotorSim.getAngularPositionRad()).plus(m_steerStartingAngle);
    }

    /**
     * Returns the position of the drive motor in meters.
     *
     * @return The position of the drive motor in meters.
     */
    private double getDrivePosition() {
        return m_driveMotorSim.getAngularPositionRad() * SwerveConfig.wheelRadius;
    }

    /**
     * Returns the velocity of the drive motor in meters per second.
     *
     * @return The drive motor velocity in meters per second.
     */
    private double getDriveVelocity() {
        return m_driveMotorSim.getAngularVelocityRadPerSec() * SwerveConfig.wheelRadius;
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
        return m_steerMotorSim.getAngularVelocityRadPerSec();
    }

    /**
     * Checks if the swerve module is synchronized between the NEO encoder and cancoder.
     *
     * @return true if the module is synchronized, false otherwise
     */
    @Override
    public boolean isModuleSynced() {
        return false;
    }

    /**
     * Sets the idle mode for the drive and steer motors.
     *
     * @param drive The idle mode for the drive motor.
     * @param steer The idle mode for the steer motor.
     */
    @Override
    public void setIdleMode(IdleMode drive, IdleMode steer) {
        m_driveBrakeMode = drive == IdleMode.kBrake;
        m_steerBrakeMode = steer == IdleMode.kBrake;
    }

    /**
     * Stops the drive and steer motors of the swerve module.
     */
    @Override
    public void stopMotors() {
        m_driveMotorSim.setInputVoltage(0);
        m_steerMotorSim.setInputVoltage(0);
    }
}
