// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public interface SwerveModuleInterface {
    /**
     * Resets the steer encoder position to the absolute position of the CanCoder.
     */
    public void resetToAbsolute();

    /**
     * Sets the desired state of the SwerveModule.
     *
     * @param desiredState The desired state of the SwerveModule.
     * @param isOpenLoop   A boolean indicating whether the control is open loop or not.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop);

    /**
     * Sets the feedforward value for the swerve module.
     *
     * @param newFeedforward the new feedforward value to be set
     */
    public void setFeedforward(SimpleMotorFeedforward newFeedforward);

    /**
     * Gets the current state of the swerve module.
     *
     * @return The SwerveModuleState object representing the current state.
     */
    public SwerveModuleState getState();

    /**
     * Returns the position of the swerve module.
     *
     * @return the position of the swerve module
     */
    public SwerveModulePosition getPosition();

    /**
     * Returns the velocity of the steering encoder.
     *
     * @return the velocity in radians per second
     */
    public double getSteeringVelocity();

    /**
     * Updates the state of the SwerveModule periodically.
     * This method is called repeatedly to update the measured speed, angle, and Cancoder angle of the SwerveModule.
     */
    public void periodic();

    /**
     * Checks if the swerve module is synchronized between the NEO encoder and cancoder.
     *
     * @return true if the module is synchronized, false otherwise
     */
    public boolean isModuleSynced();

    /**
     * Sets the idle mode for the drive and steer motors.
     *
     * @param drive The idle mode for the drive motor.
     * @param steer The idle mode for the steer motor.
     */
    public void setIdleMode(IdleMode drive, IdleMode steer);

    /**
     * Stops the drive and steer motors of the swerve module.
     */
    public void stopMotors();
}
