// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static frc.lib.lib2706.NTUtil.doublePubFast;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;

import frc.lib.lib2706.PIDConfig;
import frc.lib.lib2706.TunableDouble;
import frc.lib.lib2706.TunablePIDConfig;
import frc.lib.lib2706.TunableSimpleFeedforward;
import frc.lib.lib2706.swerve.SwerveModuleConstants;
import frc.robot.Config.NTConfig;
import frc.robot.Config.SwerveConfig;

import java.util.ArrayList;

/**
 * The abstract class representing a swerve module.
 * This class provides common functionality and defines abstract methods that must be implemented by concrete swerve module classes.
 */
public abstract class SwerveModuleAbstract {
    private static ArrayList<SwerveModuleAbstract> s_modules = new ArrayList<>();
    protected static boolean s_hasSetup = false;
    protected static NetworkTable s_driveTable, s_steerTable;
    protected static SimpleMotorFeedforward s_driveFF;
    private static TunableSimpleFeedforward s_tunableDriveFF;
    private static TunablePIDConfig s_tunableDrivePID;
    private static TunablePIDConfig s_tunableSteerPID;

    protected final String m_name;
    protected NetworkTable moduleTable;
    protected DoublePublisher pubMeasuredSpeed, pubDesiredSpeed, pubSpeedError;
    protected DoublePublisher pubMeasuredAngle, pubDesiredAngle, pubAngleError;
    protected DoublePublisher pubCancoderAngle;

    protected TunableDouble tunableSteerOffset;

    /**
     * This class represents an abstract base class for a swerve module.
     * It provides common functionality and properties for swerve modules.
     */
    protected SwerveModuleAbstract(SwerveModuleConstants constants, String name) {
        /* Register modules to update pid controllers */
        s_modules.add(this);

        /* Single Time Setup */
        moduleSingleTimeSetup();

        /* Networktable Setup */
        m_name = name;
        moduleTable = NTConfig.swerveTable.getSubTable("SwerveModule" + name);

        pubMeasuredSpeed = doublePubFast(moduleTable, "Measured Speed (mps)");
        pubDesiredSpeed = doublePubFast(moduleTable, "Desired Speed (mps)");
        pubSpeedError = doublePubFast(moduleTable, " Speed Error (mps)");

        pubMeasuredAngle = doublePubFast(moduleTable, "Measured Angle (deg)");
        pubDesiredAngle = doublePubFast(moduleTable, "Desired Angle (deg)");
        pubAngleError = doublePubFast(moduleTable, "Angle Error (deg)");

        pubCancoderAngle = doublePubFast(moduleTable, "Cancoder Angle (deg)");

        tunableSteerOffset =
                new TunableDouble(
                        "Angle Offset (deg)", moduleTable, constants.steerOffset.getDegrees());
    }

    /**
     * Performs the one-time setup for the swerve module.
     * This method initializes the necessary network tables, feedforwards, and PID configurations.
     * It is called only once during the initialization of the swerve modules.
     */
    private static void moduleSingleTimeSetup() {
        if (s_hasSetup) {
            return;
        } else {
            s_hasSetup = true;
        }

        /* Setup NT Tables */
        s_driveTable = NTConfig.swerveTable.getSubTable("Drive");
        s_steerTable = NTConfig.swerveTable.getSubTable("Steer");

        /* Setup tunable SimpleMotorFeedforwards */
        s_tunableDriveFF =
                new TunableSimpleFeedforward(
                        (ff) -> s_driveFF = ff,
                        s_driveTable,
                        SwerveConfig.driveKS,
                        SwerveConfig.driveKV,
                        SwerveConfig.driveKA);

        s_tunableDrivePID =
                new TunablePIDConfig(
                        (pid) -> updateAllModulesDrivePID(pid),
                        s_driveTable,
                        SwerveConfig.driveKFF,
                        SwerveConfig.driveKP,
                        SwerveConfig.driveKI,
                        SwerveConfig.driveKD,
                        SwerveConfig.driveIZone,
                        0);

        s_tunableSteerPID =
                new TunablePIDConfig(
                        (pid) -> updateAllModulesSteerPID(pid),
                        s_steerTable,
                        0.0, // No FF for a position PID
                        SwerveConfig.steerKP,
                        SwerveConfig.steerKI,
                        SwerveConfig.steerKD,
                        SwerveConfig.steerIZone,
                        0);
    }

    /**
     * Updates the tunable module constants for the swerve module.
     * This method checks for updates in the tunable drive feedforward, drive PID, and steer PID constants.
     */
    public static void updateTunableModuleConstants() {
        s_tunableDriveFF.checkForUpdates();
        s_tunableDrivePID.checkForUpdates();
        s_tunableSteerPID.checkForUpdates();
    }

    /**
     * Updates the drive PID configuration for all swerve modules.
     *
     * @param pid The PID configuration to be updated.
     */
    private static void updateAllModulesDrivePID(PIDConfig pid) {
        for (SwerveModuleAbstract module : s_modules) {
            module.updateDrivePID(pid);
        }
    }

    /**
     * Updates the steer PID configuration for all swerve modules.
     *
     * @param pid The PID configuration to be updated.
     */
    private static void updateAllModulesSteerPID(PIDConfig pid) {
        for (SwerveModuleAbstract module : s_modules) {
            module.updateSteerPID(pid);
        }
    }

    /**
     * Updates the drive PID configuration for the swerve module.
     *
     * @param pid The PID configuration to update.
     */
    protected abstract void updateDrivePID(PIDConfig pid);

    /**
     * Updates the PID configuration for steering control.
     *
     * @param pid The PID configuration to update.
     */
    protected abstract void updateSteerPID(PIDConfig pid);

    /**
     * Resets the steer encoder position to the absolute position of the CanCoder.
     */
    public abstract void resetToAbsolute();

    /**
     * Sets the desired state of the SwerveModule.
     *
     * @param desiredState The desired state of the SwerveModule.
     * @param isOpenLoop   A boolean indicating whether the control is open loop or not.
     */
    public abstract void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop);

    /**
     * Gets the current state of the swerve module.
     *
     * @return The SwerveModuleState object representing the current state.
     */
    public abstract SwerveModuleState getState();

    /**
     * Returns the position of the swerve module.
     *
     * @return the position of the swerve module
     */
    public abstract SwerveModulePosition getPosition();

    /**
     * Returns the velocity of the steering encoder.
     *
     * @return the velocity in radians per second
     */
    public abstract double getSteeringVelocity();

    /**
     * Updates the state of the SwerveModule periodically.
     * This method is called repeatedly to update the measured speed, angle, and Cancoder angle of the SwerveModule.
     */
    public abstract void periodic();

    /**
     * Checks if the swerve module is synchronized between the NEO encoder and cancoder.
     *
     * @return true if the module is synchronized, false otherwise
     */
    public abstract boolean isModuleSynced();

    /**
     * Sets the idle mode for the drive and steer motors.
     *
     * @param drive The idle mode for the drive motor.
     * @param steer The idle mode for the steer motor.
     */
    public abstract void setIdleMode(IdleMode drive, IdleMode steer);

    /**
     * Stops the drive and steer motors of the swerve module.
     */
    public abstract void stopMotors();
}
