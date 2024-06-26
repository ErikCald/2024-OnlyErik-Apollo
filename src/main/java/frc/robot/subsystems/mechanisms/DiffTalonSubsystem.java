// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.lib2706.SubsystemChecker;
import frc.lib.lib2706.SubsystemChecker.SubsystemType;
import frc.robot.Config;
import frc.robot.Config.RobotID;

public class DiffTalonSubsystem extends SubsystemBase {
    /**
     * Instance Variables
     */
    private WPI_TalonSRX leftLeader, rightLeader;

    private BaseMotorController leftFollower, rightFollower;

    private DifferentialDrive diffDrive;

    private PigeonIMU pigeon;

    private static DiffTalonSubsystem instance;

    public static DiffTalonSubsystem getInstance() {
        if (instance == null) {
            if (Config.DIFF.ISNEOS) {
                DriverStation.reportError(
                        "DiffTalonSubsystem.getInstance() was called even though Config.DIFF_ISNEOS"
                                + " is true. RobotID: "
                                + RobotID.getActiveID().name(),
                        true);
            } else {
                SubsystemChecker.subsystemConstructed(SubsystemType.DiffTalonSubsystem);
                instance = new DiffTalonSubsystem();
            }
        }

        return instance;
    }

    /** Creates a new ExampleSubsystem. */
    public DiffTalonSubsystem() {
        leftLeader = new WPI_TalonSRX(Config.DIFF.DIFF_LEADER_LEFT);
        rightLeader = new WPI_TalonSRX(Config.DIFF.DIFF_LEADER_RIGHT);

        // Check whether to construct a victor or a talon or nothing
        if (Config.DIFF.HAS_FOLLOWERS == true) {
            if (Config.DIFF.LEFT_FOLLOWER_ISVICTOR) {
                leftFollower = new WPI_VictorSPX(Config.DIFF.DIFF_FOLLOWER_LEFT);
            } else {
                leftFollower = new WPI_TalonSRX(Config.DIFF.DIFF_FOLLOWER_LEFT);
            }
            if (Config.DIFF.RIGHT_FOLLOWER_ISVICTOR) {
                rightFollower = new WPI_VictorSPX(Config.DIFF.DIFF_FOLLOWER_RIGHT);
            } else {
                rightFollower = new WPI_TalonSRX(Config.DIFF.DIFF_FOLLOWER_RIGHT);
            }
        } else {
            leftFollower = null;
            rightFollower = null;
        }

        leftLeader.configFactoryDefault(Config.CAN_TIMEOUT_LONG);
        rightLeader.configFactoryDefault(Config.CAN_TIMEOUT_LONG);

        leftLeader.setInverted(Config.DIFF.LEADER_LEFT_INVERTED);
        rightLeader.setInverted(Config.DIFF.LEADER_RIGHT_INVERTED);

        if (leftFollower != null && rightFollower != null) {
            leftFollower.configFactoryDefault();
            rightFollower.configFactoryDefault();

            leftFollower.setInverted(
                    Config.DIFF.FOLLOWER_LEFT_INVERTED
                            ? InvertType.OpposeMaster
                            : InvertType.FollowMaster);

            rightFollower.setInverted(
                    Config.DIFF.FOLLOWER_RIGHT_INVERTED
                            ? InvertType.OpposeMaster
                            : InvertType.FollowMaster);
        }

        if (Config.CANID.PIGEON.val() != -1) {
            if (Config.CANID.PIGEON.val() == Config.DIFF.DIFF_FOLLOWER_LEFT && leftFollower != null)
                pigeon = new PigeonIMU((WPI_TalonSRX) leftFollower);
            else {
                pigeon = new PigeonIMU(Config.CANID.PIGEON.val());
            }
        }

        diffDrive = new DifferentialDrive(leftLeader, rightLeader);
    }

    public void stopMotors() {
        leftLeader.stopMotor();
        rightLeader.stopMotor();

        if (leftFollower != null) {
            leftFollower.neutralOutput();
        }
        if (rightFollower != null) {
            rightFollower.neutralOutput();
        }
    }

    /**
     * Set the {@link NeutralMode} of the motors.
     *
     * @param mode Desired NeutralMode.
     */
    public void setNeutralMode(NeutralMode mode) {
        leftLeader.setNeutralMode(mode);
        rightLeader.setNeutralMode(mode);
        if (leftFollower != null) {
            leftFollower.setNeutralMode(mode);
        }
        if (rightFollower != null) {
            rightFollower.setNeutralMode(mode);
        }
    }

    /**
     * Motor control method for arcade drive.
     *
     * @param forwardVal The forward value
     * @param rotateVal The rotate value
     */
    public void arcadeDrive(double forwardVal, double rotateVal) {
        diffDrive.arcadeDrive(forwardVal, rotateVal, false);
    }
}
