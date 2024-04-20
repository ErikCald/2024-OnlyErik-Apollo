// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.misc;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.lib.lib6328.Alert;
import frc.robot.Config.NTConfig;

/**
 * This class is responsible for creating the shuffleboard layout for basic debugging.
 */
public class CreateShuffleboardLayout {
    private static boolean isCreated = false;

    /**
     * Creates the Shuffleboard layout if it has not been created already.
     */
    public static void create() {
        if (!isCreated) {
            new CreateShuffleboardLayout();
            isCreated = true;
        }
    }

    /**
     * Constructs the layout for shuffleboard.
     */
    private CreateShuffleboardLayout() {
        createHardwareDebuggingTab();
        createSoftwareDebuggingTab();
    }

    /**
     * Creates the basic debugging tab on the shuffleboard and adds various components to it.
     */
    private void createHardwareDebuggingTab() {
        ShuffleboardTab hardwareDebuggingTab = Shuffleboard.getTab("HardwareDebugging");

        hardwareDebuggingTab
                .add("SwerveSparkMaxStatus", Alert.getGroup(NTConfig.swerveSparkmaxAlertGroup))
                .withPosition(4, 0)
                .withSize(6, 2);

        hardwareDebuggingTab
                .add("CancoderStatus", Alert.getGroup(NTConfig.cancoderAlertGroup))
                .withPosition(0, 0)
                .withSize(4, 3);

        hardwareDebuggingTab
                .add(
                        "NonSwerveSparkMaxStatus",
                        Alert.getGroup(NTConfig.nonSwerveSparkMaxAlertGroup))
                .withPosition(4, 2)
                .withSize(6, 3);
    }

    /**
     * Creates the software debugging tab in the Shuffleboard.
     */
    private void createSoftwareDebuggingTab() {
        ShuffleboardTab softwareDebuggingTab = Shuffleboard.getTab("SoftwareDebugging");
        // Display the CommandScheduler to show what commands are currently scheduled
        softwareDebuggingTab
                .add("CommandScheduler", CommandScheduler.getInstance())
                .withPosition(3, 0)
                .withSize(3, 6);
    }
}
