// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.lib.lib2706.networktables.TunableString;
import frc.lib.lib6328.VirtualSubsystem;
import frc.robot.Config.CANID;
import frc.robot.Config.NTConfig;
import frc.robot.Config.RobotID;
import frc.robot.robotcontainers.ApolloContainer;
import frc.robot.robotcontainers.BeetleContainer;
import frc.robot.robotcontainers.PoseidonContainer;
import frc.robot.robotcontainers.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.swerve.PathPlannerSubsystem;

import org.littletonrobotics.urcl.URCL;
import org.photonvision.PhotonCamera;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    private TunableString tunableLogName = new TunableString("LogName", NTConfig.logTable, "");

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Record both DS control and joystick data
        DriverStation.startDataLog(DataLogManager.getLog());

        // Start the URCL (Unofficial REV-Compatible Logger) by FRC Team 6328, Mechanical Advantage.
        // Logs all CAN messages sent from REV devices.
        URCL.start(CANID.mapCanIdsToNames());

        // Disable PhotonVision version check in simulation
        PhotonCamera.setVersionCheckEnabled(!Robot.isSimulation());

        // Silence joystick warnings if in simulation
        DriverStation.silenceJoystickConnectionWarning(Robot.isSimulation());
        PathPlannerSubsystem.getInstance();
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        createRobotContainer();
    }

    private void createRobotContainer() {
        // Instantiate the RobotContainer based on the Robot ID.  This will perform all our button
        // bindings, and put our
        // autonomous chooser on the dashboard.

        switch (RobotID.getActiveID()) {
            case APOLLO:
                m_robotContainer = new ApolloContainer();
                break; // competition

            case SIMULATION:
                m_robotContainer = new ApolloContainer();
                break; // simulation

            case BEETLE:
                m_robotContainer = new BeetleContainer();
                break; // beetle

            case POSEIDON:
                m_robotContainer = new PoseidonContainer();
                break; // poseidon

            default:
                m_robotContainer = new ApolloContainer();
                DriverStation.reportError(
                        "ISSUE WITH CONSTRUCTING THE ROBOT CONTAINER. \n"
                                + "NewRobotContainer constructed by default. RobotID: "
                                + RobotID.getActiveID().name(),
                        true);
        }
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        VirtualSubsystem.periodicAll();
        CommandScheduler.getInstance().run();

        // Update the log name if the TunableString has changed
        TunableString.ifChanged(
                hashCode(),
                () -> {
                    System.out.println("LOG NAME HAS CHANGED TO: " + tunableLogName.get());
                    DataLogManager.stop();
                    DataLogManager.start(DataLogManager.getLogDir(), tunableLogName.get());
                },
                tunableLogName);
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link PoseidonContainer} class. */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        ArmSubsystem.getInstance().resetProfiledPIDController();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        ArmSubsystem.getInstance().resetProfiledPIDController();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}
}
