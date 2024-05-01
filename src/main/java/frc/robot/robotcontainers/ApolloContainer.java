// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.lib.lib2706.button.FakeCommandXboxController;
import frc.lib.lib2706.button.FakeCommandXboxController.FakeControllerType;
import frc.lib.lib2706.networktables.TunableDouble;
import frc.robot.Config.ArmConfig.ArmSetpoint;
import frc.robot.Config.NTConfig;
import frc.robot.Config.ShooterConfig.ShooterSetpoint;
import frc.robot.Config.SwerveConfig.TeleopSpeeds;
import frc.robot.Robot;
import frc.robot.commands.BlingCommand;
import frc.robot.commands.BlingCommand.BlingColour;
import frc.robot.commands.RotateToAngle;
import frc.robot.commands.RumbleJoystick;
import frc.robot.commands.TeleopShot;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.commands.auto.AutoSelector;
import frc.robot.subsystems.mechanisms.ArmSubsystem;
import frc.robot.subsystems.mechanisms.ClimberSubsystem;
import frc.robot.subsystems.mechanisms.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.ShooterSubsystem;
import frc.robot.subsystems.misc.CreateShuffleboardLayout;
import frc.robot.subsystems.misc.SparkMaxManagerSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.ApriltagSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic
 * should actually be handled in the {@link Robot} periodic methods
 * (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class ApolloContainer extends RobotContainer {
    /* Controllers */
        private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final CommandXboxController testJoystick = new CommandXboxController(2);

//     private final CommandXboxController driver =
//             new FakeCommandXboxController(0, FakeControllerType.EliminatorAfterShock);
//     private final CommandXboxController operator =
//             new FakeCommandXboxController(1, FakeControllerType.EliminatorAfterShock);
//     private final CommandXboxController testJoystick =
//             new FakeCommandXboxController(2, FakeControllerType.EliminatorAfterShock);

    /* Create Subsystems in a specific order */
    private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    private final ArmSubsystem arm = ArmSubsystem.getInstance();
//     private final ClimberSubsystem climber = ClimberSubsystem.getInstance();

    /* Auto */
    private AutoRoutines m_autoRoutines;
    private AutoSelector m_autoSelector;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public ApolloContainer() {
        /*  Setup default commands */
        swerve.setDefaultCommand(new TeleopSwerve(driver));
        shooter.setDefaultCommand(shooter.slowdownCommand());

        configureButtonBindings();

        // Setup auto
        m_autoRoutines = new AutoRoutines();
        m_autoSelector = new AutoSelector();

        // Setup shuffleboard
        CreateShuffleboardLayout.create();

        // Call burn flash on all SparkMaxs
        SparkMaxManagerSubsystem.getInstance().burnFlashOnAllSparkmaxes();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the {@link CommandXboxController} or other ways.
     */
    private void configureButtonBindings() {
        // Set bling to purple when note is in

        new Trigger(() -> intake.shooterSideSwitch())
                .onTrue(
                        Commands.sequence(
                                new BlingCommand(BlingColour.PURPLESTROBE),
                                new WaitCommand(2),
                                new BlingCommand(BlingColour.PURPLE)))
                .onFalse(new BlingCommand(BlingColour.DISABLED));

        new Trigger(() -> intake.shooterSideSwitchLong() && DriverStation.isTeleopEnabled())
                .onTrue(
                        Commands.parallel(
                                new RumbleJoystick(
                                        driver, RumbleType.kBothRumble, 0.75, 0.4, false),
                                new RumbleJoystick(
                                        operator, RumbleType.kBothRumble, 0.75, 0.4, false)));

        /**
         * Driver Controls
         */
        // Core Swerve Buttons
        driver.back().onTrue(SwerveSubsystem.getInstance().setHeadingCommand(new Rotation2d(0)).andThen(Commands.runOnce(() -> System.out.println("REset odometry: " + swerve.getPose()))));
        driver.leftBumper()
                .onTrue(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.SLOW)))
                .onFalse(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.MAX)));

        driver.rightBumper()
                .onTrue(Commands.runOnce(() -> TeleopSwerve.setFieldRelative(false)))
                .onFalse(Commands.runOnce(() -> TeleopSwerve.setFieldRelative(true)));

        driver.start().onTrue(Commands.runOnce(() -> SwerveSubsystem.getInstance().synchSwerve()));

        // Commands that take control of the rotation stick
        driver.y().whileTrue(new RotateToAngle(driver, Rotation2d.fromDegrees(0)));
                driver.x().whileTrue(new RotateToAngle(driver, Rotation2d.fromDegrees(90)));
        driver.a().whileTrue(new RotateToAngle(driver, Rotation2d.fromDegrees(180)));
        driver.b().whileTrue(new RotateToAngle(driver, Rotation2d.fromDegrees(270)));

        // Vision command that only schedules when we have good apriltag data
        driver.rightTrigger()
                .and(() -> ApriltagSubsystem.getInstance().hasGoodData())
                .whileTrue(new InstantCommand());

        /**
         * Operator Controls
         */
        // Cancel any active shooter and intake commands
        operator.back()
                .onTrue(Commands.idle(shooter, intake).withTimeout(0.1))
                .onTrue(arm.holdArmCommand());

        // Arm
        operator.b().onTrue(arm.moveCommand(ArmSetpoint.SAFE_IDLE)); // Idle arm within bumpers
        operator.a().onTrue(arm.moveCommand(ArmSetpoint.STAGE_IDLE)); // Idle arm to go under stage

        // Climber
        // operator.leftTrigger(0.10)
        //         .and(operator.back())
        //         .whileTrue(climber.climberCommand(() -> operator.getLeftTriggerAxis()));

        // Eject the note through the intake
        operator.start().whileTrue(intake.ejectNoteCommand().alongWith(shooter.ejectNoteCommand()));

        // Intake a note when held
        operator.leftBumper()
                .onTrue(arm.moveCommand(ArmSetpoint.INTAKE)) // Lower arm when pressed
                .onFalse(arm.moveCommand(ArmSetpoint.STAGE_IDLE)) // Raise arm when released
                .whileTrue(intake.intakeNoteCommand()) // Intake the note when held
                .onFalse(intake.intakeNoteCommand().withTimeout(0.5)); // Extra 0.5 seconds intaking

        // Score in the amp
        operator.rightTrigger(0.3)
                .onTrue(
                        new TeleopShot(
                                () -> operator.rightTrigger(0.3).getAsBoolean(),
                                ArmSetpoint.AMP,
                                ShooterSetpoint.AMP));

        // Score in the speaker from the subwoofer
        operator.rightBumper()
                .onTrue(
                        new TeleopShot(
                                () -> operator.rightBumper().getAsBoolean(),
                                ArmSetpoint.SUBWOOFER_SHOT,
                                ShooterSetpoint.SUBWOOFER));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *leop
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        int autoId = m_autoSelector.getAutoId();
        DriverStation.reportWarning("~~~~Auto ID: " + autoId, false);

        return m_autoRoutines.getAutonomousCommand(autoId);
    }
}
