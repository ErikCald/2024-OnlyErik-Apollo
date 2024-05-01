// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.lib.lib2706.button.FakeCommandXboxController;
import frc.lib.lib2706.button.FakeCommandXboxController.FakeControllerType;
import frc.lib.lib2706.networktables.TunableDouble;
import frc.robot.Config.ArmConfig.ArmSetpoints;
import frc.robot.Config.NTConfig;
import frc.robot.Config.SwerveConfig.TeleopSpeeds;
import frc.robot.Robot;
import frc.robot.commands.BlingCommand;
import frc.robot.commands.BlingCommand.BlingColour;
import frc.robot.commands.ClimberRPM;
import frc.robot.commands.CombinedCommands;
import frc.robot.commands.MakeIntakeMotorSpin;
import frc.robot.commands.RotateToAngle;
import frc.robot.commands.RumbleJoystick;
import frc.robot.commands.SetArm;
import frc.robot.commands.SubwooferShot;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.commands.auto.AutoSelector;
import frc.robot.subsystems.mechanisms.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.ShooterSubsystem;
import frc.robot.subsystems.misc.CreateShuffleboardLayout;
import frc.robot.subsystems.misc.SparkMaxManagerSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

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
    //     private final CommandXboxController driver = new CommandXboxController(0);
    // private final CommandXboxController operator = new CommandXboxController(1);
    // private final CommandXboxController testJoystick = new CommandXboxController(2);

    private final CommandXboxController driver =
            new FakeCommandXboxController(0, FakeControllerType.EliminatorAfterShock);
    private final CommandXboxController operator =
            new FakeCommandXboxController(1, FakeControllerType.EliminatorAfterShock);
    private final CommandXboxController testJoystick =
            new FakeCommandXboxController(2, FakeControllerType.EliminatorAfterShock);

    /* Create Subsystems in a specific order */
    private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();

    /* Auto */
    private AutoRoutines m_autoRoutines;
    private AutoSelector m_autoSelector;

    private TunableDouble shooterTargetRPM =
            new TunableDouble("Target RPM", NTConfig.shooterTable, 0);
    private TunableDouble shooterDesiredVoltage =
            new TunableDouble("desired Voltage", NTConfig.shooterTable, 0);
    private TunableDouble armAngleDeg =
            new TunableDouble("ArmTuning/setAngleDeg", NTConfig.armTable, 5.0);

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
                .onTrue(CombinedCommands.strobeToSolidBlingCommand())
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
         * KingstonV1: https://drive.google.com/file/d/1gDgxnz-agWGoYmTTRfViVPwR7O2H80mh
         */
        // Core Swerve Buttons
        driver.back().onTrue(SwerveSubsystem.getInstance().setHeadingCommand(new Rotation2d(0)));
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

        /**
         * Operator Controls
         * KingstonV1: https://drive.google.com/file/d/18HyIpIeW08CC6r6u-Z74xBWRv9opHnoZ
         */
        // Arm
        operator.y().onTrue(new SetArm(() -> ArmSetpoints.AMP.getDegrees())); // Amp
        operator.b().onTrue(new SetArm(() -> ArmSetpoints.IDLE.getDegrees())); // Idle
        operator.a().onTrue(new SetArm(() -> ArmSetpoints.NO_INTAKE.getDegrees())); // Pickup
        operator.x().onTrue(new SetArm(() -> ArmSetpoints.SPEAKER_KICKBOT_SHOT.getDegrees()));
        // Climber
        operator.leftTrigger(0.10)
                .and(operator.back())
                .whileTrue(
                        new ClimberRPM(
                                () ->
                                        MathUtil.applyDeadband(operator.getLeftTriggerAxis(), 0.35)
                                                * 0.5));

        // Eject the note from the front with start
        operator.start()
                .whileTrue(Commands.run(() -> intake.setVoltage(-12), intake))
                .onFalse(Commands.runOnce(() -> intake.stopMotors()));

        // operator.leftTrigger(0.3).whileTrue(
        operator.leftBumper()
                .whileTrue(CombinedCommands.armIntake())
                .onFalse(new SetArm(() -> ArmSetpoints.NO_INTAKE.getDegrees()))
                .onFalse(
                        new MakeIntakeMotorSpin(9.0, 0)
                                .withTimeout(1)
                                .until(() -> intake.shooterSideSwitch()));

        // NOTE: right Trigger has been assigned to climber
        operator.rightTrigger(0.3).whileTrue(CombinedCommands.simpleShootNoteAmp());

        operator.rightBumper()
                .onTrue(
                        new SubwooferShot(
                                operator.rightBumper(),
                                ArmSetpoints.SPEAKER_KICKBOT_SHOT.getDegrees(),
                                2820,
                                2700));

        /**
         * Testing button bindings
         */
        // SwerveModuleState[] moduleStatesForwards = {
        //   new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
        //   new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
        //   new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
        //   new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
        // };
        // testJoystick.a().whileTrue(Commands.run(
        //   () -> SwerveSubsystem.getInstance().setModuleStates(moduleStatesForwards, true, true)
        // ));

        // SwerveModuleState[] moduleStatesSideways = {
        //   new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
        //   new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
        //   new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
        //   new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
        // };
        // testJoystick.a().whileTrue(Commands.run(
        //   () -> SwerveSubsystem.getInstance().setModuleStates(moduleStatesSideways, true, true)
        // ));
        // Let testJoystick control swerve. Note disables driver joystick swerve. Never commit this
        // line.
        // s_Swerve.setDefaultCommand(new TeleopSwerve(testJoystick));
        // testJoystick.back().onTrue(SwerveSubsystem.getInstance().setHeadingCommand(new
        // Rotation2d()));
        // testJoystick.b().onTrue(SwerveSubsystem.getInstance().setOdometryCommand(new
        // Pose2d(3,3,new Rotation2d(0))));
        // testJoystick.a().whileTrue(PhotonSubsystem.getInstance().getAprilTagCommand(PhotonPositions.FAR_SPEAKER_RED, driver))
        //           .onFalse(Commands.runOnce(()->{},SwerveSubsystem.getInstance()));

        // testJoystick.x() //Drives the note into the shooter
        //   .whileTrue(Commands.runOnce(()-> intake.setMode(shooter.isReadyToShoot() ?
        // IntakeModes.SHOOT : IntakeModes.STOP_INTAKE)))
        //   .whileFalse(Commands.runOnce(()->intake.setMode(IntakeModes.STOP_INTAKE)));

        // testJoystick.leftBumper().whileTrue(
        //       new MakeIntakeMotorSpin(9.0,0));

        // testJoystick.start().onTrue( new SetArm(armAngleDeg));
        // testJoystick.back().whileTrue(new Shooter_PID_Tuner(shooterTargetRPM));
        // testJoystick.rightBumper().whileTrue(CombinedCommands.simpleShootNoteSpeaker(1, () ->
        // shooterTargetRPM.getAsDouble(), 50));

        // testJoystick.rightTrigger().whileTrue(new
        // Shooter_PID_Tuner(()->shooterTargetRPM.getAsDouble()));

        // testJoystick.a().onTrue(PhotonSubsystem.getInstance().getResetCommand(4));
        // testJoystick.b().onTrue(SwerveSubsystem.getInstance().setOdometryCommand(new Pose2d(3, 3,
        // Rotation2d.fromDegrees(0))));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *leop
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        int autoId = m_autoSelector.getAutoId();
        System.out.println("*********************** Auto Id" + autoId);

        return m_autoRoutines.getAutonomousCommand(autoId);
    }
}
