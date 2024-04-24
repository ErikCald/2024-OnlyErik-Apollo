package frc.robot.commands.auto;

import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeControl;
import frc.robot.commands.characterization.StaticCharacterizationCommand;
import frc.robot.commands.characterization.VelocityCharacterizationCommand;
import frc.robot.commands.characterization.WheelDiameterCharacterizationCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoRoutines extends SubsystemBase {
    private SendableChooser<Command> m_autoChooser;

    public AutoRoutines() {
        registerCommandsToPathplanner();

        m_autoChooser = AutoBuilder.buildAutoChooser();

        addToChooser(m_autoChooser);

        SmartDashboard.putData("Auto Chooser", m_autoChooser);
    }

    public void registerCommandsToPathplanner() {
        NamedCommands.registerCommand("IntakeControlFalse", new IntakeControl(false));
    }

    public void addToChooser(SendableChooser<Command> chooser) {
        /* Path find testing */
        ArrayList<Pose2d> poses = new ArrayList<>();
        // poses.add(new Pose2d(1, 1, new Rotation2d()));
        // poses.add(new Pose2d(13, 5, Rotation2d.fromDegrees(180)));

        poses.add(new Pose2d(1.93, 7.78, new Rotation2d(1.58)));
        poses.add(new Pose2d(15.58, 1.32, new Rotation2d(-1.127)));
        poses.add(new Pose2d(0.661, 4.391, new Rotation2d(2.09)));
        poses.add(new Pose2d(15.973, 3.170, new Rotation2d(0)));
        poses.add(new Pose2d(0.77, 6.83, new Rotation2d(-2.185)));

        chooser.addOption("TestPathFinding", createTestPathFinding(poses));

    }
    public static Command commands;
    public Command createTestPathFinding(ArrayList<Pose2d> poses) {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
            3.0, 4.0,
            Math.toRadians(540), Math.toRadians(720));

        commands = new InstantCommand();
        for (Pose2d pose : poses) {
            commands = Commands.sequence(
                commands,
                new WaitCommand(3),
                AutoBuilder.pathfindToPose(
                    pose,
                    constraints,
                    0.0, // Goal end velocity in meters/sec
                    0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
            ));
        }
        return new RunCommand(() -> {
            if (!commands.isScheduled()) {
                commands.schedule();
            }
        });
    }

    public Command getAutonomousCommand(int autoIndex) {
        switch (autoIndex) {
                /** Do nothing command, default for safety */
            default:
            case 0:
                return new InstantCommand();

            case 1:
                return m_autoChooser.getSelected();

                /** Auto Testing commands */
            case 2:
                SwerveModuleState state = new SwerveModuleState(1, Rotation2d.fromDegrees(0));
                SwerveModuleState[] states = new SwerveModuleState[] {state, state, state, state};
                return Commands.run(
                        () -> SwerveSubsystem.getInstance().setModuleStates(states, true));


                /** Characterization commands */
            case 20:
                return new WheelDiameterCharacterizationCommand(false);
            case 21:
                return new WheelDiameterCharacterizationCommand(true);
            case 22:
                return StaticCharacterizationCommand.createSwerve();
            case 23:
                return VelocityCharacterizationCommand.createSwerve();
        }
    }
}
