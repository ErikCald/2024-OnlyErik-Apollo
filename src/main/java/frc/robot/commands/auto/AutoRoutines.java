package frc.robot.commands.auto;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.IntakeControl;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoRoutines extends SubsystemBase {
    PathPlannerAuto demoAuto;

    public AutoRoutines() {
        registerCommandsToPathplanner();

        demoAuto = new PathPlannerAuto("TestingAuto");
    }

    public void registerCommandsToPathplanner() {
        NamedCommands.registerCommand("IntakeControlFalse", new IntakeControl(false));
    }

    public Command getAutonomousCommand(int autoIndex) {
        switch (autoIndex) {
            default:
            case 0:
                return new InstantCommand();

            case 1:
                return demoAuto;

            case 2:
                return Commands.run(
                        () ->
                                SwerveSubsystem.getInstance()
                                        .setModuleStates(
                                                new SwerveModuleState[] {
                                                    new SwerveModuleState(
                                                            1, Rotation2d.fromDegrees(0)),
                                                    new SwerveModuleState(
                                                            1, Rotation2d.fromDegrees(0)),
                                                    new SwerveModuleState(
                                                            1, Rotation2d.fromDegrees(0)),
                                                    new SwerveModuleState(
                                                            1, Rotation2d.fromDegrees(0)),
                                                },
                                                true));
        }
    }
}
