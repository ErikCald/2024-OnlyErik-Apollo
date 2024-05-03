// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.Config.FixedPosition;
import frc.robot.subsystems.mechanisms.ArmSubsystem;
import frc.robot.subsystems.mechanisms.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * This class represents a command group for teleoperated subwoofer shot.
 * It will spinup the shooter, move the arm and reverse the intake.
 * Then once the shooter, arm and intake are ready, and the button is pressed, it fires the note.
 */
public class FixedPositionShot extends SequentialCommandGroup {
    private final FixedPosition m_setpoint;

    /* Grab the Subsystems to improve code readability for this class */
    private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private final ArmSubsystem arm = ArmSubsystem.getInstance();

    /**
     * This class represents a command group for teleoperated subwoofer shot.
     * It will spinup the shooter, move the arm and reverse the intake.
     * Then once the shooter, arm and intake are ready, and the button is pressed, it fires the note.
     *
     * @param isButtonPressed A BooleanSupplier for the button that scheduled this command. Example: {@code }
     * @param isSubwooferNotAmp A boolean to use either AMP setpoints or Subwoofer shot setpoints.
     */
    public FixedPositionShot(FixedPosition setpoint) {
        m_setpoint = setpoint;

        /* These are the commands that need to run when preparing to shoot and while shooting. */
        Command runAlways =
                Commands.parallel(
                        shooter.spinCommand(m_setpoint.shooter),
                        arm.moveCommand(m_setpoint.arm),
                        swerve.getDriveToPoseCommand(m_setpoint.blueAlliancePose, true));

        /* IntakeSubsystem sequence */
        Command intakeSequence =
                Commands.sequence(
                        intake.reverseNoteCommand(),
                        new WaitUntilCommand(() -> shouldShoot()),
                        intake.feedNoteCommand());

        /* Commands are run in a Parallel deadline group, meaning once the intakeSequence is done, this commands ends*/
        addCommands(Commands.deadline(intakeSequence, runAlways));
    }

    /**
     * Determines whether the robot should shoot into the subwoofer.
     *
     * @return true to shoot, false to not shoot
     */
    public boolean shouldShoot() {
        return shooter.isAtSetpoint(m_setpoint.shooter)
                && arm.isAtSetpoint(m_setpoint.arm)
                && swerve.isAtPose(true);
    }
}
