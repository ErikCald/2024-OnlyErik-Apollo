package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.Config;
import frc.robot.Config.ArmConfig.ArmSetpoints;
import frc.robot.commands.BlingCommand.BlingColour;
import frc.robot.subsystems.mechanisms.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import java.util.function.DoubleSupplier;

public class CombinedCommands {
    /**
     * This file should not be constructed. It should only have static factory methods.
     */
    private CombinedCommands() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Spin up the shooter while doing the following,
     * backing up note, waiting a bit, then feeding the note.
     */
    public static Command simpleShootNoteSpeaker(double intakeTimeout) {
        return (simpleShootNoteSpeaker(
                intakeTimeout, () -> Config.ShooterConfig.subwooferRPM, 100));
    }

    /**
     * Spin up the shooter while doing the following,
     * backing up note, waiting a bit, then feeding the note.
     */
    public static Command simpleShootNoteSpeaker(
            double intakeTimeout, DoubleSupplier RPM, double clearance) {
        return Commands.deadline(
                Commands.sequence(
                        new IntakeControl(false).withTimeout(0.15),
                        new WaitUntilCommand(
                                () ->
                                        ShooterSubsystem.getInstance().getTopVelRPM()
                                                > RPM.getAsDouble()),
                        new IntakeControl(true).withTimeout(intakeTimeout)),
                new Shooter_PID_Tuner(() -> (RPM.getAsDouble() + clearance)));
    }

    public static Command simpleShootNoteAmp() {
        return Commands.deadline(
                Commands.sequence(
                        new IntakeControl(false).withTimeout(0.3),
                        new WaitCommand(0.5),
                        new IntakeControl(true).withTimeout(0.6)),
                new Shooter_Voltage(() -> 6));
    }

    /**
     * Runs the given command. If the given command ends before the timeout, the command will end as normal.
     * If the timeout happens before the command ends, this command will forceful cancel itself and any
     * future command groups it's apart of.
     *
     * This means if the next command in a sequence is to shoot, it won't shoot unless the given command here
     * has correctly ended before the timeout.
     *
     * @param timeout in seconds
     * @param command to run
     * @return
     */
    public static Command forcefulTimeoutCommand(double timeoutSeconds, Command command) {
        // Create a command that has the same requirements as the given command.
        Command requirementCommand = new InstantCommand();
        for (Subsystem s : command.getRequirements()) {
            requirementCommand.addRequirements(s);
        }

        // If WaitCommand ends before the given command ends, schedule a command to forceful cancel
        // this
        // command group and any future commands this command group is apart of (like a command to
        // shoot)
        // Else if the given command ends before the timeout, continue on as normal.
        return Commands.race(
                new WaitCommand(timeoutSeconds).andThen(new ScheduleCommand(requirementCommand)),
                command);
    }

    // Intake and Arm Intake Position
    public static Command armIntake() {

        return Commands.parallel(
                new MakeIntakeMotorSpin(9.0, 0),
                new SetArm(
                        () ->
                                ArmSetpoints.INTAKE
                                        .getDegrees())); // Continue to hold arm in the correct
        // position
    }

    /**
     * Centers the note then spins up the shooter.
     *
     * The belts naturally center the note while the intake is spinning as long as the robot is not rotating very fast.
     * This command group will spin the intake rollers until the chassis has not rotated for 0.3 seconds then spinup the shooter.
     *
     * @param shooterSpeed The speed in RPM to set the shooter to.
     */
    public static Command centerNoteThenSpinUpShooer(double shooterSpeed) {
        // Intake and shooter sequence
        // Spin the intake forwards to center the note, when the chassis is not rotating for a bit,
        // lodge the centered note in the intake rollers
        Debouncer notRotatingDebouncer = new Debouncer(0.5);
        return Commands.sequence(
                Commands.runOnce(() -> notRotatingDebouncer.calculate(false)),
                new MakeIntakeMotorSpin(8.0, 0)
                        .until(
                                () ->
                                        notRotatingDebouncer.calculate(
                                                Math.abs(
                                                                SwerveSubsystem.getInstance()
                                                                        .getRobotRelativeSpeeds()
                                                                        .omegaRadiansPerSecond)
                                                        < Math.toRadians(3))),
                Commands.parallel(
                        new IntakeControl(false), // Reverse note until not touching shooter
                        new WaitCommand(0.2).andThen(new Shooter_PID_Tuner(() -> shooterSpeed))));
    }

    /*Bling command to indicate that a note is loaded in intake*/
    public static Command strobeToSolidBlingCommand() {
        return Commands.sequence(
                new BlingCommand(BlingColour.PURPLESTROBE),
                new WaitCommand(2),
                new BlingCommand(BlingColour.PURPLE));
    }
}
