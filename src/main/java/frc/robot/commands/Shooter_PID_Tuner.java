// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.ShooterSubsystem;

import java.util.function.DoubleSupplier;

public class Shooter_PID_Tuner extends Command {

    // variable for setpoint
    private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    private DoubleSupplier setPointInRPM;

    /** Creates a new Shooter_tuner. */
    public Shooter_PID_Tuner(DoubleSupplier setPointInRPM) {
        this.setPointInRPM = setPointInRPM;

        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.setRPM(setPointInRPM.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}