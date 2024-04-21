// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.characterization;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.lib.lib2706.networktables.TunableDouble;
import frc.robot.Config.NTConfig;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * This class represents a command for static characterization of a subsystem.
 * Static characterization involves measuring the velocity of the subsystem at different input voltages.
 * The command gradually increases the input voltage and measures the velocity until a minimum velocity is reached.
 *
 * The command uses a NetworkTable to store the characterization data and tunable parameters.
 * It requires a DoubleConsumer to accept the input voltage and a DoubleSupplier to provide the current velocity.
 *
 * The command is typically used with a SwerveSubsystem, but can be used with other subsystems as well.
 *
 * Example usage:
 * StaticCharacterizationCommand.createSwerve() - Creates a static characterization command for a SwerveSubsystem.
 */
public class StaticCharacterizationCommand extends Command {
    private static final NetworkTable table =
            NTConfig.characterizingTable.getSubTable("StaticCharacterization");
    private static final TunableDouble currentRampFactor =
            new TunableDouble("VoltageRampPerSec", table, 0.1);
    private static final TunableDouble minVelocity =
            new TunableDouble("MinStaticVelocity", table, 0.1);

    private final DoubleConsumer inputConsumer;
    private final DoubleSupplier velocitySupplier;
    private final Timer timer = new Timer();
    private double currentInput = 0.0;

    /**
     * Creates a StaticCharacterizationCommand for swerve drive.
     *
     * @return The created StaticCharacterizationCommand object.
     */
    public static StaticCharacterizationCommand createSwerve() {
        return new StaticCharacterizationCommand(
                SwerveSubsystem.getInstance()::setVoltsForCharacterization,
                () -> {
                    double driveVelocityAverage = 0.0;
                    for (SwerveModuleState states : SwerveSubsystem.getInstance().getStates()) {
                        driveVelocityAverage += states.speedMetersPerSecond;
                    }

                    return driveVelocityAverage / 4.0;
                });
    }

    /**
     * Creates a StaticCharacterizationCommand for swerve drive.
     *
     * @param characterizationInputConsumer A consumer to feed the input voltage for all drive motors.
     * @param velocitySupplier A supplier to get the average velocity of all drive motors.
     */
    public StaticCharacterizationCommand(
            DoubleConsumer characterizationInputConsumer, DoubleSupplier velocitySupplier) {
        inputConsumer = characterizationInputConsumer;
        this.velocitySupplier = velocitySupplier;
        addRequirements(SwerveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        currentInput = timer.get() * currentRampFactor.get();
        inputConsumer.accept(currentInput);
    }

    @Override
    public boolean isFinished() {
        return velocitySupplier.getAsDouble() >= minVelocity.get();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Static Characterization output: " + currentInput + " volts");
        SwerveSubsystem.getInstance().stopMotors();
    }
}
