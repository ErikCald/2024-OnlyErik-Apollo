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
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.lib2706.networktables.TunableDouble;
import frc.lib.lib6328.PolynomialRegression;
import frc.robot.Config.NTConfig;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * The VelocityCharacterizationCommand class is used to characterize the kS
 * (static friction) and kV (velocity) gain of a subsystem of a swerve drive.
 *
 * It then prints the number of data points, the coefficient of determination (R2),
 * and the feedforward gains (kS and kV). The R2 value is a measure of how well
 * the regression fits the data, while kS and kV are the static and velocity
 * gains, respectively, for the feedforward controller.
 */
public class VelocityCharacterizationCommand extends Command {
    private static final NetworkTable table =
        NTConfig.characterizingTable.getSubTable("VelocityCharacterization");
    private static final TunableDouble currentRampFactor =
            new TunableDouble("VoltageRampPerSec", table, 0.1);

    private static final double START_DELAY_SECS = 2.0;

    private FeedForwardCharacterizationData data;
    private final Consumer<Double> voltageConsumer;
    private final Supplier<Double> velocitySupplier;

    private final Timer timer = new Timer();

    /**
     * Creates a VelocityCharacterizationCommand for swerve drive.
     *
     * @return The VelocityCharacterizationCommand object.
     */
    public static VelocityCharacterizationCommand createSwerve() {
        return new VelocityCharacterizationCommand(
                SwerveSubsystem.getInstance(),
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
     * A command used for velocity characterization of a subsystem. This
     * command requires a subsystem, a voltage consumer, and a velocity supplier.
     * <p>
     * The voltage consumer is used to apply voltage to the subsystem, while the
     * velocity supplier is used to measure the velocity of the subsystem.
     */
    public VelocityCharacterizationCommand(
            Subsystem subsystem,
            Consumer<Double> voltageConsumer,
            Supplier<Double> velocitySupplier) {
        addRequirements(subsystem);
        this.voltageConsumer = voltageConsumer;
        this.velocitySupplier = velocitySupplier;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        data = new FeedForwardCharacterizationData();
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (timer.get() < START_DELAY_SECS) {
            voltageConsumer.accept(0.0);
        } else {
            double voltage = (timer.get() - START_DELAY_SECS) * currentRampFactor.get();
            voltageConsumer.accept(voltage);
            data.add(velocitySupplier.get(), voltage);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        voltageConsumer.accept(0.0);
        timer.stop();
        data.print();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * A class representing the data for feedforward characterization.
     */
    public static class FeedForwardCharacterizationData {
        private final List<Double> velocityData = new LinkedList<>();
        private final List<Double> voltageData = new LinkedList<>();

        /**
         * Adds a data point to the feedforward characterization data.
         *
         * @param velocity The velocity data point.
         * @param voltage  The voltage data point.
         */
        public void add(double velocity, double voltage) {
            if (Math.abs(velocity) > 1E-4) {
                velocityData.add(Math.abs(velocity));
                voltageData.add(Math.abs(voltage));
            }
        }

        /**
         * Prints the feedforward characterization results.
         * If there is no data, nothing will be printed.
         */
        public void print() {
            if (velocityData.size() == 0 || voltageData.size() == 0) {
                return;
            }

            PolynomialRegression regression =
                    new PolynomialRegression(
                            velocityData.stream().mapToDouble(Double::doubleValue).toArray(),
                            voltageData.stream().mapToDouble(Double::doubleValue).toArray(),
                            1);

            System.out.println("FF Characterization Results:");
            System.out.println("\tCount=" + Integer.toString(velocityData.size()) + "");
            System.out.println(String.format("\tR2=%.5f", regression.R2()));
            System.out.println(String.format("\tkS=%.5f", regression.beta(0)));
            System.out.println(String.format("\tkV=%.5f", regression.beta(1)));
        }
    }
}
