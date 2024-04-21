// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.characterization;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;

import frc.lib.lib2706.networktables.NTUtil;
import frc.lib.lib2706.networktables.TunableDouble;
import frc.robot.Config.NTConfig;
import frc.robot.Config.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import java.util.function.DoubleSupplier;

/**
 * This class represents a command for characterizing the wheel radius of a swerve drive system.
 *
 * This was written by FRC 6328 Mechanical Advantage and modified by FRC 2706 Merge Robotics.
 */
public class WheelDiameterCharacterizationCommand extends Command {
    private static final NetworkTable table =
            NTConfig.characterizingTable.getSubTable("WheelDiameter");
    private static final TunableDouble characterizationSpeed =
            new TunableDouble("DesiredSpeedRadsPerSec", table, 0.1);
    private static final DoublePublisher pubDrivePosition =
            NTUtil.doublePubFast(table, "AverageDrivePosition");
    private static final DoublePublisher pubAccumGyroYaw =
            NTUtil.doublePubFast(table, "AccumGyroYaw");
    private static final DoublePublisher pubCalcWheelDiameter =
            NTUtil.doublePubFast(table, "CalculatedWheelDiameterMeters");

    private final double drivebaseRadius = SwerveConfig.drivebaseRadius;
    private final DoubleSupplier gyroYawRadsSupplier =
            () -> SwerveSubsystem.getInstance().getHeading().getRadians();

    private final double omegaDirection;
    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

    private double lastGyroYawRads = 0.0;
    private double accumGyroYawRads = 0.0;
    private double[] startWheelPositions;
    private double currentEffectiveWheelRadius = 0.0;

    /**
     * A command used for characterizing the wheel radius of a swerve drive subsystem.
     *
     * @param invertRotatingDirection If true the robot will rotate in the opposite direction.
     */
    public WheelDiameterCharacterizationCommand(boolean invertRotatingDirection) {
        if (invertRotatingDirection) {
            omegaDirection = -1.0;
        } else {
            omegaDirection = 1.0;
        }

        addRequirements(SwerveSubsystem.getInstance());
    }

    /**
     * Returns an array of drive positions in radians.
     * Size of array is equal to the number of modules.
     *
     * @return an array of wheel positions in radians
     */
    public double[] getWheelPositionsRadians() {
        SwerveModulePosition[] positionsMeters = SwerveSubsystem.getInstance().getPositions();
        double positionsRadians[] = new double[4];
        for (int i = 0; i < 4; i++) {
            positionsRadians[i] = positionsMeters[i].distanceMeters / SwerveConfig.wheelRadius;
        }

        return positionsRadians;
    }

    @Override
    public void initialize() {
        lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
        accumGyroYawRads = 0.0;

        startWheelPositions = getWheelPositionsRadians();

        omegaLimiter.reset(0);
    }

    @Override
    public void execute() {
        // Run drive at velocity
        double angVel = omegaLimiter.calculate(omegaDirection * characterizationSpeed.get());
        SwerveSubsystem.getInstance().drive(new ChassisSpeeds(0, 0, angVel), false, true);

        // Get yaw and wheel positions
        accumGyroYawRads +=
                MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
        lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
        double averageWheelPosition = 0.0;
        double[] wheelPositiions = getWheelPositionsRadians();
        for (int i = 0; i < 4; i++) {
            averageWheelPosition += Math.abs(wheelPositiions[i] - startWheelPositions[i]);
        }
        averageWheelPosition /= 4.0;

        // Calculate effective wheel radius
        currentEffectiveWheelRadius = (accumGyroYawRads * drivebaseRadius) / averageWheelPosition;

        // Publish values
        pubDrivePosition.accept(averageWheelPosition);
        pubAccumGyroYaw.accept(accumGyroYawRads);
        pubCalcWheelDiameter.accept(currentEffectiveWheelRadius * 2);
    }

    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().stopMotors();

        if (accumGyroYawRads <= Math.PI * 2.0) {
            System.out.println("Not enough data for characterization");
            pubCalcWheelDiameter.accept(0.0);
        } else {
            System.out.println(
                    "Effective Wheel Radius: "
                            + Units.metersToInches(currentEffectiveWheelRadius * 2)
                            + " inches");
            System.out.println(
                    "Effective Wheel Radius: " + (currentEffectiveWheelRadius * 2) + " meters");
        }
    }
}
