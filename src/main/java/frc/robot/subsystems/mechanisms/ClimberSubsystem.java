// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import static frc.lib.lib2706.ErrorCheck.configureSpark;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.lib2706.SubsystemChecker;
import frc.lib.lib2706.SubsystemChecker.SubsystemType;
import frc.lib.lib2706.networktables.NTUtil;
import frc.lib.lib2706.networktables.TunableDouble;
import frc.robot.Config.CANID;
import frc.robot.Config.ClimberConfig;
import frc.robot.Config.NTConfig;
import frc.robot.subsystems.misc.SparkMaxManagerSubsystem;

import java.util.function.DoubleSupplier;

/**
 * The ClimberSubsystem class represents the subsystem responsible for controlling the climber mechanism.
 * It handles the setup of the SparkMax motor controller, encoder, NetworkTables, and tunable parameters.
 */
public class ClimberSubsystem extends SubsystemBase {
    private static ClimberSubsystem INSTANCE;

    private final CANSparkMax m_sparkmax;
    private final RelativeEncoder m_encoder;

    private final DoublePublisher pubPos, pubVel;
    private final TunableDouble tunablePosForReducedSpeed, tunableSpeedFast, tunableSpeedSlow;

    /*
     * Returns the singleton instance for the Climber Subsystem
     */
    public static ClimberSubsystem getInstance() {
        if (INSTANCE == null) {
            SubsystemChecker.canSubsystemConstruct(SubsystemType.ClimberSubsystem);
            INSTANCE = new ClimberSubsystem();
        }
        return INSTANCE;
    }

    /**
     * The ClimberSubsystem class represents the subsystem responsible for controlling the climber mechanism.
     * It handles the setup of the SparkMax motor controller, encoder, NetworkTables, and tunable parameters.
     */
    private ClimberSubsystem() {
        /* Setup SparkMax */
        m_sparkmax = new CANSparkMax(CANID.CLIMBER.val(), MotorType.kBrushless);

        configureSpark(
                "climber set can timeout", () -> m_sparkmax.setCANTimeout(CANID.CANTIMEOUT_MS));
        configureSpark("climber factory defaults", () -> m_sparkmax.restoreFactoryDefaults());
        configureSpark(
                "climber set can timeout", () -> m_sparkmax.setCANTimeout(CANID.CANTIMEOUT_MS));
        m_sparkmax.setInverted(ClimberConfig.invertMotor);
        configureSpark(
                "climber current limit",
                () -> m_sparkmax.setSmartCurrentLimit(ClimberConfig.currentLimit));

        /* Setup encoder */
        m_encoder = m_sparkmax.getEncoder();
        m_encoder.setPositionConversionFactor(ClimberConfig.posConvFactor);
        m_encoder.setVelocityConversionFactor(ClimberConfig.velConvFactor);

        /* Setup NetworkTables */
        pubPos = NTUtil.doublePubFast(NTConfig.climberTable, "Position (drum rotations)");
        pubVel = NTUtil.doublePubFast(NTConfig.climberTable, "Velocity (drum rotations per sec)");

        /* Setup Tunables */
        tunablePosForReducedSpeed =
                new TunableDouble(
                        "Position for reduced speed (drum rotations)",
                        NTConfig.climberTable,
                        ClimberConfig.posForReducedSpeed);
        tunableSpeedFast =
                new TunableDouble(
                        "Fast speed (volts)", NTConfig.climberTable, ClimberConfig.speedFast);
        tunableSpeedSlow =
                new TunableDouble(
                        "Slow speed (volts)", NTConfig.climberTable, ClimberConfig.speedSlow);

        SparkMaxManagerSubsystem.getInstance()
                .register(NTConfig.nonSwerveSparkMaxAlertGroup, m_sparkmax);
    }

    /**
     * Returns the current position of the climber mechanism.
     *
     * @return the number of rotations of the drum since boot
     */
    public double getPosition() {
        return m_encoder.getPosition();
    }

    /**
     * Returns the velocity of the climber mechanism.
     *
     * @return the velocity in rotations per second of the drum
     */
    public double getVelocity() {
        return m_encoder.getVelocity();
    }

    /**
     * Sets the voltage of the climber mechanism.
     *
     * @param volts the voltage to set
     */
    public void setVoltage(double volts) {
        m_sparkmax.setVoltage(volts);
    }

    /**
     * Adjusts the speed of the climber based on its current position.
     * If the position is below a certain threshold, the climber will spin at a fast speed.
     * Otherwise, it will spin at a slow speed.
     *
     * @param percentSpeed The percentage of the maximum speed at which the climber should spin.
     */
    public void smartSpin(double percentSpeed) {
        if (Math.abs(getPosition()) < tunablePosForReducedSpeed.get()) {
            setVoltage(percentSpeed * tunableSpeedFast.get());
        } else {
            setVoltage(percentSpeed * tunableSpeedSlow.get());
        }
    }

    /**
     * This method is called periodically to update the state of the ClimberSubsystem.
     * It publishes the current position and velocity of the climber mechanism.
     */
    @Override
    public void periodic() {
        pubPos.accept(getPosition());
        pubVel.accept(getVelocity());
    }

    /**
     * Stops the climber mechanism.
     */
    public void stopMotors() {
        m_sparkmax.stopMotor();
    }

    /**
     * Creates a Command object for controlling the climber mechanism.
     *
     * @param getPercentOutput a DoubleSupplier that provides the desired percent output for the climber motors
     * @return a Command object that controls the climber mechanism
     */
    public Command climberCommand(DoubleSupplier getPercentOutput) {
        return run(() -> setVoltage(MathUtil.applyDeadband(getPercentOutput.getAsDouble(), 0.35)))
                .finallyDo(() -> stopMotors());
    }
}
