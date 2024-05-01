// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import static frc.lib.lib2706.ErrorCheck.configureSpark;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.lib.lib2706.networktables.NTUtil;
import frc.lib.lib2706.networktables.TunableDouble;
import frc.robot.Config.CANID;
import frc.robot.Config.IntakeConfig;
import frc.robot.Config.NTConfig;
import frc.robot.subsystems.misc.SparkMaxManagerSubsystem;

/**
 * The IntakeSubsystem class represents the intake mechanism of the robot.
 * It provides methods for controlling the intake subsystem.
 */
public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem instance;

    private CANSparkMax m_sparkmax;

    private final DigitalInput centerSwitch, shooterSideSwitch;
    private final Debouncer centerDebouncer, shooterSideDebouncer, shooterSideLongDebouncer;
    private final BooleanPublisher pubCenterSwitch, pubShooterSideSwitch, pubShooterSideLongSwitch;
    private boolean isCenterSwitchActive, isShooterSideSwitchActive, isShooterSideLongSwitchActive;
    private final TunableDouble tunableReverseNoteVolts, tunableFireVolts, tunableIntakeVolts;

    /**
     * The IntakeSubsystem class represents the intake mechanism of the robot.
     * It provides methods for controlling the intake subsystem.
     */
    public static IntakeSubsystem getInstance() {
        if (instance == null) instance = new IntakeSubsystem();
        return instance;
    }

    /**
     * The IntakeSubsystem class represents the subsystem responsible for controlling the intake
     * mechanism of the robot. It initializes and configures the SparkMax motor controller, digital
     * proximity switches, and NetworkTables for monitoring the intake status.
     */
    private IntakeSubsystem() {
        /* Setup sparkmax */
        m_sparkmax = new CANSparkMax(CANID.INTAKE.val(), MotorType.kBrushless);
        configureSpark("intake factory defaults", () -> m_sparkmax.restoreFactoryDefaults());
        configureSpark(
                "intake set can timeout", () -> m_sparkmax.setCANTimeout(CANID.CANTIMEOUT_MS));
        m_sparkmax.setInverted(IntakeConfig.invertMotor);
        configureSpark(
                "intake current limit",
                () -> m_sparkmax.setSmartCurrentLimit(IntakeConfig.currentLimit));
        configureSpark("intake idle mode", () -> m_sparkmax.setIdleMode(IntakeConfig.idleMode));
        configureSpark(
                "intake volt comp",
                () -> m_sparkmax.enableVoltageCompensation(IntakeConfig.voltageCompensation));

        /* Setup digital proximity switches */
        centerSwitch = new DigitalInput(IntakeConfig.centerSwitchChannel);
        shooterSideSwitch = new DigitalInput(IntakeConfig.shooterSideSwitchChannel);

        Debouncer.DebounceType type = Debouncer.DebounceType.kBoth;
        centerDebouncer = new Debouncer(IntakeConfig.shortDebouncePeriod, type);
        shooterSideDebouncer = new Debouncer(IntakeConfig.shortDebouncePeriod, type);
        shooterSideLongDebouncer = new Debouncer(IntakeConfig.longDebouncePeriod, type);

        /* Setup NetworkTables */
        NetworkTable table = NTConfig.intakeTable;
        pubCenterSwitch = table.getBooleanTopic("CenterSwitch").publish(NTUtil.fastPeriodic());
        pubShooterSideSwitch =
                table.getBooleanTopic("ShooterSideSwitch").publish(NTUtil.fastPeriodic());
        pubShooterSideLongSwitch =
                table.getBooleanTopic("ShooterSideSwitchLong").publish(NTUtil.fastPeriodic());

        /* Setup tunable parameters */
        tunableReverseNoteVolts =
                new TunableDouble("ReverseNote (volts)", table, IntakeConfig.reverseNoteVolts);
        tunableFireVolts = new TunableDouble("Fire (volts)", table, IntakeConfig.fireVolts);
        tunableIntakeVolts = new TunableDouble("Intake (volts)", table, IntakeConfig.intakeVolts);

        configureSpark("intake remove can timeout", () -> m_sparkmax.setCANTimeout(0));
        SparkMaxManagerSubsystem.getInstance()
                .register(NTConfig.nonSwerveSparkMaxAlertGroup, m_sparkmax);
    }

    /**
     * Returns the status of the center switch.
     *
     * @return true if the center switch is active, false otherwise.
     */
    public boolean centerSwitch() {
        return isCenterSwitchActive;
    }

    /**
     * Returns the status of the shooter side switch.
     *
     * @return true if the shooter side switch is active, false otherwise
     */
    public boolean shooterSideSwitch() {
        return isShooterSideSwitchActive;
    }

    /**
     * Returns the status of the shooter side long switch.
     *
     * @return true if the shooter side long switch is active, false otherwise
     */
    public boolean shooterSideSwitchLong() {
        return isShooterSideLongSwitchActive;
    }

    /**
     * Sets the voltage of the intake mechanism.
     *
     * @param voltage the voltage to set
     */
    public void setVoltage(double voltage) {
        m_sparkmax.setVoltage(voltage);
    }

    /**
     * Stops the intake mechanism.
     */
    public void stopMotors() {
        m_sparkmax.stopMotor();
    }

    /**
     * This method is called periodically to update the state of the intake subsystem.
     * It calculates the status of various switches and publishes the results.
     */
    @Override
    public void periodic() {
        isCenterSwitchActive = centerDebouncer.calculate(!centerSwitch.get());
        isShooterSideSwitchActive = shooterSideDebouncer.calculate(!shooterSideSwitch.get());
        isShooterSideLongSwitchActive =
                shooterSideLongDebouncer.calculate(!shooterSideSwitch.get());

        pubCenterSwitch.accept(isCenterSwitchActive);
        pubShooterSideSwitch.accept(isShooterSideSwitchActive);
        pubShooterSideLongSwitch.accept(isShooterSideLongSwitchActive);
    }

    /**
     * Reverses the note by setting the voltage to the tunable reverse note volts.
     */
    public void reverseNote() {
        setVoltage(tunableReverseNoteVolts.get());
    }

    /**
     * Fires a note by setting the voltage of the intake subsystem to the value specified by tunableFireVolts.
     */
    public void fireNote() {
        setVoltage(tunableFireVolts.get());
    }

    /**
     * Sets the voltage of the intake mechanism to the value specified by tunableIntakeVolts.
     */
    public void intakeNote() {
        setVoltage(tunableIntakeVolts.get());
    }

    /**
     * Returns a Command object that stops the intake mechanism.
     *
     * @return the Command object that stops the intake mechanism
     */
    public Command stopCommand() {
        return runOnce(() -> stopMotors());
    }

    /**
     * Returns a Command to reverse the note until the shooter side proximity switch is no longer active.
     *
     * @return A command that requires the IntakeSubsystem.
     */
    public Command reverseNoteCommand() {
        return run(() -> reverseNote()).until(() -> !shooterSideSwitch()).andThen(stopCommand());
    }

    /**
     * Returns a Command object that fires a note.
     *
     * The Command will spin up the intake until the shooterSideSwitch becomes active and not active again.
     * The intake is spindown once completed.
     *
     * @return the Command object that fires a note
     */
    public Command fireNoteCommand() {
        return Commands.deadline(
                        Commands.sequence(
                                new WaitUntilCommand(() -> shooterSideSwitch()).withTimeout(0.2),
                                new WaitUntilCommand(() -> !shooterSideSwitchLong())),
                        run(() -> fireNote()))
                .andThen(stopCommand());
    }
}
