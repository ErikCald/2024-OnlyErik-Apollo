package frc.robot.subsystems.mechanisms;

import static frc.lib.lib2706.ErrorCheck.configureSpark;
import static frc.lib.lib2706.ErrorCheck.errSpark;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.lib2706.controllers.PIDConfig;
import frc.lib.lib2706.networktables.NTUtil;
import frc.lib.lib2706.networktables.TunableDouble;
import frc.lib.lib2706.networktables.TunablePIDConfig;
import frc.robot.Config.CANID;
import frc.robot.Config.NTConfig;
import frc.robot.Config.ShooterConfig;
import frc.robot.Config.ShooterConfig.ShooterSetpoint;
import frc.robot.subsystems.misc.SparkMaxManagerSubsystem;

/**
 * The ShooterSubsystem class represents the subsystem responsible for controlling
 * the shooter mechanism of the robot. It initializes and configures the SparkMax
 * motor controllers, encoders, PID controllers, and network tables for monitoring
 * and control.
 */
public class ShooterSubsystem extends SubsystemBase {
    private static ShooterSubsystem INSTANCE;

    private CANSparkMax m_topSpark, m_botSpark;
    private SparkPIDController m_topSparkPID, m_botSparkPID;
    private RelativeEncoder m_topEncoder, m_botEncoder;
    private DoublePublisher pubTopVel, pubBotVel;
    private TunableDouble tunableEjectVolts;
    private TunablePIDConfig tunablePID0, tunablePID1, tunablePID3Slowdown;

    private double m_setpointRPM = 0;

    /**
     * The ShooterSubsystem class represents the shooter mechanism of the robot.
     * It provides methods for controlling and managing the shooter.
     */
    public static ShooterSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new ShooterSubsystem();
        return INSTANCE;
    }

    /**
     * The ShooterSubsystem class represents the subsystem responsible for controlling
     * the shooter mechanism of the robot. It initializes and configures the SparkMax
     * motor controllers, encoders, PID controllers, and network tables for monitoring
     * and control.
     */
    public ShooterSubsystem() {
        /* Setup Sparkmaxs and encoders */
        m_topSpark = new CANSparkMax(CANID.SHOOTER_TOP.val(), MotorType.kBrushless);
        m_botSpark = new CANSparkMax(CANID.SHOOTER_BOT.val(), MotorType.kBrushless);

        m_topEncoder = setupSparkmax("shooter top", m_topSpark);
        m_botEncoder = setupSparkmax("shooter bot", m_botSpark);

        /* Setup Spark PID Controllers */
        m_topSparkPID = m_topSpark.getPIDController();
        m_botSparkPID = m_botSpark.getPIDController();
        updatePIDValues(ShooterConfig.pid0Config, false);
        updatePIDValues(ShooterConfig.pid1Config, false);
        updatePIDValues(ShooterConfig.pid3SlowdownConfig, false);

        /* Setup NetworkTables */
        pubTopVel = NTUtil.doublePubFast(NTConfig.shooterTable, "Top Vel (RPM)");
        pubBotVel = NTUtil.doublePubFast(NTConfig.shooterTable, "Bot Vel (RPM)");

        /* Setup Tunable values */
        tunableEjectVolts =
                new TunableDouble("Eject (volts)", NTConfig.shooterTable, ShooterConfig.ejectVolts);

        tunablePID0 =
                new TunablePIDConfig(
                        (config) -> updatePIDValues(config, true),
                        NTConfig.shooterTable.getSubTable("PID0"),
                        ShooterConfig.pid0Config);
        tunablePID1 =
                new TunablePIDConfig(
                        (config) -> updatePIDValues(config, true),
                        NTConfig.shooterTable.getSubTable("PID1"),
                        ShooterConfig.pid1Config);
        tunablePID3Slowdown =
                new TunablePIDConfig(
                        (config) -> updatePIDValues(config, true),
                        NTConfig.shooterTable.getSubTable("PID3Slowdown"),
                        ShooterConfig.pid3SlowdownConfig);

        /* Finalize Sparkmaxs */
        configureSpark("shooter bot remove can timeout", () -> m_botSpark.setCANTimeout(0));
        configureSpark("shooter top remove can timeout", () -> m_topSpark.setCANTimeout(0));
        SparkMaxManagerSubsystem.getInstance()
                .register(NTConfig.nonSwerveSparkMaxAlertGroup, m_botSpark, m_topSpark);
    }

    /**
     * Setup a SparkMax motor controller with the given name and configuration.
     *
     * @param name the name of the motor controller
     * @param sparkmax the SparkMax motor controller to configure
     * @return the encoder of the SparkMax motor controller
     */
    private RelativeEncoder setupSparkmax(String name, CANSparkMax sparkmax) {
        configureSpark(
                name + " set can timeout", () -> sparkmax.setCANTimeout(CANID.CANTIMEOUT_MS));
        configureSpark(name + " factory defaults", () -> sparkmax.restoreFactoryDefaults());
        configureSpark(
                name + " set can timeout", () -> sparkmax.setCANTimeout(CANID.CANTIMEOUT_MS));
        configureSpark(name + " set idle mode", () -> sparkmax.setIdleMode(ShooterConfig.idleMode));
        configureSpark(
                name + " set volt comp",
                () -> sparkmax.enableVoltageCompensation(ShooterConfig.voltComp));
        configureSpark(
                name + " set curr limit",
                () -> sparkmax.setSmartCurrentLimit(ShooterConfig.currentLimit));

        RelativeEncoder encoder = sparkmax.getEncoder();
        configureSpark(
                name + " set pos conv factor",
                () -> encoder.setVelocityConversionFactor(ShooterConfig.velConvFactor));
        return encoder;
    }

    /**
     * Updates the PID values for the Shooter motors.
     *
     * @param config the PID configuration containing the new values
     */
    private void updatePIDValues(PIDConfig config, boolean isRuntime) {
        int slot = config.pidSlot;
        if (!isRuntime) {
            configureSpark("shooter bot f" + slot, () -> m_botSparkPID.setP(config.kF, slot));
            configureSpark("shooter bot p" + slot, () -> m_botSparkPID.setP(config.kP, slot));
            configureSpark("shooter bot i" + slot, () -> m_botSparkPID.setI(config.kI, slot));
            configureSpark("shooter bot d" + slot, () -> m_botSparkPID.setD(config.kD, slot));
            configureSpark(
                    "shooter bot iZone" + slot, () -> m_botSparkPID.setIZone(config.iZone, slot));

            configureSpark("shooter top f" + slot, () -> m_topSparkPID.setP(config.kF, slot));
            configureSpark("shooter top p" + slot, () -> m_topSparkPID.setP(config.kP, slot));
            configureSpark("shooter top i" + slot, () -> m_topSparkPID.setI(config.kI, slot));
            configureSpark("shooter top d" + slot, () -> m_topSparkPID.setD(config.kD, slot));
            configureSpark(
                    "shooter top iZone" + slot, () -> m_topSparkPID.setIZone(config.iZone, slot));
        } else {
            errSpark("shooter bot f" + slot, m_botSparkPID.setP(config.kF, slot));
            errSpark("shooter bot p" + slot, m_botSparkPID.setP(config.kP, slot));
            errSpark("shooter bot i" + slot, m_botSparkPID.setI(config.kI, slot));
            errSpark("shooter bot d" + slot, m_botSparkPID.setD(config.kD, slot));
            errSpark("shooter bot iZone" + slot, m_botSparkPID.setIZone(config.iZone, slot));

            errSpark("shooter top f" + slot, m_topSparkPID.setP(config.kF, slot));
            errSpark("shooter top p" + slot, m_topSparkPID.setP(config.kP, slot));
            errSpark("shooter top i" + slot, m_topSparkPID.setI(config.kI, slot));
            errSpark("shooter top d" + slot, m_topSparkPID.setD(config.kD, slot));
            errSpark("shooter top iZone" + slot, m_topSparkPID.setIZone(config.iZone, slot));
        }
    }

    @Override
    public void periodic() {
        pubBotVel.accept(getBotVelRPM());
        pubTopVel.accept(getTopVelRPM());

        /* Update tunables */
        tunablePID0.updateValues();
        tunablePID1.updateValues();
        tunablePID3Slowdown.updateValues();
    }

    /**
     * Stops the motors of the shooter subsystem.
     */
    public void stopMotors() {
        m_botSpark.stopMotor();
        m_topSpark.stopMotor();
        m_setpointRPM = 0;
    }

    /**
     * Returns the velocity of the bot in RPM (Rotations Per Minute).
     *
     * @return The velocity of the bot in RPM.
     */
    public double getBotVelRPM() {
        return m_botEncoder.getVelocity();
    }

    /**
     * Returns the velocity of the top in RPM (Rotations Per Minute).
     *
     * @return The velocity of the top in RPM.
     */
    public double getTopVelRPM() {
        return m_topEncoder.getVelocity();
    }

    /**
     * Sets the RPM (Rotations Per Minute) for the shooter mechanism.
     *
     * @param vel the desired RPM value
     */
    public void setRPM(double vel) {
        int slotID = 0;
        if (vel <= 50) {
            slotID = 3; // Slowdown PID
        } else {
            slotID = 1; // Normal PID
        }

        m_topSparkPID.setReference(vel, ControlType.kVelocity, slotID);
        m_botSparkPID.setReference(vel, ControlType.kVelocity, slotID);

        m_setpointRPM = vel;
    }

    /**
     * Sets the voltage for the shooter mechanism.
     *
     * @param volts the voltage to set for the shooter mechanism
     */
    public void setVoltage(double volts) {
        m_topSpark.setVoltage(volts);
        m_botSpark.setVoltage(volts);
    }

    /**
     * Sets the brake mode for the shooter mechanism.
     *
     * @param enableBrake true to enable brake mode, false to enable coast mode
     */
    public void setBrake(boolean enableBreak) {
        m_topSpark.setIdleMode(enableBreak ? IdleMode.kBrake : IdleMode.kCoast);
        m_botSpark.setIdleMode(enableBreak ? IdleMode.kBrake : IdleMode.kCoast);
    }

    /**
     * Spins the shooter mechanism to the specified setpoint.
     *
     * @param setpoint The desired shooter setpoint.
     */
    public void spinToSetpoint(ShooterSetpoint setpoint) {
        setRPM(setpoint.getRPM());
    }

    /**
     * Ejects a note by setting the voltage of the shooter subsystem to the tunable eject voltage.
     */
    public void ejectNote() {
        setVoltage(tunableEjectVolts.get());
    }

    /**
     * Checks if the shooter is at the specified setpoint.
     *
     * @param setpoint The desired shooter setpoint.
     * @return True if the shooter is at the setpoint, false otherwise.
     */
    public boolean isAtSetpoint(ShooterSetpoint setpoint) {
        return Math.abs(getTopVelRPM() - setpoint.getRPM())
                < ShooterSetpoint.FIRE_THRESHLD.getRPM();
    }

    /**
     * Returns a Command object that stops the motors of the shooter subsystem.
     *
     * @return the Command object to stop the motors
     */
    public Command stopCommand() {
        return runOnce(() -> stopMotors());
    }

    /**
     * Create a Command to eject the note and stops the shooter when canceled.
     *
     * @return the Command object for ejecting a note
     */
    public Command ejectNoteCommand() {
        return run(() -> ejectNote()).finallyDo(() -> stopMotors());
    }

    /**
     * Create a Command to slow down the shooter.
     * Runs PID to reduce the speed of the Shooter to 0.
     * Then runs brake mode until this Command is canceled by another Command.
     *
     * Uses an RPM of -5 instead of 0 to ensure PID is used to slow down the shooter.
     *
     * @return A Command that requires only the ShooterSubsystem
     */
    public Command slowdownCommand() {
        return Commands.sequence(
                runOnce(() -> setBrake(false)),
                run(() -> spinToSetpoint(ShooterSetpoint.SLOWDOWN))
                        .until(() -> isAtSetpoint(ShooterSetpoint.SLOWDOWN)),
                runOnce(() -> setBrake(true)),
                run(() -> stopMotors()));
    }

    /**
     * Creates a command to spin the shooter to the specified setpoint.
     *
     * @param setpoint The desired setpoint for the shooter.
     * @return The command to spin the shooter to the setpoint.
     */
    public Command spinCommand(ShooterSetpoint setpoint) {
        return run(() -> spinToSetpoint(setpoint));
    }
}
