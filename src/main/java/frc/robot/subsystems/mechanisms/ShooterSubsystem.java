package frc.robot.subsystems.mechanisms;

import static frc.robot.subsystems.mechanisms.ShooterStateMachine.ShooterModes.SHOOT_SPEAKER;
import static frc.robot.subsystems.mechanisms.ShooterStateMachine.ShooterModes.STOP_SHOOTER;
import static frc.robot.subsystems.mechanisms.ShooterStateMachine.States.AMP_LAUNCH_READY;
import static frc.robot.subsystems.mechanisms.ShooterStateMachine.States.SPEAKER_LAUNCH_READY;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.lib2706.ErrorCheck;
import frc.lib.lib2706.networktables.TunableDouble;
import frc.robot.Config;
import frc.robot.Config.NTConfig;
import frc.robot.subsystems.mechanisms.ShooterStateMachine.ShooterModes;
import frc.robot.subsystems.mechanisms.ShooterStateMachine.States;
import frc.robot.subsystems.misc.SparkMaxManagerSubsystem;

import java.util.function.BooleanSupplier;

public class ShooterSubsystem extends SubsystemBase {
    private CANSparkMax m_motor;
    private SparkPIDController m_pidController;
    private RelativeEncoder m_encoder;
    private boolean closedLoopControl = false;
    private boolean stateFulControl = false;

    private TunableDouble kP =
            new TunableDouble("PID0/kP", NTConfig.shooterTable, Config.ShooterConstants.kP);
    private TunableDouble kI =
            new TunableDouble("PID0/kI", NTConfig.shooterTable, Config.ShooterConstants.kI);
    private TunableDouble kD =
            new TunableDouble("PID0/kD", NTConfig.shooterTable, Config.ShooterConstants.kD);
    private TunableDouble kFF =
            new TunableDouble("PID0/kFF", NTConfig.shooterTable, Config.ShooterConstants.kFF);

    private TunableDouble kP1 =
            new TunableDouble("PID1/kP", NTConfig.shooterTable, Config.ShooterConstants.kP1);
    private TunableDouble kI1 =
            new TunableDouble("PID1/kI", NTConfig.shooterTable, Config.ShooterConstants.kI1);
    private TunableDouble kD1 =
            new TunableDouble("PID1/kD", NTConfig.shooterTable, Config.ShooterConstants.kD1);
    private TunableDouble kFF1 =
            new TunableDouble("PID1/kFF", NTConfig.shooterTable, Config.ShooterConstants.kFF1);
    private TunableDouble shooterTreshHold =
            new TunableDouble("tresh hold", NTConfig.shooterTable, 100);

    private DoublePublisher velocityPub;
    private StringPublisher statePub;
    private BooleanPublisher shooterReadyPub;
    private ShooterStateMachine shooterStates = new ShooterStateMachine();

    private static ShooterSubsystem shooter;

    public static ShooterSubsystem getInstance() {
        if (shooter == null) shooter = new ShooterSubsystem();
        return shooter;
    }

    public ShooterSubsystem() {
        System.out.println("[Init] Creating Shooter");
        m_motor = new CANSparkMax(Config.ShooterConstants.MOTOR_ID, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();

        m_motor.setCANTimeout(500); // Units in miliseconds
        m_motor.setIdleMode(IdleMode.kBrake);
        m_motor.setInverted(false);

        m_pidController = m_motor.getPIDController();
        m_encoder = m_motor.getEncoder();

        // Voltage compensation
        m_motor.enableVoltageCompensation(10); // adjust on final robot
        m_motor.setSmartCurrentLimit(70);
        setBrake(true);

        m_pidController.setOutputRange(
                Config.ShooterConstants.kMinOutput, Config.ShooterConstants.kMaxOutput);
        setPIDGains(kP.get(), kI.get(), kD.get(), 0);
        setFFGains(kFF.get(), 0);

        setPIDGains(kP1.get(), kI1.get(), kD1.get(), 1);
        setFFGains(kFF1.get(), 1);

        ErrorCheck.sparkBurnFlash("Shooter", m_motor);

        velocityPub =
                NTConfig.shooterTable
                        .getDoubleTopic("Shooter Velocity RPM")
                        .publish(PubSubOption.periodic(0.02));
        shooterReadyPub =
                NTConfig.shooterTable
                        .getBooleanTopic("Shooter is Ready to shoot")
                        .publish(PubSubOption.periodic(0.02));
        statePub =
                NTConfig.shooterTable
                        .getStringTopic("Shooter state")
                        .publish(PubSubOption.periodic(0.02));

        SparkMaxManagerSubsystem.getInstance()
                .register(NTConfig.nonSwerveSparkMaxAlertGroup, m_motor);
    }

    public double getVelocityRPM() {
        return m_encoder.getVelocity();
    }

    public void setRPM(double setPoint) {
        int slotID = 1;
        m_pidController.setReference(setPoint, ControlType.kVelocity, slotID);
    }

    public void setVoltage(double setVolt) {
        m_motor.setVoltage(setVolt);
    }

    public void stop() {
        m_motor.stopMotor();
    }

    public void setMode(ShooterModes desiredMode) {
        shooterStates.setMode(desiredMode);
    }

    public void allowAutoMovement(boolean isThereNote) {
        if (!isThereNote && stateFulControl)
            setMode(STOP_SHOOTER); // it should set to stop now when there is no note in intake

        if (closedLoopControl) {
            setRPM(shooterStates.getDesiredVelocityRPM());
        } else {
            setVoltage(shooterStates.getDesiredVoltage());
        }
    }

    public void setBrake(boolean enableBreak) {
        m_motor.setIdleMode(enableBreak ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public States getCurrentState() {
        return shooterStates.getCurrentState();
    }

    private void setPIDGains(double kP, double kI, double kD, int slotID) {
        m_pidController.setP(kP, slotID);
        m_pidController.setI(kI, slotID);
        m_pidController.setD(kD, slotID);
    }

    private void setFFGains(double kFF, int slotID) {
        m_pidController.setFF(kFF, slotID);
    }

    public boolean isReadyToShoot() {
        return getCurrentState().equals(SPEAKER_LAUNCH_READY)
                || getCurrentState().equals(AMP_LAUNCH_READY);
    }

    public void setStateMachineOff() {
        stateFulControl = false;
    }

    public void setStateMachineOn() {
        stateFulControl = true;
    }

    /*---------------------------Commands---------------------------*/

    /**
     * This allows the Shooter's state machine to have effect on the beheavior of the shooter
     * This should be called every run loop cycle, set it as the default command
     * @return Default Intake Command
     */
    public Command defaultShooterCommand(BooleanSupplier isThereNote) {
        return Commands.sequence(
                runOnce(() -> setMode(STOP_SHOOTER)),
                run(() -> allowAutoMovement(isThereNote.getAsBoolean())));
    }

    /**
     * Sets the mode of the Shooter's state nachine to "SHOOT_SPEAKER"
     * This will set the velocity to the
     * @return
     */
    public Command speedUpForSpeakerCommand() {
        return Commands.deadline(
                Commands.waitUntil(() -> isReadyToShoot()),
                Commands.runOnce(() -> setMode(SHOOT_SPEAKER)));
    }

    /**
     * Command that will set the the given mode if shooter is stopped,
     * or stop the shooter if it's currently doing an action.
     *
     * @param mode Mode to toggle.
     * @return Command to attach to a button as onTrue.
     */
    public Command toggleSpinUpCommand(ShooterModes mode) {
        return Commands.runOnce(
                () -> {
                    if (shooterStates.getDesiredMode() != ShooterModes.STOP_SHOOTER) {
                        shooterStates.setMode(ShooterModes.STOP_SHOOTER);
                    } else {
                        shooterStates.setMode(mode);
                    }
                });
    }

    @Override
    public void periodic() {
        TunableDouble.ifChanged(
                hashCode(), () -> setPIDGains(kP.get(), kI.get(), kD.get(), 0), kP, kI, kD);
        TunableDouble.ifChanged(hashCode(), () -> setFFGains(kFF.get(), 0), kFF);

        TunableDouble.ifChanged(
                hashCode(), () -> setPIDGains(kP1.get(), kI1.get(), kD1.get(), 1), kP1, kI1, kD1);
        TunableDouble.ifChanged(hashCode(), () -> setFFGains(kFF1.get(), 1), kFF1);

        // Check if this method would work like this
        if (stateFulControl == true) {
            shooterStates.isInRange(
                    () ->
                            getVelocityRPM()
                                    > shooterStates.getDesiredVelocityRPM()
                                            - shooterTreshHold.get());
            shooterStates.updateState();
        }

        velocityPub.accept(getVelocityRPM());
        shooterReadyPub.accept(isReadyToShoot());
        statePub.accept(getCurrentState().toString());
    }
}
