package frc.robot.subsystems.arm;

import static frc.lib.lib2706.ErrorCheck.configureSpark;
import static frc.lib.lib2706.ErrorCheck.errSpark;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.lib2706.SubsystemChecker;
import frc.lib.lib2706.SubsystemChecker.SubsystemType;
import frc.lib.lib2706.controllers.PIDConfig;
import frc.lib.lib2706.controllers.ProfiledExternalPIDController;
import frc.lib.lib2706.networktables.AdvantageUtil;
import frc.lib.lib2706.networktables.NTUtil;
import frc.lib.lib2706.networktables.TunableDouble;
import frc.lib.lib2706.networktables.TunablePIDConfig;
import frc.lib.lib2706.networktables.TunableProfiledPIDConfig;
import frc.robot.Config.ArmConfig;
import frc.robot.Config.CANID;
import frc.robot.Config.NTConfig;
import frc.robot.subsystems.misc.SparkMaxManagerSubsystem;

/**
 * The ArmSubsystem class represents the subsystem responsible for controlling the arm of the robot.
 * It handles the configuration and control of the SparkMax motor, absolute encoder, and PID controllers.
 * The class provides methods for setting the arm angle, calculating the PID slot, resetting the profiled PID controller,
 * and retrieving the current arm position in radians.
 */
public class ArmSubsystem extends SubsystemBase {
    private static ArmSubsystem instance = null;

    private final CANSparkMax m_sparkmax;
    private final SparkAbsoluteEncoder m_absEncoder;
    private final SparkPIDController m_sparkPID;

    private final ProfiledExternalPIDController m_profiledPID;

    private final NetworkTable tunablesTable = NTConfig.armTable.getSubTable("Tunables");
    private final NetworkTable dataTable = NTConfig.armTable.getSubTable("Data");
    private final TunableProfiledPIDConfig tunableTransportPID0;
    private final TunablePIDConfig tunablePID1, tunablePID2, tunablePID3;
    private final TunableDouble tunableAbsOffset;

    private final DoublePublisher pubCurrSetpoint, pubGoalSetpoint;
    private final DoublePublisher pubMeasPos, pubMeasVel;
    private final IntegerPublisher pubActivePIDSlot;
    private final DoubleArrayPublisher pubArmPose = NTUtil.doubleArrayPubFast(dataTable, "ArmPose");

    /**
     * The ArmSubsystem class represents the subsystem responsible for controlling the arm of the robot.
     * It follows the Singleton design pattern to ensure that only one instance of the subsystem is created.
     */
    public static ArmSubsystem getInstance() {
        if (instance == null) {
            SubsystemChecker.subsystemConstructed(SubsystemType.ArmSubsystem);
            instance = new ArmSubsystem();
        }
        return instance;
    }

    /**
     * The ArmSubsystem class represents the subsystem responsible for controlling the arm mechanism of the robot.
     * It initializes and configures the SparkMax motor controller, absolute encoder, PID controllers, and network table publishers.
     * The class also provides methods for setting PID values, updating setpoints, and publishing sensor data.
     */
    private ArmSubsystem() {
        /* Setup SparkMax */
        m_sparkmax = new CANSparkMax(CANID.ARM.val(), MotorType.kBrushless);
        configureSpark("Arm restore factory defaults", () -> (m_sparkmax.restoreFactoryDefaults()));
        configureSpark("arm set CANTimeout", () -> m_sparkmax.setCANTimeout(CANID.CANTIMEOUT_MS));
        configureSpark(
                "Arm set current limits",
                () -> m_sparkmax.setSmartCurrentLimit(ArmConfig.currentLimit));
        m_sparkmax.setInverted(ArmConfig.invertMotor);
        configureSpark("Arm set brakes when idle", () -> (m_sparkmax.setIdleMode(IdleMode.kBrake)));
        configureSpark("Arm voltage compesentation", () -> m_sparkmax.enableVoltageCompensation(6));

        configureSpark(
                "Arm set soft limits forward",
                () ->
                        m_sparkmax.setSoftLimit(
                                SoftLimitDirection.kForward, (float) (ArmConfig.forwardSoftLimit)));
        configureSpark(
                "Arm sets soft limits reverse",
                () ->
                        m_sparkmax.setSoftLimit(
                                SoftLimitDirection.kReverse, (float) (ArmConfig.reverseSoftLimit)));
        configureSpark(
                "Arm enables soft limits forward",
                () ->
                        m_sparkmax.enableSoftLimit(
                                SoftLimitDirection.kForward, ArmConfig.enableSoftLimit));
        configureSpark(
                "Arm enable soft limit reverse",
                () ->
                        m_sparkmax.enableSoftLimit(
                                SoftLimitDirection.kReverse, ArmConfig.enableSoftLimit));

        configureSpark( // Set absolute encoder position data frequency to 20 ms
                "Arm set periodic frame period",
                () -> m_sparkmax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20));
        configureSpark( // Set absolute encoder velocity data frequency to 20 ms
                "Arm set periodic frame period",
                () -> m_sparkmax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20));

        m_absEncoder = m_sparkmax.getAbsoluteEncoder(Type.kDutyCycle);
        configureSpark(
                "Absolute encoder set inerted",
                () -> m_absEncoder.setInverted(ArmConfig.invertEncoder));
        configureSpark(
                "Absolute encoder set position conersation factor",
                () -> m_absEncoder.setPositionConversionFactor(ArmConfig.posConvFactor));
        configureSpark(
                "Absolute Encoder set velocity conversion factor",
                () -> m_absEncoder.setVelocityConversionFactor(ArmConfig.velConvFactor));
        configureSpark(
                "Absolute encoder set zero offset",
                () -> setEncoderOffset(ArmConfig.absEncoderOffset));

        m_sparkPID = m_sparkmax.getPIDController();
        configureSpark(
                "Pid controller arm set feedback device",
                () -> m_sparkPID.setFeedbackDevice(m_absEncoder));

        /* Setup tunable encoder offset */
        tunableAbsOffset =
                new TunableDouble(
                        "AbsEncoderOffset (deg)", tunablesTable, ArmConfig.absEncoderOffset);

        /* Setup the profiled pid controller */
        m_profiledPID = ArmConfig.transportPIDConfig.createExternalController();

        /* Setup initial PID values on SparkMax */
        updatePIDValues(ArmConfig.transportPIDConfig.kPIDConfig, false);
        updatePIDValues(ArmConfig.pid1Config, false);
        updatePIDValues(ArmConfig.pid2Config, false);
        updatePIDValues(ArmConfig.pid3Config, false);

        /* Setup tunable PIDs */
        tunableTransportPID0 =
                new TunableProfiledPIDConfig(
                        (config) -> {
                            updatePIDValues(config.kPIDConfig, true);
                            config.applyConfig(m_profiledPID);
                        },
                        NTConfig.armTable.getSubTable("TransportPID0"),
                        ArmConfig.transportPIDConfig);

        tunablePID1 =
                new TunablePIDConfig(
                        (config) -> updatePIDValues(config, true),
                        NTConfig.armTable.getSubTable("PID1"),
                        ArmConfig.pid1Config);
        tunablePID2 =
                new TunablePIDConfig(
                        (config) -> updatePIDValues(config, true),
                        NTConfig.armTable.getSubTable("PID2"),
                        ArmConfig.pid2Config);
        tunablePID3 =
                new TunablePIDConfig(
                        (config) -> updatePIDValues(config, true),
                        NTConfig.armTable.getSubTable("PID3"),
                        ArmConfig.pid3Config);

        /* Setup network table publishers */
        pubCurrSetpoint = NTUtil.doublePubFast(dataTable, "CurrSetpoint (deg)");
        pubGoalSetpoint = NTUtil.doublePubFast(dataTable, "GoalSetpoint (deg)");
        pubMeasPos = NTUtil.doublePubFast(dataTable, "MeasPos (deg)");
        pubMeasVel = NTUtil.doublePubFast(dataTable, "MeasVel (deg/s)");
        pubActivePIDSlot =
                dataTable.getIntegerTopic("ActivePIDSlot").publish(NTUtil.fastPeriodic());

        configureSpark("Arm set CANTimeout", () -> m_sparkmax.setCANTimeout(0));
        SparkMaxManagerSubsystem.getInstance()
                .register(NTConfig.nonSwerveSparkMaxAlertGroup, m_sparkmax);
    }

    /**
     * Updates the PID values for the arm subsystem.
     *
     * @param config the PID configuration containing the new values
     */
    private void updatePIDValues(PIDConfig config, boolean isRuntime) {
        int slot = config.pidSlot;
        if (!isRuntime) {
            configureSpark("arm set spark f" + slot, () -> m_sparkPID.setP(config.kF, slot));
            configureSpark("arm set spark p" + slot, () -> m_sparkPID.setP(config.kP, slot));
            configureSpark("arm set spark i" + slot, () -> m_sparkPID.setI(config.kI, slot));
            configureSpark("arm set spark d" + slot, () -> m_sparkPID.setD(config.kD, slot));
            configureSpark(
                    "arm set spark iZone" + slot, () -> m_sparkPID.setIZone(config.iZone, slot));
        } else {
            errSpark("arm set spark f" + slot, m_sparkPID.setP(config.kF, slot));
            errSpark("arm set spark p" + slot, m_sparkPID.setP(config.kP, slot));
            errSpark("arm set spark i" + slot, m_sparkPID.setI(config.kI, slot));
            errSpark("arm set spark d" + slot, m_sparkPID.setD(config.kD, slot));
            errSpark("arm set spark iZone" + slot, m_sparkPID.setIZone(config.iZone, slot));
        }
    }

    /**
     * Updates the absolute encoder offset of the arm subsystem.
     *
     * @param offsetDeg the offset in degrees
     * @return the REVLibError indicating the success or failure of the operation
     */
    private REVLibError setEncoderOffset(double offsetDeg) {
        double offsetRad = Math.toRadians(offsetDeg - ArmConfig.shiftEncoderRange);
        return m_absEncoder.setZeroOffset(offsetRad);
    }

    /**
     * This method is called periodically to update the arm subsystem.
     * It publishes measurements to network tables, updates tunable PID values,
     * and performs other necessary operations.
     */
    @Override
    public void periodic() {
        // Publish measurements to network tables
        pubMeasPos.accept(Math.toDegrees(getAngleRad()));
        pubMeasVel.accept(Math.toDegrees(getVelocityRadPS()));
        publishAdvantageScopeArmPose(getAngleRad());

        // Update tunable pid values
        TunableDouble.ifChanged(
                hashCode(), () -> setEncoderOffset(tunableAbsOffset.get()), tunableAbsOffset);
        tunableTransportPID0.updateValues();
        tunablePID1.updateValues();
        tunablePID2.updateValues();
        tunablePID3.updateValues();
    }

    /**
     * Sets the angle of the arm subsystem to the specified value.
     *
     * @param angleDeg the desired angle in degrees
     */
    public void setAngle(double angleDeg) {
        angleDeg =
                MathUtil.clamp(angleDeg, ArmConfig.MIN_ARM_ANGLE_DEG, ArmConfig.MAX_ARM_ANGLE_DEG);

        double pidSetpointRad =
                m_profiledPID.calculatePIDSetpoint(getAngleRad(), Math.toRadians(angleDeg));
        int pidSlot = calcuatePIDSlot(angleDeg);

        m_sparkPID.setReference(
                pidSetpointRad + Math.toRadians(ArmConfig.shiftEncoderRange),
                ControlType.kPosition,
                pidSlot,
                0);

        pubCurrSetpoint.accept(Math.toDegrees(pidSetpointRad));
        pubGoalSetpoint.accept(angleDeg);
        pubActivePIDSlot.accept(pidSlot);
    }

    /**
     * Calculates the PID slot based on the given angle in degrees.
     *
     * @param angleDeg the angle in degrees
     * @return the PID slot to be used
     */
    public int calcuatePIDSlot(double angleDeg) {
        // Use transport PID if angle is outside 10 degrees of the setpoint
        if (Math.abs(angleDeg - Math.toDegrees(getAngleRad())) > 10) {
            return 0;
        }

        // Use PID 1 for shooter shots
        if (angleDeg >= 25 && angleDeg < 55) {
            return 1;
        }

        // Use transport PID for all other setpoints
        return 0;
    }

    /**
     * Resets the profiled PID controller for the arm subsystem.
     * This method resets the profiled PID controller by setting the current angle and velocity of the arm.
     */
    public void resetProfile() {
        m_profiledPID.reset(getAngleRad(), getVelocityRadPS());
    }

    /**
     * Returns the current position of the arm in radians.
     *
     * @return the current position of the arm in radians
     */
    public double getAngleRad() {
        return m_absEncoder.getPosition() - Math.toRadians(ArmConfig.shiftEncoderRange);
    }

    /**
     * Returns the velocity of the arm subsystem in radians per second.
     *
     * @return the velocity in radians per second.
     */
    public double getVelocityRadPS() {
        return m_absEncoder.getVelocity();
    }

    /**
     * Publishes the pose of the arm for AdvantageScope to display with the CAD model.
     *
     * @param angleDeg the angle in radians
     */
    public void publishAdvantageScopeArmPose(double angleRad) {
        pubArmPose.accept(
                AdvantageUtil.deconstruct(
                        new Pose3d(
                                new Translation3d(-0.15, 0, 0.375),
                                new Rotation3d(0, -angleRad, 0))));
    }

    /**
     * Sets the idle mode.
     *
     * @param mode the idle mode to set
     */
    public void setArmIdleMode(IdleMode mode) {
        m_sparkmax.setIdleMode(mode);
    }

    /**
     * Stops the motors of the arm subsystem.
     */
    public void stopMotors() {
        m_sparkmax.stopMotor();
    }
}
