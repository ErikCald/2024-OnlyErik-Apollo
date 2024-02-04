// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.lib.lib2706.ErrorCheck.configureSpark;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.lib2706.ProfiledExternalPIDController;
import frc.lib.lib2706.UpdateSimpleFeedforward;
import frc.robot.Config;
import frc.robot.Config.ArmConfig;
import frc.robot.Config.CanID;

public class ArmSubsystem extends SubsystemBase {
    private static ArmSubsystem INSTANCE = null;

    public static ArmSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ArmSubsystem();
        }
        return INSTANCE;
    }

    private final CANSparkMax m_sparkLeader;
    private final CANSparkMax m_sparkFollower;
    private final SparkPIDController m_sparkPid;
    private final SparkAbsoluteEncoder m_encoder;

    private final ProfiledExternalPIDController m_profiledPid;

    private final PIDController m_simPid;
    private boolean m_runPidSim = false;

    private SimpleMotorFeedforward m_feedforward;
    private UpdateSimpleFeedforward m_updateFeedforward;

    private double lastSpeed = 0;
    private double lastFFVoltage = 0;

    private SingleJointedArmSim m_armSim;

    private ArmDisplay m_armDisplay;
    private DoublePublisher pubPos, pubVel, pubPosSet;
    private DoublePublisher pubPosProSet, pubVelProSet, pubAccelSet;

    /**
     * Creates a new ArmSubsystem.
     */
    private ArmSubsystem() {
        // spotless:off
        /*
         * Setup CANSparkMaxs
         */
        m_sparkLeader = new CANSparkMax(CanID.ARM_SPARK_LEADER, Config.NEO_MOTORTYPE);
        m_sparkFollower = new CANSparkMax(CanID.ARM_SPARK_FOLLOWER, Config.NEO_MOTORTYPE);

        configureSpark("Arm Leader CANTimeoutBlocking", () -> m_sparkLeader.setCANTimeout(Config.SPARK_SETTERS_TIMEOUT));
        configureSpark("Arm Follower CANTimeoutBlocking", () -> m_sparkLeader.setCANTimeout(Config.SPARK_SETTERS_TIMEOUT));

        configureSpark("Arm Leader FactoryDefaults", () -> m_sparkLeader.restoreFactoryDefaults());
        configureSpark("Arm Follower FactoryDefaults", () -> m_sparkFollower.restoreFactoryDefaults());

        configureSpark("Arm Leader CANTimeoutBlocking", () -> m_sparkLeader.setCANTimeout(Config.SPARK_SETTERS_TIMEOUT));
        configureSpark("Arm Follower CANTimeoutBlocking", () -> m_sparkLeader.setCANTimeout(Config.SPARK_SETTERS_TIMEOUT));

        configureSpark("Arm Leader CurrentLimit", () -> m_sparkLeader.setSmartCurrentLimit(ArmConfig.CURRENT_LIMIT));
        configureSpark("Arm Follower CurrentLimit", () -> m_sparkFollower.setSmartCurrentLimit(ArmConfig.CURRENT_LIMIT));

        m_sparkLeader.setInverted(ArmConfig.LEADER_INVERTED);
        configureSpark("Arm Follower follows leader", () -> m_sparkFollower.follow(m_sparkLeader, ArmConfig.FOLLOWER_INVERTED));

        configureSpark("Arm Leader BrakeMode", () -> m_sparkLeader.setIdleMode(IdleMode.kBrake));
        configureSpark("Arm Follower BrakeMode", () -> m_sparkFollower.setIdleMode(IdleMode.kBrake));

        configureSpark("Arm Leader VoltageCompensation", () -> m_sparkLeader.enableVoltageCompensation(ArmConfig.VOLTAGE_COMP));
        configureSpark("Arm Follower VoltageCompensation", () -> m_sparkFollower.enableVoltageCompensation(ArmConfig.VOLTAGE_COMP));

        /*
         * Setup SparkMaxPidControllers
         */
        m_sparkPid = m_sparkLeader.getPIDController();

        configureSpark("Arm Leader kP", () -> m_sparkPid.setP(ArmConfig.kP));
        configureSpark("Arm Leader kI", () -> m_sparkPid.setI(ArmConfig.kI));
        configureSpark("Arm Leader kD", () -> m_sparkPid.setD(ArmConfig.kD));
        configureSpark("Arm Leader iZone", () -> m_sparkPid.setIZone(ArmConfig.iZone));

        /**
         * Setup Absolute Encoder
         */
        m_encoder = m_sparkLeader.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        configureSpark("Arm AbsEncoder PosUnits", () -> m_encoder.setPositionConversionFactor(ArmConfig.POS_CONV_FACTOR));
        configureSpark("Arm AbsEncoder VelUnits", () -> m_encoder.setVelocityConversionFactor(ArmConfig.VEL_CONV_FACTOR));
        configureSpark("Arm AbsEncoder Inverted", () -> m_encoder.setInverted(ArmConfig.ABS_ENCODER_INVERTED));
        configureSpark("Arm AbsEncoder Offset", () -> m_encoder.setZeroOffset(ArmConfig.ABS_ENCODER_OFFSET));

        // Set status frame to get absolute encoder position & velocity every 20 ms
        configureSpark("Arm AbsEncoder PosCanFramePeriod", () -> m_sparkLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20));
        configureSpark("Arm AbsEncoder VelCanFramePeriod", () -> m_sparkLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20));

        configureSpark("Arm AbsEncoder UseAsPidFeedback", () -> m_sparkPid.setFeedbackDevice(m_encoder));

        /**
         * Setup soft limits after encoder
         */
        configureSpark("Arm SoftLimit ForwardVal", () -> m_sparkLeader.setSoftLimit(SoftLimitDirection.kForward, ArmConfig.FORW_LIMIT));
        configureSpark("Arm SoftLimit ReverseVal", () -> m_sparkLeader.setSoftLimit(SoftLimitDirection.kReverse, ArmConfig.REV_LIMIT));
        configureSpark("Arm SoftLimit ForwardEnable", () -> m_sparkLeader.enableSoftLimit(SoftLimitDirection.kForward, ArmConfig.SOFT_LIMIT_ENABLE));
        configureSpark("Arm SoftLimit ReverseEnable", () -> m_sparkLeader.enableSoftLimit(SoftLimitDirection.kReverse, ArmConfig.SOFT_LIMIT_ENABLE));

        /** 
         * Setup ProfiledPidControllers and Simulation PIDControllers 
         */
        m_profiledPid =
                new ProfiledExternalPIDController(
                        new Constraints(ArmConfig.MAX_VEL, ArmConfig.MAX_ACCEL));

        m_simPid = new PIDController(ArmConfig.kP, ArmConfig.kI, ArmConfig.kD);
        m_simPid.setIZone(ArmConfig.iZone);

        /**
         * Setup networktables
         */
        m_armDisplay = new ArmDisplay();

        NetworkTable table = NetworkTableInstance.getDefault().getTable(ArmConfig.DATA_NT_TABLE);

        pubPos = table.getDoubleTopic("Pos Deg").publish(PubSubOption.periodic(0.02));
        pubVel = table.getDoubleTopic("Vel DegPerSec").publish(PubSubOption.periodic(0.02));

        pubPosSet = table.getDoubleTopic("Pos Setpoint Deg").publish(PubSubOption.periodic(0.02));
        pubPosProSet = table.getDoubleTopic("Pos ProfileSetpoint Deg").publish(PubSubOption.periodic(0.02));
        pubVelProSet = table.getDoubleTopic("Vel ProfileSetpoint DegPerSec").publish(PubSubOption.periodic(0.02));
        pubAccelSet = table.getDoubleTopic("Accel ProfileSetpoint DegPerSecPerSec").publish(PubSubOption.periodic(0.02));
        
        /**
         * Setup SimpleMotorFeedforward
         */
        m_feedforward = ArmConfig.SIMPLE_FF;

        m_updateFeedforward = new UpdateSimpleFeedforward(
            (ff) -> m_feedforward = ff, 
            table, 
            ArmConfig.SIMPLE_FF.ks, 
            ArmConfig.SIMPLE_FF.kv, 
            ArmConfig.SIMPLE_FF.ka);

        /**
         * Burn flash on CANSparkMaxs
         */
        burnFlash();

        configureSpark("Arm Leader CANTimeoutNonBlocking", () -> m_sparkLeader.setCANTimeout(0));
        configureSpark("Arm Follower CANTimeoutNonBlocking", () -> m_sparkFollower.setCANTimeout(0));

        //spotless:on

        /**
         * Configure simulation object
         */
        m_armSim =
                new SingleJointedArmSim(
                        DCMotor.getNEO(1),
                        ArmConfig.GEAR_RATIO,
                        ArmConfig.MOMENT_OF_INERTIA,
                        ArmConfig.LENGTH_METERS,
                        ArmConfig.REV_LIMIT - Math.toRadians(10),
                        ArmConfig.FORW_LIMIT + Math.toRadians(10),
                        true,
                        ArmConfig.REV_LIMIT + Math.toRadians(5), // Starting angle
                        VecBuilder.fill(ArmConfig.SIM_NOISE) // Add noise with a small std-dev
                        );
    }

    /**
     * Burn flash after a 0.2 second delay to ensure settings are saved.
     */
    private void burnFlash() {
        try {
            Thread.sleep(200);
        } catch (Exception e) {
        }

        configureSpark("Arm Leader BurnFlash", () -> m_sparkLeader.burnFlash());
        configureSpark("Arm Follower BurnFlash", () -> m_sparkFollower.burnFlash());
    }

    public double calculateGravityCompensation() {
        return 0;
    }

    @Override
    public void periodic() {
        if (Config.ARM_TUNING) {
            m_updateFeedforward.checkForUpdates();
        }

        pubPos.accept(Math.toDegrees(m_encoder.getPosition()));
        pubVel.accept(Math.toDegrees(m_encoder.getVelocity()));
    }

    public double getPosition() {
        if (!RobotBase.isSimulation()) {
            return m_encoder.getPosition();
        } else {
            return m_armSim.getAngleRads();
        }
    }

    public double getVelocity() {
        if (!RobotBase.isSimulation()) {
            return m_encoder.getVelocity();
        } else {
            return m_armSim.getVelocityRadPerSec();
        }
    }

    public void resetProfiledPIDControllers() {
        m_profiledPid.reset(getPosition(), getVelocity());

        lastFFVoltage = 0;
        lastSpeed = getVelocity();

        if (RobotBase.isSimulation()) {
            m_simPid.reset();
        }
    }

    public void setAngle(double angleRad) {
        double pidSetpoint = m_profiledPid.calculatePIDSetpoint(getPosition(), angleRad);

        double acceleration = (m_profiledPid.getSetpoint().velocity - lastSpeed) / 0.02;

        lastFFVoltage =
                m_feedforward.calculate(m_profiledPid.getSetpoint().velocity, acceleration)
                        + calculateGravityCompensation();

        m_sparkPid.setReference(
                pidSetpoint, ControlType.kPosition, 0, lastFFVoltage, ArbFFUnits.kVoltage);

        lastSpeed = m_profiledPid.getSetpoint().velocity;

        pubPosSet.accept(Math.toDegrees(angleRad));
        pubPosProSet.accept(Math.toDegrees(pidSetpoint));
        pubVelProSet.accept(Math.toDegrees(m_profiledPid.getSetpoint().velocity));
        pubAccelSet.accept(Math.toDegrees(acceleration));

        m_runPidSim = true;
    }

    public boolean isAtSetpoint() {
        return m_profiledPid.atGoal(ArmConfig.POSITION_TOLERANCE, ArmConfig.VELOCITY_TOLERANCE);
    }

    public void stopMotors() {
        m_sparkLeader.stopMotor();
        m_sparkFollower.stopMotor();

        lastFFVoltage = 0;
        m_runPidSim = false;
    }

    @Override
    public void simulationPeriodic() {
        REVPhysicsSim.getInstance().run();

        double voltageApplied = 0;

        if (m_runPidSim && DriverStation.isEnabled()) {
            // spotless:off
            // Use the simPid to simulate the CANSparkMax pid controller
            voltageApplied = lastFFVoltage
                    + m_simPid.calculate(getPosition(), m_profiledPid.getSetpoint().position);

            //spotless:on
        }

        // Pass the simulation with the inputs as voltages
        m_armSim.setInput(voltageApplied - calculateGravityCompensation());

        // Update simulation
        m_armSim.update(0.020); // 20ms clock cycle

        // Update the simulation of the load on the battery
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
    }

    public void testFeedforward(double additionalVoltage) {
        double voltage = additionalVoltage + calculateGravityCompensation();
        m_sparkPid.setReference(voltage, ControlType.kVoltage);
    }
}
