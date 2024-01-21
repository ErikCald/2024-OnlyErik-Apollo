// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private static ArmSubsystem INSTANCE = null;

    private final CANSparkMax m_spark;
    private final SparkMaxPIDController m_sparkPid;
    private final SparkMaxAbsoluteEncoder m_encoder;

    private final ProfiledExternalPIDController m_profiledPid;

    private final PIDController m_simPid;
    private boolean m_runPidSim = false;

    private SimpleMotorFeedforward m_feedforward;

    private double lastSpeed = 0;
    private double lastFFVoltage = 0;

    private DoublePublisher pubPos, pubVel, pubPosSet;
    private DoublePublisher pubPosProSet, pubVelProSet, pubAccelSet;

    /** Creates a new ArmSubsystem. */
    public ArmSubsystem() {
        /*
         * Setup CANSparkMaxs
         */
        
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
