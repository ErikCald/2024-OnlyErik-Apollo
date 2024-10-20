package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.lib254.SwerveSetpoint;
import frc.lib.lib254.SwerveSetpointGenerator;
import frc.lib.lib2706.GeomUtil;
import frc.lib.lib2706.networktables.AdvantageUtil;
import frc.lib.lib2706.networktables.NTUtil;
import frc.lib.lib2706.networktables.TunableDouble;
import frc.lib.lib2706.networktables.TunableProfiledPIDConfig;
import frc.lib.lib2706.swerve.PoseBuffer;
import frc.lib.lib6328.AllianceFlipUtil;
import frc.robot.Config;
import frc.robot.Config.CANID;
import frc.robot.Config.GeneralConfig;
import frc.robot.Config.NTConfig;
import frc.robot.Config.RobotID;
import frc.robot.Config.SwerveConfig;
import frc.robot.Config.SwerveConfig.ModuleLimits;

import java.util.Optional;

/**
 * The SwerveSubsystem class represents the subsystem responsible for controlling the swerve drive of the robot.
 */
public class SwerveSubsystem extends SubsystemBase {
    private final WPI_PigeonIMU m_gyro;
    private Rotation2d m_simHeading = new Rotation2d();

    private final SwerveSetpointGenerator m_setpointGenerator;
    private SwerveSetpoint m_currentSetpoint = new SwerveSetpoint(SwerveConfig.numSwerveModules);
    private ModuleLimits m_moduleLimits = ModuleLimits.TELEOP_FAST;

    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final SwerveModuleAbstract[] m_modules;
    private final PoseBuffer m_poseBuffer;
    private int modSyncCounter = 0;

    private NetworkTable directPIDTable = NTConfig.swerveTable.getSubTable("DirectPID");
    private final ProfiledPIDController m_pidX, m_pidY, m_pidRot;
    private final PIDController m_holdHeadingPid;
    private double lastHeadingRadians = 0;
    private Pose2d m_desiredPose = null;

    private final DoubleArrayPublisher pubEstimatedPose, pubDesiredStates, pubMeasuredStates;
    private final DoublePublisher pubPoseX, pubPoseY, pubPoseRot, pubGyroRate;
    private final DoublePublisher pubVelSetpointX, pubVelSetpointY, pubVelSetpointRot;
    private final DoublePublisher pubMeasVelX, pubMeasVelY, pubMeasVelRot;
    private final DoublePublisher pubDesiredX, pubDesiredY, pubDesiredRot;
    private final DoublePublisher pubVelCmdX, pubVelCmdY, pubVelCmdRot;
    private final IntegerEntry pubNumSyncs;

    private final TunableDouble tunableHeadingkP;
    private final TunableProfiledPIDConfig tunableXPid, tunableYPid, tunableRotPid;

    private static SwerveSubsystem instance;

    /**
     * Returns the instance of the SwerveSubsystem.
     * If the instance does not exist, it creates a new one.
     *
     * @return the instance of the SwerveSubsystem
     */
    public static SwerveSubsystem getInstance() {
        if (instance == null) {
            instance = new SwerveSubsystem();
        }
        return instance;
    }

    /**
     * The SwerveSubsystem class represents a subsystem for controlling a swerve drive system.
     * It initializes the gyro, swerve modules, odometry, and PID controllers.
     * It also provides methods for driving the robot, resetting odometry, and accessing pose information.
     */
    private SwerveSubsystem() {
        m_gyro = new WPI_PigeonIMU(CANID.PIGEON.val());
        m_gyro.configFactoryDefault();

        switch (RobotID.getActiveID()) {
            default:
            case APOLLO:
                m_modules =
                        new SwerveModuleAbstract[] {
                            new SwerveModuleSparkMaxCancoderV5(
                                    Config.SwerveConfig.Mod0.constants, "FL"),
                            new SwerveModuleSparkMaxCancoderV5(
                                    Config.SwerveConfig.Mod1.constants, "FR"),
                            new SwerveModuleSparkMaxCancoderV5(
                                    Config.SwerveConfig.Mod2.constants, "BL"),
                            new SwerveModuleSparkMaxCancoderV5(
                                    Config.SwerveConfig.Mod3.constants, "BR")
                        };
                break;

            case SIMULATION:
                m_modules =
                        new SwerveModuleAbstract[] {
                            new SwerveModuleSim(Config.SwerveConfig.Mod0.constants, "FL"),
                            new SwerveModuleSim(Config.SwerveConfig.Mod1.constants, "FR"),
                            new SwerveModuleSim(Config.SwerveConfig.Mod2.constants, "BL"),
                            new SwerveModuleSim(Config.SwerveConfig.Mod3.constants, "BR")
                        };
                break;

            case BEETLE:
                throw new UnsupportedOperationException("Beetle does not support swerve.");
        }

        m_setpointGenerator =
                new SwerveSetpointGenerator(
                        SwerveConfig.swerveKinematics, SwerveConfig.moduleLocations);

        m_poseEstimator =
                new SwerveDrivePoseEstimator(
                        Config.SwerveConfig.swerveKinematics,
                        getYaw(),
                        getPositions(),
                        new Pose2d());

        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto
                // has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::drivePathPlanner, // Method that will drive the robot given ROBOT
                // RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely
                        // live in your Constants class
                        new PIDConstants(10.0, 0.0, 0.0), // Translation PID constants kP = 5.0
                        new PIDConstants(10.0, 0.0, 0.0), // Rotation PID constants kP = 5.0
                        SwerveConfig.maxSpeed, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest
                        // module.
                        new ReplanningConfig() // Default path replanning config. See the API for
                        // the options here
                        ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
                );

        m_poseBuffer = new PoseBuffer();

        /* Setup ProfiledPIDControllers */
        m_pidX = SwerveConfig.translationPIDConfig.createController();
        m_pidY = SwerveConfig.translationPIDConfig.createController();
        m_pidRot = SwerveConfig.rotationPIDConfig.createController();
        m_pidRot.enableContinuousInput(-Math.PI, Math.PI);

        /* Setup tunable constants */
        tunableHeadingkP =
                new TunableDouble(
                        "HoldHeadingkP", NTConfig.swerveTable, SwerveConfig.holdHeadingkP);
        tunableXPid =
                new TunableProfiledPIDConfig(
                        (config) -> config.applyConfig(m_pidX),
                        NTConfig.swerveTable.getSubTable("PidX"),
                        SwerveConfig.translationPIDConfig);
        tunableYPid =
                new TunableProfiledPIDConfig(
                        (config) -> config.applyConfig(m_pidY),
                        NTConfig.swerveTable.getSubTable("PidY"),
                        SwerveConfig.translationPIDConfig);
        tunableRotPid =
                new TunableProfiledPIDConfig(
                        (config) -> config.applyConfig(m_pidRot),
                        NTConfig.swerveTable.getSubTable("PidRot"),
                        SwerveConfig.rotationPIDConfig);

        /* PID to hold the heading stable */
        m_holdHeadingPid = new PIDController(tunableHeadingkP.get(), 0, 0);
        m_holdHeadingPid.enableContinuousInput(-Math.PI, Math.PI);
        lastHeadingRadians = getHeading().getRadians();

        /* Setup NetworkTables */
        pubEstimatedPose = NTUtil.doubleArrayPubFast(NTConfig.swerveTable, "EstimatedPose");
        pubDesiredStates = NTUtil.doubleArrayPubFast(NTConfig.swerveTable, "DesiredStates");
        pubMeasuredStates = NTUtil.doubleArrayPubFast(NTConfig.swerveTable, "MeasuredStates");

        pubPoseX = NTUtil.doublePubFast(NTConfig.swerveTable, "PoseX (m)");
        pubPoseY = NTUtil.doublePubFast(NTConfig.swerveTable, "PoseY (m)");
        pubPoseRot = NTUtil.doublePubFast(NTConfig.swerveTable, "PoseRot (deg)");
        pubGyroRate = NTUtil.doublePubFast(NTConfig.swerveTable, "GyroRate (degps)");

        pubMeasVelX = NTUtil.doublePubFast(directPIDTable, "MeasuredVelX (mps)");
        pubMeasVelY = NTUtil.doublePubFast(directPIDTable, "MeasuredVelY (mps)");
        pubMeasVelRot = NTUtil.doublePubFast(directPIDTable, "MeasuredVelRot (degps)");

        pubVelCmdX = NTUtil.doublePubFast(directPIDTable, "VelCommandedX (mps)");
        pubVelCmdY = NTUtil.doublePubFast(directPIDTable, "VelCommandedY (mps)");
        pubVelCmdRot = NTUtil.doublePubFast(directPIDTable, "VelCommandedRot (degps)");

        // Direct PID NT values
        pubVelSetpointX = NTUtil.doublePubFast(directPIDTable, "VelSetpointX (mps)");
        pubVelSetpointY = NTUtil.doublePubFast(directPIDTable, "VelSetpointY (mps)");
        pubVelSetpointRot = NTUtil.doublePubFast(directPIDTable, "VelSetpointRot (degps)");
        pubDesiredX = NTUtil.doublePubFast(directPIDTable, "DesiredX (m)");
        pubDesiredY = NTUtil.doublePubFast(directPIDTable, "DesiredY (m)");
        pubDesiredRot = NTUtil.doublePubFast(directPIDTable, "DesiredRot (degps)");

        pubNumSyncs =
                NTConfig.swerveTable
                        .getIntegerTopic("NumEncoderSyncs")
                        .getEntry(0, PubSubOption.periodic(NTConfig.SLOW_PERIODIC_SECONDS));
        pubNumSyncs.set(0);
    }

    /**
     * Drives the swerve subsystem with the given speeds.
     *
     * @param speeds The desired chassis speeds.
     * @param fieldRelative True for if the speeds are field-relative or false for robot-relative.
     * @param isOpenLoop True to use open-loop control, or false for closed loop control of the swerve modules.
     */
    public void drive(ChassisSpeeds speeds, boolean fieldRelative, boolean isOpenLoop) {
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading());
        }

        speeds = ChassisSpeeds.discretize(speeds, SwerveConfig.discretizePeriodSecs);

        // Heading correction to hold the heading stable when we don't want it to rotate
        // Originally made by Team 1466 Webb Robotics.
        // Modified by Team 7525 Pioneers and BoiledBurntBagel of 6036
        if (SwerveConfig.enableHeadingCorrection) {
            boolean dontWantToRotate =
                    Math.abs(speeds.omegaRadiansPerSecond) < SwerveConfig.headingCorrectionDeadband;
            boolean wantToMove =
                    Math.abs(speeds.vxMetersPerSecond) > SwerveConfig.headingCorrectionDeadband
                            || Math.abs(speeds.vyMetersPerSecond)
                                    > SwerveConfig.headingCorrectionDeadband;
            boolean notRotating =
                    getRobotRelativeSpeeds().omegaRadiansPerSecond
                            < SwerveConfig.headingCorrectionRotatingDeadband;
            if (dontWantToRotate && wantToMove && notRotating) {
                if (getRobotRelativeSpeeds().omegaRadiansPerSecond
                        < SwerveConfig.headingCorrectionDeadband) {
                    speeds.omegaRadiansPerSecond = 0;
                } else {
                    speeds.omegaRadiansPerSecond =
                            m_holdHeadingPid.calculate(
                                    SwerveSubsystem.getInstance().getHeading().getRadians(),
                                    lastHeadingRadians);
                }
            } else {
                lastHeadingRadians = getHeading().getRadians();
            }
        }

        // Use the SwerveSetpointGenerator to limit module states to only what is physically achievable
        if (SwerveConfig.enableSwerveSetpointGenerator) {
            m_currentSetpoint =
                    m_setpointGenerator.generateSetpoint(
                            m_moduleLimits.setpoint,
                            m_currentSetpoint,
                            speeds,
                            GeneralConfig.loopPeriodSecs);
        } else {
            m_currentSetpoint =
                    new SwerveSetpoint(
                            speeds, SwerveConfig.swerveKinematics.toSwerveModuleStates(speeds));
        }

        pubVelCmdX.accept(m_currentSetpoint.chassisSpeeds.vxMetersPerSecond);
        pubVelCmdY.accept(m_currentSetpoint.chassisSpeeds.vyMetersPerSecond);
        pubVelCmdRot.accept(Math.toDegrees(m_currentSetpoint.chassisSpeeds.omegaRadiansPerSecond));
        pubDesiredStates.accept(
                AdvantageUtil.deconstructSwerveModuleState(m_currentSetpoint.moduleStates));

        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].setDesiredState(m_currentSetpoint.moduleStates[i], isOpenLoop);
        }
    }

    /**
     * Sets the desired states for the swerve modules. Runs SwerveSetpointGenerator to enforce a viable
     *
     * @param desiredStates An array of SwerveModuleState objects representing the desired states for each module.
     * @param isOpenLoop    A boolean indicating whether the control mode is open loop or not.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Config.SwerveConfig.maxSpeed);
        pubDesiredStates.accept(AdvantageUtil.deconstructSwerveModuleState(desiredStates));

        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].setDesiredState(desiredStates[i], isOpenLoop);
        }
    }

    /**
     * Sets the drive voltage and steering angle for characterization on all swerve modules.
     *
     * @param driveVolts    The drive voltage to set for all swerve modules.
     */
    public void setVoltsForCharacterization(double driveVolts) {
        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].setVolts(driveVolts, new Rotation2d());
        }
    }

    /**
     * Drives the robot with the given robot-relative speeds.
     * This specific parameter set is required by PathPlanner.
     *
     * @param robotRelativeSpeeds the robot-relative speeds to drive the robot with
     */
    public void drivePathPlanner(ChassisSpeeds robotRelativeSpeeds) {
        drive(robotRelativeSpeeds, false, false);
    }

    /**
     * Returns the current pose of the robot
     *
     * @return the current pose as a Pose2d object
     */
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the odometry of the swerve subsystem to the specified pose.
     *
     * @param pose The desired pose to set the odometry to.
     */
    public void resetOdometry(Pose2d pose) {
        m_poseEstimator.resetPosition(getYaw(), getPositions(), pose);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
     *     in meters, y position in meters, and heading in radians). Increase these numbers to trust
     *     the vision pose measurement less.
     */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        m_poseEstimator.addVisionMeasurement(
                visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /**
     * Retrieves the current state of all swerve modules.
     *
     * @return an array of SwerveModuleState objects representing the state of each swerve module
     */
    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < m_modules.length; i++) {
            states[i] = m_modules[i].getState();
        }

        return states;
    }

    /**
     * Retrieves the positions of all swerve modules in the swerve subsystem.
     *
     * @return an array of SwerveModulePosition objects representing the
     *         positions of the swerve modules.
     */
    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < m_modules.length; i++) {
            positions[i] = m_modules[i].getPosition();
        }

        return positions;
    }

    /**
     * Returns the yaw rotation in the form of a Rotation2d object.
     *
     * This returns directly from the gyro which is not correct for the field.
     * Use getHeading() to get a heading corrected for the field.
     *
     * @return The yaw rotation as a Rotation2d object.
     */
    private Rotation2d getYaw() {
        if (RobotID.getActiveID().equals(RobotID.SIMULATION)) {
            return m_simHeading;
        }
        return m_gyro.getRotation2d();
    }

    /**`
     * Returns a command to set the given angle as the heading.
     * Rotates the angle by 180 degrees if on the red alliance.
     *
     * @param angle to set for the blue alliance
     * @return Command to reset the heading
     */
    public Command setHeadingCommand(Rotation2d angle) {
        return Commands.runOnce(
                () ->
                        resetOdometry(
                                new Pose2d(
                                        getPose().getTranslation(),
                                        GeomUtil.rotateForAlliance(angle))));
    }

    /**
     * Creates a command that sets the odometry to the specified pose.
     *
     * @param pose The desired pose to set the odometry to.
     * @return The command that sets the odometry.
     */
    public Command setOdometryCommand(Pose2d pose) {
        return Commands.runOnce(() -> resetOdometry(pose));
    }

    /**
     * Returns a Command object that sets the wheels of the swerve subsystem to a locked position in an X pattern.
     *
     * @return The Command object that sets the wheels to a locked position in an X pattern.
     */
    public Command setLockWheelsInXCommand() {
        return run(
                () ->
                        setModuleStates(
                                new SwerveModuleState[] {
                                    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                                    new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                                    new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                                    new SwerveModuleState(0, Rotation2d.fromDegrees(45))
                                },
                                true));
    }

    /**
     * Returns a Command object that drives the robot to the desired pose.
     *
     * @param desiredPose The desired pose for the robot to drive to.
     * @param flipForAlliance True to flip the pose for the red alliance, false otherwise.
     * @return A Command object that drives the robot to the desired pose.
     */
    public Command getDriveToPoseCommand(Pose2d desiredPose, boolean flipForAlliance) {
        return runOnce(() -> resetDriveToPose())
                .andThen(run(() -> driveToPose(desiredPose, flipForAlliance)));
    }

    /**
     * Resets the drive to pose by setting the desired pose to null and resetting the PID controllers.
     * This ensures that isAtPose returns false until driveToPose is called again.
     */
    public void resetDriveToPose() {
        // Set desired pose to null so that isAtPose returns false until driveToPose is called
        m_desiredPose = null;

        ChassisSpeeds speeds = getFieldRelativeSpeeds();
        m_pidX.reset(getPose().getX(), speeds.vxMetersPerSecond);
        m_pidY.reset(getPose().getY(), speeds.vyMetersPerSecond);
        m_pidRot.reset(getHeading().getRadians(), speeds.omegaRadiansPerSecond);
    }

    /**
     * Calculate the pid value for rotating the chassis to the desired angle
     *
     * @param desiredAngle Desired angle for the chassis
     * @return The pid value to pass to rotation in the drive method
     */
    public double calculateRotation(Rotation2d desiredAngle) {
        return m_pidRot.calculate(
                SwerveSubsystem.getInstance().getHeading().getRadians(), desiredAngle.getRadians());
    }

    /**
     * Drives the robot to a specified pose.
     *
     * @param pose The desired pose to drive to.
     * @param flipForAlliance True to flip the pose for the red alliance, false otherwise.
     */
    public void driveToPose(Pose2d pose, boolean flipForAlliance) {
        if (flipForAlliance) {
            pose = AllianceFlipUtil.apply(pose);
        }

        double xSpeed = 0;
        double ySpeed = 0;
        double rotSpeed = 0;

        if (Math.abs(getPose().getX() - pose.getX()) > SwerveConfig.translationAllowableError) {
            xSpeed = m_pidX.calculate(getPose().getX(), pose.getX());
            xSpeed += m_pidX.getSetpoint().velocity;
        }

        if (Math.abs(getPose().getY() - pose.getY()) > SwerveConfig.translationAllowableError) {
            ySpeed = m_pidY.calculate(getPose().getX(), pose.getY());
            ySpeed += m_pidY.getSetpoint().velocity;
        }

        double angleError = getHeading().getRadians() - pose.getRotation().getRadians();
        if (Math.abs(angleError) > SwerveConfig.rotationAllowableError) {
            rotSpeed =
                    m_pidRot.calculate(getHeading().getRadians(), pose.getRotation().getRadians());
        }

        m_desiredPose = pose;
        pubDesiredX.accept(pose.getX());
        pubDesiredY.accept(pose.getY());
        pubDesiredRot.accept(pose.getRotation().getDegrees());

        pubVelSetpointX.accept(m_pidX.getSetpoint().velocity);
        pubVelSetpointY.accept(m_pidY.getSetpoint().velocity);
        pubVelSetpointRot.accept(Math.toDegrees(m_pidRot.getSetpoint().velocity));

        drive(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed), true, false);
    }

    /**
     * Checks if the robot is at the desired pose within a given tolerance.
     *
     * @param tightTolerance True for a tight tolerance, false for a loose tolerance
     */
    public boolean isAtPose(boolean tightTolerance) {
        double posTol, angleTol;
        if (tightTolerance) {
            posTol = SwerveConfig.posTolerance;
            angleTol = SwerveConfig.angleTolerance;
        } else {
            posTol = SwerveConfig.loosePosTolerance;
            angleTol = SwerveConfig.looseAngleTolerance;
        }

        double angleError = getHeading().getRadians() - m_desiredPose.getRotation().getRadians();
        ChassisSpeeds speeds = getFieldRelativeSpeeds();

        return m_desiredPose != null // If we haven't set a pose, we can't be at it
                && Math.abs(getPose().getX() - m_desiredPose.getX()) < posTol
                && Math.abs(getPose().getY() - m_desiredPose.getY()) < posTol
                && Math.abs(MathUtil.angleModulus(angleError)) < angleTol
                && Math.abs(speeds.vxMetersPerSecond) < SwerveConfig.velTolerance
                && Math.abs(speeds.vxMetersPerSecond) < SwerveConfig.velTolerance
                && Math.abs(speeds.omegaRadiansPerSecond) < SwerveConfig.angleVelTolerance;
    }

    /**
     * Get a pose at the given timestamp.
     * Returns an empty Optional if the buffer is empty or doesn't go back far enough.
     *
     * @param timestampSeconds The timestamp for the pose to get, matching WPILib PoseEstimator's
     *                         timestamps (which matches PhotonVision and Limelight)
     * @return An Optional of the Pose2d or an empty Optional.
     */
    public Optional<Pose2d> getPoseAtTimestamp(double timestampSeconds) {
        return m_poseBuffer.getPoseAtTimestamp(timestampSeconds);
    }

    @Override
    public void periodic() {
        // Get current speeds and positions
        ChassisSpeeds fieldSpeeds = getFieldRelativeSpeeds();
        SwerveModulePosition[] positions = getPositions();
        SwerveModuleState[] states = getStates();

        // Simulate the heading if we are in simulation mode
        if (RobotID.getActiveID().equals(RobotID.SIMULATION)) {
            double addRadians = fieldSpeeds.omegaRadiansPerSecond * GeneralConfig.loopPeriodSecs;
            m_simHeading = m_simHeading.plus(new Rotation2d(addRadians));
        }

        // Update the pose estimator with encoder positions and gyro angle
        m_poseEstimator.update(getYaw(), positions);
        m_poseBuffer.addPoseToBuffer(getPose());

        // Run periodic on all swerve modules
        for (SwerveModuleAbstract mod : m_modules) {
            mod.periodic();
        }

        // Check if the swerve modules are synchronized and sync them if they are not
        if (DriverStation.isDisabled() && !isChassisMoving(0.01) && !areModulesRotating(2)) {
            if (++modSyncCounter > 6 && isSwerveNotSynched()) {
                // synchSwerve();
                pubNumSyncs.accept(pubNumSyncs.get() + 1);
                modSyncCounter = 0;
            }
        } else {
            modSyncCounter = 0;
        }

        // Check if tunable values have changed and update them if so
        SwerveModuleAbstract.updateTunableModuleConstants();
        TunableDouble.ifChanged(
                hashCode(), () -> m_holdHeadingPid.setP(tunableHeadingkP.get()), tunableHeadingkP);
        tunableXPid.checkForUpdates();
        tunableYPid.checkForUpdates();
        tunableRotPid.checkForUpdates();

        // Update NetworkTables
        pubPoseRot.accept(getPose().getRotation().getDegrees());
        pubPoseX.accept(getPose().getX());
        pubPoseY.accept(getPose().getY());
        pubEstimatedPose.accept(AdvantageUtil.deconstruct(getPose()));

        pubMeasuredStates.accept(AdvantageUtil.deconstructSwerveModuleState(states));
        pubGyroRate.accept(getAngularRate());

        pubMeasVelX.accept(fieldSpeeds.vxMetersPerSecond);
        pubMeasVelY.accept(fieldSpeeds.vyMetersPerSecond);
        pubMeasVelRot.accept(Math.toDegrees(fieldSpeeds.omegaRadiansPerSecond));
    }

    /**
     * Returns the robot-relative speeds of the chassis.
     *
     * @return The robot-relative speeds of the chassis.
     */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return SwerveConfig.swerveKinematics.toChassisSpeeds(getStates());
    }

    /**
     * Returns the heading of the swerve subsystem. This is corrected for the field.
     * Heading of zero is away from the blue alliance driver stations.
     *
     * @return the rotation2d representing the heading of the swerve subsystem
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Returns the angular rate of the swerve subsystem.
     *
     * @return The angular rate in degrees per second.
     */
    public double getAngularRate() {
        double[] xyz_dps = new double[3];
        m_gyro.getRawGyro(xyz_dps);
        return xyz_dps[2];
    }

    /**
     * Returns the field-relative speeds of the chassis.
     *
     * @return The field-relative speeds of the chassis.
     */
    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(), getHeading());
    }

    /**
     * Checks if the chassis is currently moving based on the velocity tolerance.
     *
     * @param velToleranceMPS the velocity tolerance in meters per second
     * @return true if the chassis is moving, false otherwise
     */
    public boolean isChassisMoving(double velToleranceMPS) {
        double sumVelocity = 0;
        for (SwerveModuleAbstract mod : m_modules) {
            sumVelocity += Math.abs(mod.getState().speedMetersPerSecond);
        }

        if (sumVelocity <= velToleranceMPS) {
            return false;
        } else {
            return true;
        }
    }

    /**
     * Checks if the swerve modules are currently rotating.
     *
     * @param toleranceDegPerSec the tolerance for angular velocity in degrees per second
     * @return true if any of the swerve modules are rotating, false otherwise
     */
    public boolean areModulesRotating(double toleranceDegPerSec) {
        double angularVelocitySum = 0;
        for (int i = 0; i < m_modules.length; i++) {
            angularVelocitySum += m_modules[i].getSteeringVelocity();
        }

        return angularVelocitySum > Math.toRadians(toleranceDegPerSec);
    }

    /**
     * Checks if any of the swerve modules in the swerve subsystem are not synchronized.
     *
     * @return true if any swerve module is not synchronized, false otherwise
     */
    public boolean isSwerveNotSynched() {
        for (SwerveModuleAbstract module : m_modules) {
            if (!module.isModuleSynced()) {
                return true;
            }
        }
        return false;
    }

    /**
     * Synchronizes all swerve modules by resetting them to their absolute position.
     */
    public void synchSwerve() {
        for (SwerveModuleAbstract module : m_modules) {
            module.resetToAbsolute();
        }
    }

    /**
     * Stops all the motors in the swerve subsystem.
     */
    public void stopMotors() {
        for (SwerveModuleAbstract mod : m_modules) {
            mod.stopMotors();
        }
    }
}
