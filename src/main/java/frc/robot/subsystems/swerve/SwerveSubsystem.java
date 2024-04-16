package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.lib2706.AdvantageUtil;
import frc.lib.lib2706.TunableSimpleMotorFeedforward;
import frc.lib.lib2706.swerve.PoseBuffer;
import frc.robot.Config;
import frc.robot.Config.CANID;
import frc.robot.Config.PhotonConfig;
import frc.robot.Config.SwerveConfig;

import java.util.Optional;

public class SwerveSubsystem extends SubsystemBase {
    private final PigeonIMU gyro;

    private SwerveDriveOdometry swerveOdometry;
    private SwerveModule[] mSwerveMods;
    String tableName = "SwerveChassis";
    private NetworkTable swerveTable = NetworkTableInstance.getDefault().getTable(tableName);
    private DoublePublisher pubCurrentAngle =
            swerveTable.getDoubleTopic("Current angle (deg)").publish(PubSubOption.periodic(0.02));
    private DoublePublisher pubCurrentPositionX =
            swerveTable
                    .getDoubleTopic("Current positionX (m) ")
                    .publish(PubSubOption.periodic(0.02));
    private DoublePublisher pubCurrentPositionY =
            swerveTable
                    .getDoubleTopic("Current positionY (m) ")
                    .publish(PubSubOption.periodic(0.02));
    private DoubleArrayPublisher pubCurrentPose =
            swerveTable.getDoubleArrayTopic("Pose ").publish(PubSubOption.periodic(0.02));
    private TunableSimpleMotorFeedforward tunableSimpleFF;
    private DoublePublisher pubGyroRate =
            swerveTable.getDoubleTopic("Gyro Rate (degps)").publish(PubSubOption.periodic(0.02));

    private NetworkTable visionPidTable = swerveTable.getSubTable("VisionPid");
    private DoublePublisher pubMeasuredSpeedX =
            visionPidTable
                    .getDoubleTopic("MeasuredSpeedX (mps)")
                    .publish(PubSubOption.periodic(0.02));
    private DoublePublisher pubMeasuredSpeedY =
            visionPidTable
                    .getDoubleTopic("MeasuredSpeedY (mps)")
                    .publish(PubSubOption.periodic(0.02));
    private DoublePublisher pubMeasuredSpeedRot =
            visionPidTable
                    .getDoubleTopic("MeasuredSpeedRot (radps)")
                    .publish(PubSubOption.periodic(0.02));
    private DoublePublisher pubDesiredX =
            visionPidTable.getDoubleTopic("DesiredX (m)").publish(PubSubOption.periodic(0.02));
    private DoublePublisher pubDesiredY =
            visionPidTable.getDoubleTopic("DesiredY (m)").publish(PubSubOption.periodic(0.02));
    private DoublePublisher pubDesiredRot =
            visionPidTable.getDoubleTopic("DesiredRot (deg)").publish(PubSubOption.periodic(0.02));

    // ProfiledPIDControllers for the pid control
    ProfiledPIDController pidControlX;
    double currentX;
    double desiredX;
    ProfiledPIDController pidControlY;
    double currentY;
    double desiredY;
    ProfiledPIDController pidControlRotation;
    double currentRotation;
    double desiredRotation;
    int tempSynchCounter = 0;
    boolean recievedPidInstruction = false;

    /**
     * Counter to synchronize the modules relative encoder with absolute encoder when not moving.
     */
    private int moduleSynchronizationCounter = 0;

    private Field2d field;

    private PoseBuffer poseBuffer;

    private static SwerveSubsystem instance;

    public static SwerveSubsystem getInstance() {
        if (instance == null) {
            instance = new SwerveSubsystem();
        }
        return instance;
    }

    private SwerveSubsystem() {
        gyro = new PigeonIMU(CANID.PIGEON.val());
        gyro.configFactoryDefault();

        mSwerveMods =
                new SwerveModule[] {
                    new SwerveModule(Config.SwerveConfig.Mod0.constants, "FL"),
                    new SwerveModule(Config.SwerveConfig.Mod1.constants, "FR"),
                    new SwerveModule(Config.SwerveConfig.Mod2.constants, "BL"),
                    new SwerveModule(Config.SwerveConfig.Mod3.constants, "BR")
                };

        swerveOdometry =
                new SwerveDriveOdometry(
                        Config.SwerveConfig.swerveKinematics,
                        getYaw(),
                        getPositions(),
                        new Pose2d());

        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto
                // has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT
                // RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely
                        // live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants kP = 5.0
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants kP = 5.0
                        3.0, // Max module speed, in m/s
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

        poseBuffer = new PoseBuffer();

        field = new Field2d();
        SmartDashboard.putData("Field", field);

        pidControlX =
                new ProfiledPIDController(9, 0.5, 0.2, new TrapezoidProfile.Constraints(2.5, 4.5));
        pidControlY =
                new ProfiledPIDController(9, 0.5, 0.2, new TrapezoidProfile.Constraints(2.5, 4.5));
        pidControlRotation =
                new ProfiledPIDController(
                        5.0, 0.5, 0.3, new TrapezoidProfile.Constraints(8 * Math.PI, 8 * Math.PI));
        pidControlRotation.enableContinuousInput(-Math.PI, Math.PI);

        pidControlX.setIZone(0.3);
        pidControlY.setIZone(0.3);
        pidControlRotation.setIZone(Math.toRadians(3));

        tunableSimpleFF =
                new TunableSimpleMotorFeedforward(
                        (ff) -> updateModuleFeedforward(ff),
                        swerveTable,
                        Config.SwerveConfig.driveKS,
                        Config.SwerveConfig.driveKV,
                        Config.SwerveConfig.driveKA);
    }

    public void drive(
            ChassisSpeeds speeds,
            boolean fieldRelative,
            boolean isOpenLoop,
            boolean flipForAlliance) {
        Rotation2d heading = flipForAlliance ? rotateForAlliance(getHeading()) : getHeading();

        SwerveModuleState[] swerveModuleStates =
                Config.SwerveConfig.swerveKinematics.toSwerveModuleStates(
                        // ChassisSpeeds.discretize(
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds, heading)
                                : speeds
                        // , 0.02)
                        );
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, Config.SwerveConfig.maxSpeed);

        for (int i = 0; i < mSwerveMods.length; i++) {
            mSwerveMods[i].setDesiredState(swerveModuleStates[i], isOpenLoop);
        }
    }

    public void drive(ChassisSpeeds speeds, boolean fieldRelative, boolean isOpenLoop) {
        drive(speeds, fieldRelative, isOpenLoop, true);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        setModuleStates(desiredStates, isOpenLoop, false);
    }

    public void setModuleStates(
            SwerveModuleState[] desiredStates, boolean isOpenLoop, boolean isDisableAntiJitter) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Config.SwerveConfig.maxSpeed);

        for (int i = 0; i < mSwerveMods.length; i++) {
            mSwerveMods[i].setDesiredState(desiredStates[i], isOpenLoop);
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < mSwerveMods.length; i++) {
            states[i] = mSwerveMods[i].getState();
        }

        return states;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < mSwerveMods.length; i++) {
            positions[i] = mSwerveMods[i].getPosition();
        }

        return positions;
    }

    public void updateModuleFeedforward(SimpleMotorFeedforward ff) {
        for (SwerveModule mod : mSwerveMods) {
            mod.setFeedforward(ff);
        }
    }

    private Rotation2d getYaw() {
        return (Config.SwerveConfig.invertGyro)
                ? Rotation2d.fromDegrees(360 - gyro.getYaw())
                : Rotation2d.fromDegrees(gyro.getYaw());
    }

    /**
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
                                new Pose2d(getPose().getTranslation(), rotateForAlliance(angle))));
    }

    public Command setOdometryCommand(Pose2d pose) {
        return Commands.runOnce(() -> resetOdometry(pose));
    }

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
                                true,
                                true));
    }

    public Command getDriveToPoseCommand(Pose2d desiredPose) {
        return runOnce(() -> resetDriveToPose())
                .andThen(run(() -> driveToPose(desiredPose)))
                .until(() -> isAtPose(PhotonConfig.POS_TOLERANCE, PhotonConfig.ANGLE_TOLERANCE));
    }

    // Swerve actual driving methods
    public void resetDriveToPose() {
        recievedPidInstruction = false;

        currentX = getPose().getX();
        currentY = getPose().getY();
        currentRotation = getPose().getRotation().getRadians();

        desiredX = getPose().getX();
        desiredY = getPose().getY();
        desiredRotation = getPose().getRotation().getRadians();

        ChassisSpeeds speeds = getFieldRelativeSpeeds();
        pidControlX.reset(getPose().getX(), speeds.vxMetersPerSecond);
        pidControlY.reset(getPose().getY(), speeds.vyMetersPerSecond);
        pidControlRotation.reset(
                getPose().getRotation().getRadians(), speeds.omegaRadiansPerSecond);
    }

    /**
     * Calculate the pid value for rotating the chassis to the desired angle
     *
     * @param desiredAngle Desired angle for the chassis
     * @return The pid value to pass to rotation in the drive method
     */
    public double calculateRotation(Rotation2d desiredAngle) {
        return pidControlRotation.calculate(
                SwerveSubsystem.getInstance().getHeading().getRadians(), desiredAngle.getRadians());
    }

    public void driveToPose(Pose2d pose) {
        // update the currentX and currentY

        currentX = getPose().getX();
        currentY = getPose().getY();
        currentRotation = getPose().getRotation().getRadians();

        desiredX = pose.getX();
        desiredY = pose.getY();
        desiredRotation = pose.getRotation().getRadians();

        double xSpeed = 0;
        double ySpeed = 0;
        double rotSpeed = 0;

        if (Math.abs(currentX - desiredX) > SwerveConfig.translationAllowableError) {
            xSpeed = pidControlX.calculate(currentX, desiredX);
        }

        if (Math.abs(currentY - desiredY) > SwerveConfig.translationAllowableError) {
            ySpeed = pidControlY.calculate(currentY, desiredY);
        }

        if (Math.abs(currentRotation - desiredRotation) > SwerveConfig.rotationAllowableError) {
            rotSpeed = pidControlRotation.calculate(currentRotation, desiredRotation);
        }

        pubDesiredX.accept(pidControlX.getSetpoint().position);
        pubDesiredY.accept(pidControlY.getSetpoint().position);
        pubDesiredRot.accept(Math.toDegrees(pidControlRotation.getSetpoint().position));

        recievedPidInstruction = true;
        drive(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed), true, true, false);
    }

    public boolean isAtPose(double tol, double angleTol) {
        return recievedPidInstruction
                && Math.abs(currentX - desiredX) < tol
                && Math.abs(currentY - desiredY) < tol
                && Math.abs(MathUtil.angleModulus(currentRotation - desiredRotation)) < angleTol;
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
        return (poseBuffer.getPoseAtTimestamp(timestampSeconds));
    }

    @Override
    public void periodic() {
        for (SwerveModule mod : mSwerveMods) {
            mod.periodic();
        }
        SwerveModulePosition[] tempGetPositions = getPositions();
        SwerveModuleState[] tempGetStates = getStates();
        swerveOdometry.update(getYaw(), tempGetPositions);
        field.setRobotPose(getPose());

        // If the robot isn't moving synchronize the encoders every 100ms (Inspired by democrat's
        // SDS
        // lib)
        // To ensure that everytime we initialize it works.
        if (DriverStation.isDisabled() && !isChassisMoving(0.01) && !areModulesRotating(2)) {
            if (++moduleSynchronizationCounter > 6 && isSwerveNotSynched()) {
                synchSwerve();
                System.out.println("Resynced" + ++tempSynchCounter);
                moduleSynchronizationCounter = 0;
            }
        } else {
            moduleSynchronizationCounter = 0;
        }

        poseBuffer.addPoseToBuffer(getPose());

        // Check if the feedforward has been updated on NT
        tunableSimpleFF.updateFeedforward();

        pubCurrentAngle.accept(getPose().getRotation().getDegrees());
        pubCurrentPositionX.accept(getPose().getX());
        pubCurrentPositionY.accept(getPose().getY());
        pubCurrentPose.accept(AdvantageUtil.deconstruct(getPose()));

        pubGyroRate.accept(getAngularRate());

        ChassisSpeeds speeds = getFieldRelativeSpeeds();
        pubMeasuredSpeedX.accept(speeds.vxMetersPerSecond);
        pubMeasuredSpeedY.accept(speeds.vyMetersPerSecond);
        pubMeasuredSpeedRot.accept(Math.toDegrees(speeds.omegaRadiansPerSecond));
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        // ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
        ChassisSpeeds targetSpeeds = robotRelativeSpeeds;

        SwerveModuleState[] targetStates =
                Config.SwerveConfig.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates, false);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Config.SwerveConfig.swerveKinematics.toChassisSpeeds(getStates());
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public double getAngularRate() {
        double[] xyz_dps = new double[3];
        gyro.getRawGyro(xyz_dps);
        return xyz_dps[2];
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(), getHeading());
    }

    public boolean isChassisMoving(double velToleranceMPS) {
        double sumVelocity = 0;
        for (SwerveModule mod : mSwerveMods) {
            sumVelocity += Math.abs(mod.getState().speedMetersPerSecond);
        }

        if (sumVelocity <= velToleranceMPS) {
            return false;
        } else {
            return true;
        }
    }

    public boolean areModulesRotating(double angleTolerance) {
        double angularVelocitySum = 0;
        for (int i = 0; i < mSwerveMods.length; i++) {
            angularVelocitySum += mSwerveMods[i].getSteeringVelocity();
        }

        return angularVelocitySum > Math.toRadians(angleTolerance);
    }

    public boolean isSwerveNotSynched() {
        for (SwerveModule module : mSwerveMods) {
            if (!module.isModuleSynced()) {
                return (true);
            }
        }
        return (false);
    }

    public void synchSwerve() {
        for (SwerveModule module : mSwerveMods) {
            module.resetToAbsolute();
        }
    }

    public void stopMotors() {
        for (SwerveModule mod : mSwerveMods) {
            mod.stopMotors();
        }
    }

    /**
     * Rotates the given angle by 180 if the red alliance or 0 if blue and returns it.
     * Aka defaults to assuming we are on the blue alliance.
     *
     * @param angle to rotate.
     * @return The angle rotated for the alliance.
     */
    public static Rotation2d rotateForAlliance(Rotation2d angle) {
        var alliance = DriverStation.getAlliance();

        // Default to blue alliance
        if (alliance.isEmpty()) {
            DriverStation.reportWarning("Unable to detect alliance color.", false);
            return angle;
        }

        if (alliance.get() == DriverStation.Alliance.Blue) {
            return angle;
        } else {
            return angle.rotateBy(new Rotation2d(Math.PI));
        }
    }
}
