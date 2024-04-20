package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

import frc.lib.lib254.SwerveModuleLimits;
import frc.lib.lib2706.swerve.SwerveModuleConstants;

import java.io.BufferedReader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public final class Config {
    /**
     * Instructions for set up of robot.conf file on robot
     *
     * 0. Connect to the robot to the robot using a usb cable or the wifi network.
     * 1. Using a tool like Git Bash or putty, ssh into admin@roboRIO-2706-FRC.local
     * (ssh admin@roboRIO-2706-FRC.local)
     * a. There is no password on a freshly flashed roboRIO
     * 2. Go up a directory (cd ..)
     * 3. cd into lvuser/ (cd lvuser/)
     * 4. Create a new file called robot.conf (touch robot.conf)
     * 5. Open the file with vi (vi robot.conf)
     * 6. Press i to enter insert mode
     * 7. Add an integer denoting the robot id. If it's the first robot, use 0,
     * second use 1 etc.
     * 8. Press [ESC] followed by typing :wq in order to save and quit
     * 9. To verify this worked type: more robot.conf
     * 10. If it displays the value you entered, it was successful
     * 11. Type exit to safely exit the ssh session
     */
    private static final Path ROBOT_ID_LOC =
            Paths.get(System.getProperty("user.home"), "robot.conf");

    /**
     * Enum representing the different robot IDs.
     * RobotID integers must be array indexes.
     */
    public enum RobotID {
        APOLLO(0), // Crescendo, Apollo
        SIMULATION(1), // Simulated Apollo
        BEETLE(2), // Beetle
        POSEIDON(3); // Poseidon, Charged Up

        private final int id;

        RobotID(int id) {
            this.id = id;
        }

        /**
         * Get the robot id.
         * @return An array index representing the robot id.
         */
        public int getId() {
            return id;
        }

        /** ID of the robot that code is running on */
        private static RobotID activeRobot = null;

        /** Fallback ID used if the robot id code ever fails */
        private static final RobotID FALLBACK_ID = APOLLO;

        /**
         * Get the active robot id. If the robot id is not found, it will default to the fallbackID.
         *
         * @return Active robot id
         */
        public static RobotID getActiveID() {
            if (activeRobot != null) {
                return activeRobot;
            }

            if (DriverStation.isFMSAttached()) {
                // Backup in case the FMS is attached, force to comp robot
                activeRobot = APOLLO;
            } else if (RobotBase.isSimulation()) {
                activeRobot = SIMULATION;
            } else {
                // Read the robot.conf file on the roborio to get it's robot id.
                try (BufferedReader reader = Files.newBufferedReader(ROBOT_ID_LOC)) {
                    int robotID = Integer.parseInt(reader.readLine());
                    for (RobotID robot : RobotID.values()) {
                        if (robot.getId() == robotID) {
                            activeRobot = robot;
                            return activeRobot;
                        }
                    }
                    activeRobot = FALLBACK_ID;
                    DriverStation.reportError(
                            "Integer in robot.conf is not mapped to a robot id. Falling back to "
                                    + activeRobot.name(),
                            false);
                } catch (Exception e) {
                    activeRobot = FALLBACK_ID;
                    DriverStation.reportError(
                            "Could not read robot.conf for a robot id. Falling back to "
                                    + activeRobot.name(),
                            false);
                }
            }
            return activeRobot;
        }
    }

    /**
     * Returns one of the values passed based on the robot ID
     *
     * @param first The first value (default value)
     * @param more  Other values that could be selected
     * @param <T>   The type of the value
     * @return The value selected based on the ID of the robot
     */
    @SafeVarargs
    public static <T> T robotSpecific(T first, T... more) {
        if (RobotID.getActiveID().getId() < 1 || RobotID.getActiveID().getId() > more.length) {
            return first;
        } else {
            return more[RobotID.getActiveID().getId() - 1];
        }
    }

    public enum CANID {
        PIGEON(robotSpecific(16, 16, 27, 30)),
        CANDLE(robotSpecific(25, 25, 15, 15)),
        CLIMBER(robotSpecific(18, 18, 5, -1)),

        // swerve CAN IDs
        SWERVE_FL_DRIVE(4),
        SWERVE_FL_STEERING(5),
        SWERVE_FR_DRIVE(6),
        SWERVE_FR_STEERING(7),
        SWERVE_RL_DRIVE(8),
        SWERVE_RL_STEERING(9),
        SWERVE_RR_DRIVE(10),
        SWERVE_RR_STEERING(11),
        SWERVE_FL_CANCODER(12),
        SWERVE_FR_CANCODER(13),
        SWERVE_RL_CANCODER(14),
        SWERVE_RR_CANCODER(15),

        // mechanism CAN IDs
        ARM(19),
        INTAKE(21),
        SHOOTER(22);

        private final int id;

        CANID(int id) {
            this.id = id;
        }

        public int val() {
            return id;
        }

        public static Map<Integer, String> mapCanIdsToNames() {
            HashMap<Integer, String> map = new HashMap<>();
            for (CANID canid : CANID.values()) {
                map.put(canid.val(), canid.name());
            }
            return map;
        }

        public static final int CANTIMEOUT_MS = 100;
    }

    public static final class GeneralConfig {
        public static final boolean enableTunableData = false;
        public static final double joystickDeadband = 0.1;

        public static final int revMaxRetries = 5;
        public static final int ctreMaxRetries = 5;

        public static final double loopPeriodSecs = 0.02;
    }

    public static final boolean disableStateBasedProgramming =
            true; // True to disable state based programming and use only simple commands

    public static final class RioConfig {
        public static int ANALOG_SELECTOR_PORT = robotSpecific(3, 3, -1, 0);
    }

    public static final class PhotonConfig {
        public static boolean USE_3D_TAGS = true;
        public static final List<Integer> ALLOWED_TAGS_3D = List.of(3, 4, 7, 8);

        public static final double CAMERA_HEIGHT = 0.215;
        public static final Rotation2d CAMERA_PITCH = Rotation2d.fromDegrees(33);
        // x is forwards, y is sideways with +y being left, rotation probobly if + left too
        public static final Pose2d cameraOffset =
                new Pose2d(new Translation2d(-0.1, 0), Rotation2d.fromDegrees(180));
        // public static final Pose2d cameraOffsetRed = new Pose2d(new Translation2d(-0.1, 0),
        // Rotation2d.fromDegrees(0));

        public static final Transform3d cameraTransform =
                new Transform3d(
                        -(0.865 / 2 - 0.095),
                        0,
                        0.23,
                        new Rotation3d(0, Math.toRadians(-33), Math.toRadians(180)));

        // networkTableName
        public static final String apriltagCameraName = "FrontApriltagOV9281";
        public static final String networkTableName = "PhotonCamera";
        public static final String frontCameraName = "HD_USB_CAMERA";
        // data max
        public static final int maxNumSamples = 10;

        // these are the heights for the apriltags 3, 4, 5, 6, 7, 8
        public static final double[] APRIL_HEIGHTS = {1.32, 1.32, 1.22, 1.22, 1.32, 1.32};
        public static final double POS_TOLERANCE = 0.01; // meters
        public static final double ANGLE_TOLERANCE = Math.toRadians(1.0);
        public static final double WAYPOINT_POS_TOLERANCE = 0.2; // meters
        public static final double WAYPOINT_ANGLE_TOLERANCE = Math.toRadians(10.0);
        public static final double VEL_TOLERANCE = 0.1 * 4;

        public static enum PhotonPositions {
            RIGHT_SPEAKER_RED(
                    4,
                    new Translation2d(-0.937, 0.937),
                    new Translation2d(-0.637, 0.637),
                    Rotation2d.fromDegrees(-60)),
            MIDDLE_SPEAKER_RED(
                    4,
                    new Translation2d(-1.3, 0),
                    new Translation2d(-0.95, 0),
                    Rotation2d.fromDegrees(0)),
            LEFT_SPEAKER_BLUE(
                    7,
                    new Translation2d(0.937, 0.937),
                    new Translation2d(0.637, 0.637),
                    Rotation2d.fromDegrees(-120)),
            MIDDLE_SPEAKER_BLUE(
                    7,
                    new Translation2d(1.20, 0),
                    new Translation2d(0.90, 0),
                    Rotation2d.fromDegrees(180)),
            TEST(4, new Translation2d(-2, 0), new Translation2d(-1, 0), Rotation2d.fromDegrees(0)),
            AMP_RED(
                    5,
                    new Translation2d(0, -0.70),
                    new Translation2d(0, -0.5),
                    Rotation2d.fromDegrees(90)),
            AMP_BLUE(
                    6,
                    new Translation2d(0, -0.30),
                    new Translation2d(0, 0.05),
                    Rotation2d.fromDegrees(90)),

            // COMPETITION USE
            FAR_SPEAKER_RED(4, new Translation2d(-3.6, 0), Rotation2d.fromDegrees(180)),
            FAR_SPEAKER_BLUE(7, new Translation2d(3.6, 0), Rotation2d.fromDegrees(0)),

            PODIUM_SOURCESIDE_BLUE(8, new Translation2d(3.2, -1.5), Rotation2d.fromDegrees(-33)),
            PODIUM_SOURCESIDE_RED(
                    3, new Translation2d(-3.2, -1.5), Rotation2d.fromDegrees(180 + 33)),

            // NOT FULLY TESTED
            FAR_SPEAKER_RED_SIDE_TAG(
                    3,
                    new Translation2d(-2.5, 0),
                    new Translation2d(-2.1, 0.58),
                    Rotation2d.fromDegrees(0)),
            FAR_SPEAKER_BLUE_SIDE_TAG(
                    8,
                    new Translation2d(2.4, 0),
                    new Translation2d(2.1, -0.58),
                    Rotation2d.fromDegrees(0)),

            PODIUM_AMPSIDE_BLUE(7, new Translation2d(3.35, -0.65), Rotation2d.fromDegrees(-20)),
            PODIUM_AMPSIDE_RED(
                    4, new Translation2d(-3.35, -0.65), Rotation2d.fromDegrees(180 + 20)),

            RIGHT_SPEAKER_BLUE(8, new Translation2d(1, -1.1), Rotation2d.fromDegrees(-55)),
            LEFT_SPEAKER_RED(3, new Translation2d(-1, -1.1), Rotation2d.fromDegrees(180 + 55));

            // 2.2 , 33 deg
            // FAR_SPEAKER_BLUE at new Translation2d(2.35,-0.65), arm angle of 35.8 and shooter
            // speed at 3750

            public final int id;
            public final boolean hasWaypoint;
            public final Translation2d waypoint;
            public final Translation2d destination;
            public final Rotation2d direction;

            PhotonPositions(
                    int id,
                    Translation2d waypoint,
                    Translation2d destination,
                    Rotation2d direction) {
                this.id = id;
                this.hasWaypoint = true;
                this.waypoint = waypoint;
                this.destination = destination;
                this.direction = direction;
            }

            PhotonPositions(int id, Translation2d destination, Rotation2d direction) {
                this.id = id;
                this.hasWaypoint = false;
                this.waypoint = null;
                this.destination = destination;
                this.direction = direction;
            }
        }
    }

    public static final class Climber_CANID {
        public static int CLIMBER = CANID.CLIMBER.val();
    }

    public static final class SwerveConfig {
        public static final int numSwerveModules = 4;

        public static final double trackWidth = Units.inchesToMeters(25.787);
        public static final double wheelBase = Units.inchesToMeters(20.472);
        public static final double wheelDiameter = Units.inchesToMeters(3.884);
        public static final double wheelRadius = wheelDiameter / 2.0;
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double driveGearRatio = (8.14 / 1.0);
        public static final double steerGearRatio = (12.8 / 1.0);

        public static final double discretizePeriodSecs =
                GeneralConfig.loopPeriodSecs * robotSpecific(1.0, 3.0); // 0.02 * fudgeFactor
        public static final double syncMetersTol = Math.toRadians(1);
        public static final double syncMPSTol = 1;
        public static final double syncRadTol = Math.toRadians(1);

        public static final Translation2d[] moduleLocations = {
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        };

        public static final SwerveDriveKinematics swerveKinematics =
                new SwerveDriveKinematics(moduleLocations);

        /* Swerve Heading Correction */
        public static final boolean enableHeadingCorrection = true;
        public static final double headingCorrectionDeadband = 0.01;
        public static final double headingCorrectionRotatingDeadband = Math.toRadians(60);
        public static final double holdHeadingkP = 5.0;

        /* Swerve Voltage Compensation Changed */
        public static final double driveVoltComp = 12.0;
        public static final double steerVoltComp = 12.0;

        /* Swerve Current Limiting, Changed */
        public static final int steerContinuousCurrentLimit = 30; // 20
        public static final int driveContinuousCurrentLimit = 50;

        /* Steer Motor PID Values, Changed */
        public static final double steerKP = robotSpecific(2.0, 30.0);
        public static final double steerKI = robotSpecific(0.0, 0.0);
        public static final double steerKD = robotSpecific(0.1, 0.0);
        public static final double steerIZone = robotSpecific(0.0, 0.0);

        /* Drive Motor PID Values, Changed*/
        public static final double driveKP = robotSpecific(0.2, 10.0);
        public static final double driveKI = robotSpecific(0.0, 0.0);
        public static final double driveKD = robotSpecific(0.0, 0.0);
        public static final double driveKFF = robotSpecific(0.0, 0.0);
        public static final double driveIZone = robotSpecific(0.0, 0.0);

        /* Drive Motor Characterization Values Changed */
        public static final double driveKS = robotSpecific(0.667, 0.4); // Volts for static friction
        public static final double driveKV = robotSpecific(4.0, 3.1); // Volts per mps
        public static final double driveKA = robotSpecific(0.5, 0.0); // Volts per mps^2 (NOT USED)

        public static final double driveVelAllowableError = 0.001; // mps
        public static final double steerPosAllowableError = Math.toRadians(0.01);

        /* Conversion Factors */
        public static final double driveConvPosFactor = (wheelDiameter * Math.PI) / driveGearRatio;
        public static final double driveConvVelFactor = driveConvPosFactor / 60.0;
        public static final double steerConvFactor = 2 * Math.PI / steerGearRatio;
        public static final double steerVelConvFactor = steerConvFactor / 60.0;

        /* Swerve ProfiledPidController values */
        public static final double translationAllowableError = 0.01;
        public static final double rotationAllowableError = Math.toRadians(0.7);

        public static enum ModuleLimits {
            // NEO V1.0/V1.1 L1 Modules has a Drivetrain Free Speed of 12.5 ft/s or 3.81 m/s
            AUTO(new SwerveModuleLimits(3.5, 3.5 * 5, Math.toRadians(1080))),
            TELEOP_FAST(new SwerveModuleLimits(3.5, 3.5 * 5, Math.toRadians(1080)));

            public final SwerveModuleLimits setpoint;

            ModuleLimits(SwerveModuleLimits setpoint) {
                this.setpoint = setpoint;
            }
        }

        /* Swerve Profiling Values Changed */
        public static enum TeleopSpeeds {
            SLOW(0.5, 0.5 * Math.PI),
            MAX(3.5, 3.0 * Math.PI);

            public final double translationalSpeed;
            public final double angularSpeed;

            TeleopSpeeds(double translationalSpeed, double angularSpeed) {
                this.translationalSpeed = translationalSpeed;
                this.angularSpeed = angularSpeed;
            }
        }

        public static final double maxSpeed = 3.5; // meters per second
        public static final double maxAngularVelocity = Math.PI * 3.0;

        /* Neutral Modes */
        public static final IdleMode steerIdleMode = IdleMode.kBrake;
        public static final IdleMode driveIdleMode = IdleMode.kBrake;

        /* Motor Inverts */
        public static final boolean driveInvert = false;
        public static final boolean steerInvert = false;

        /* Steer Encoder Invert */
        public static final boolean cancoderInvert = false;
        public static final double cancoderUpdatePeriod = 0.04;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 Changed*/
        public static final class Mod0 {
            public static final int driveMotorID = CANID.SWERVE_FL_DRIVE.val();
            public static final int steerMotorID = CANID.SWERVE_FL_STEERING.val();
            public static final int canCoderID = CANID.SWERVE_FL_CANCODER.val();
            public static final Rotation2d steerOffset = Rotation2d.fromDegrees(270);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, steerMotorID, canCoderID, steerOffset);
        }

        /* Front Right Module - Module 1 Changed*/
        public static final class Mod1 {
            public static final int driveMotorID = CANID.SWERVE_FR_DRIVE.val();
            public static final int steerMotorID = CANID.SWERVE_FR_STEERING.val();
            public static final int canCoderID = CANID.SWERVE_FR_CANCODER.val();
            public static final Rotation2d steerOffset = Rotation2d.fromDegrees(157.5);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, steerMotorID, canCoderID, steerOffset);
        }

        /* Back Left Module - Module 2 Changed*/
        public static final class Mod2 {
            public static final int driveMotorID = CANID.SWERVE_RL_DRIVE.val();
            public static final int steerMotorID = CANID.SWERVE_RL_STEERING.val();
            public static final int canCoderID = CANID.SWERVE_RL_CANCODER.val();
            public static final Rotation2d steerOffset = Rotation2d.fromDegrees(192);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, steerMotorID, canCoderID, steerOffset);
        }

        /* Back Right Module - Module 3 Changed*/
        public static final class Mod3 {
            public static final int driveMotorID = CANID.SWERVE_RR_DRIVE.val();
            public static final int steerMotorID = CANID.SWERVE_RR_STEERING.val();
            public static final int canCoderID = CANID.SWERVE_RR_CANCODER.val();
            public static final Rotation2d steerOffset = Rotation2d.fromDegrees(6);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, steerMotorID, canCoderID, steerOffset);
        }
    }

    public static final class AutoConstants {
        // Changed
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        // Changed values

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1.35;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class BlingConstants {
        public static int CANDLE = CANID.CANDLE.val();
    }

    public static final class Intake {
        public static final int INTAKE = CANID.INTAKE.val();
        public static final byte frontSensor = 0; // its the same but lighter, so dtw
        public static final byte centerSensor = 2; // its the same but lighter, so dtw
        public static final byte backSensor = 1; // its the same but lighter, so dtw
    }

    public class ArmConfig {
        public static final int ARM_SPARK_CAN_ID = CANID.ARM.val();
        public static final boolean SET_INVERTED = true;
        public static final boolean setInvered = true;
        public static final boolean INVERT_ENCODER = false;

        public static final int CURRENT_LIMIT = 20;

        public static final double shiftEncoderRange = 10;
        // offset unit: degrees
        public static final double armAbsEncoderOffset =
                Math.toDegrees(3.20433) + 3.0 - shiftEncoderRange;

        public static final double MAX_ARM_ANGLE_DEG = 180;
        public static final double MIN_ARM_ANGLE_DEG = -2;

        // soft limit constant for bottom arm
        public static final float arm_forward_limit =
                (float) Math.toRadians(MAX_ARM_ANGLE_DEG + shiftEncoderRange);
        public static final float arm_reverse_limit =
                (float) Math.toRadians(MIN_ARM_ANGLE_DEG + shiftEncoderRange);
        public static final boolean SOFT_LIMIT_ENABLE = true;

        // PID constants
        public static final double arm_kP = robotSpecific(2.700000, 0.0, 0.0, 1.4);
        public static final double arm_kI = robotSpecific(0.0, 0.0, 0.0, 0.0003);
        public static final double arm_kD = robotSpecific(0.800000, 0.0, 0.0, 0.9);
        public static final double arm_kIz = robotSpecific(0.02, 0.0, 0.0, 0.3);
        public static final double arm_kFF = 0.013;
        public static final double min_output = -1;
        public static final double max_output = 1;

        // PID constants for far shots
        public static final double arm_far_kP = 6.0;
        public static final double arm_far_kI = 0;
        public static final double arm_far_kD = 6.0;
        public static final double arm_far_kFF = 0.06;
        public static final double arm_far_iZone = Math.toRadians(1.5);

        // ff calculations
        public static final double gravitationalConstant =
                389.0886; // inches/s/s which is equal to 9.81 m/s/s
        public static final double ARM_FORCE = 11.29 * gravitationalConstant; // 11.29 lb

        public static final double LENGTH_ARM_TO_COG = 14.56;

        public static final double ARM_ENCODER_GEAR_RATIO = 1;

        // arm position unit: radians
        public static final double armPositionConversionFactor =
                2 * Math.PI / ARM_ENCODER_GEAR_RATIO;
        // arm velocity unit: radians/sec
        public static final double armVelocityConversionFactor = armPositionConversionFactor / 60.0;

        public static final double MAX_VEL = Math.PI * 1.5;
        public static final double MAX_ACCEL = Math.PI * 1.5;

        public static final double MOMENT_TO_VOLTAGE = 0.000005;

        // Arm Display values
        public static final double LENGTH_METERS = 0.371475;
    }

    public static enum ArmSetPoints {
        // @todo: to be calibrated
        IDLE(60),
        INTAKE(-0.1),
        SPEAKER_KICKBOT_SHOT(15.5),
        NO_INTAKE(5.0),
        SPEAKER_VISION_SHOT(33),
        AMP(100);

        public final double angleDeg;

        ArmSetPoints(double angleDeg) {
            this.angleDeg = angleDeg;
        }
    }

    public static final boolean tuningMode = true;

    public static final class ShooterConstants {
        public static final byte MOTOR_ID = (byte) CANID.SHOOTER.val();
        public static final double kP = 0.0002,
                kI = 0.0,
                kD = 0.0,
                kFF = 0.0003,
                kP1 = 0.00027,
                kI1 = 0.0,
                kD1 = 0.00015,
                kFF1 = 0.00027,
                kMaxOutput = 1.0,
                kMinOutput = -1.0,
                maxRPM = 5700.0,
                subwooferRPM = 2750;
    }

    /**
     * Differential Drive Constants
     */
    public static class DIFF {

        // Differential Drive CAN IDs
        public static int DIFF_LEADER_LEFT = robotSpecific(-01, 0, 2, -01);
        public static int DIFF_LEADER_RIGHT = robotSpecific(-01, 0, 1, -01);
        public static int DIFF_FOLLOWER_LEFT = robotSpecific(-01, 0, -1, -01);
        public static int DIFF_FOLLOWER_RIGHT = robotSpecific(-01, 0, -1, -01);

        public static boolean ISNEOS = robotSpecific(true, false, false, true);
        public static boolean HAS_FOLLOWERS = robotSpecific(true, false, false, true);
        public static boolean LEFT_FOLLOWER_ISVICTOR = robotSpecific(false, false, false, false);
        public static boolean RIGHT_FOLLOWER_ISVICTOR = robotSpecific(false, false, false, false);

        // Invert motors to consider forward as forward (same practice for all objects)
        public static boolean LEADER_LEFT_INVERTED = robotSpecific(false, false, false, false);
        public static boolean LEADER_RIGHT_INVERTED = robotSpecific(false, false, false, false);
        public static boolean FOLLOWER_LEFT_INVERTED = robotSpecific(false, false, false, false);
        public static boolean FOLLOWER_RIGHT_INVERTED = robotSpecific(false, false, false, false);

        public static boolean LEFT_SENSORPHASE = robotSpecific(false, false, true, false);
        public static boolean RIGHT_SENSORPHASE = robotSpecific(false, false, true, false);

        // Current limiter Constants
        public static boolean TALON_CURRENT_LIMIT =
                true; // Enable or disable motor current limiting.
        public static int TALON_PEAK_CURRENT_AMPS =
                80; // Peak current threshold to trigger the current limit
        public static int TALON_PEAK_TIME_MS =
                250; // Time after current exceeds peak current to trigger current limit
        public static int TALON_CONTIN_CURRENT_AMPS =
                40; // Current to mantain once current limit is triggered

        // Drivetrain idle mode and voltage/current limits
        public static int NEO_RAMSETE_CURRENTLIMIT = 40;
        public static int NEO_DRIVER_CURRENTLIMIT = 80;

        public static IdleMode TELEOP_IDLEMODE = IdleMode.kBrake;
        public static NeutralMode TELEOP_NEUTRALMODE = NeutralMode.Brake;

        public static IdleMode AUTO_IDLEMODE = IdleMode.kBrake;
        public static NeutralMode AUTO_NEUTRALMODE = NeutralMode.Brake;

        public static double BRAKE_IN_DISABLE_TIME = 2.0;
    }

    public static final int CAN_TIMEOUT_SHORT = 10;
    public static final int CAN_TIMEOUT_LONG = 100;
    public static Double DRIVER_JOYSTICK_DEADBAND = 0.15;

    public static final class NTConfig {
        public static NetworkTable
                shooterTable = NetworkTableInstance.getDefault().getTable("Shooter"),
                armTable = NetworkTableInstance.getDefault().getTable("Arm"),
                swerveTable = NetworkTableInstance.getDefault().getTable("Swerve"),
                climberTable = NetworkTableInstance.getDefault().getTable("Climber"),
                intakeTable = NetworkTableInstance.getDefault().getTable("Intake"),
                visionTable = NetworkTableInstance.getDefault().getTable("Vision");

        public static double SLOW_PERIODIC_SECONDS = 250;
        public static double FAST_PERIODIC_SECONDS = 20;
    }
}
