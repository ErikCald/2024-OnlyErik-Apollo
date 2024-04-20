package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.lib.lib2706.button.FakeCommandXboxController;
import frc.lib.lib2706.button.FakeCommandXboxController.FakeXboxController;
import frc.robot.Config.GeneralConfig;
import frc.robot.Config.SwerveConfig.TeleopSpeeds;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class TeleopSwerve extends Command {
    private SwerveSubsystem s_Swerve;
    private CommandXboxController m_driver;

    private final int translationAxis;
    private final int strafeAxis;
    private final int rotationAxis;

    private static TeleopSpeeds speed = TeleopSpeeds.MAX;
    private static boolean isFieldRelative = true;

    public TeleopSwerve(CommandXboxController m_driver) {
        this.m_driver = m_driver;

        if (m_driver instanceof FakeCommandXboxController) {
            translationAxis = ((FakeXboxController) m_driver.getHID()).mappedAxis.kLeftY;
            strafeAxis = ((FakeXboxController) m_driver.getHID()).mappedAxis.kLeftX;
            rotationAxis = ((FakeXboxController) m_driver.getHID()).mappedAxis.kRightX;
        } else {
            translationAxis = XboxController.Axis.kLeftY.value;
            strafeAxis = XboxController.Axis.kLeftX.value;
            rotationAxis = XboxController.Axis.kRightX.value;
        }

        this.s_Swerve = SwerveSubsystem.getInstance();
        addRequirements(s_Swerve);
    }

    public static void setSpeeds(TeleopSpeeds newSpeed) {
        speed = newSpeed;
    }

    public static void setFieldRelative(boolean newIsFieldRelative) {
        isFieldRelative = newIsFieldRelative;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        s_Swerve.drive(calculateChassisSpeeds(), isFieldRelative, false);
    }

    protected double getJoystickX() {
        return -m_driver.getRawAxis(translationAxis);
    }

    protected double getJoystickY() {
        return -m_driver.getRawAxis(strafeAxis);
    }

    protected double calculateRotationVal() {
        double rotationVal =
                MathUtil.applyDeadband(
                        -m_driver.getRawAxis(rotationAxis), GeneralConfig.joystickDeadband);
        rotationVal = Math.copySign(rotationVal * rotationVal, rotationVal);
        return rotationVal * speed.angularSpeed;
    }

    protected Translation2d calculateXYVal() {
        Translation2d linearVelocity = calcLinearVelocity(getJoystickX(), getJoystickY());
        return new Translation2d(
                linearVelocity.getX() * speed.translationalSpeed,
                linearVelocity.getY() * speed.translationalSpeed);
    }

    protected ChassisSpeeds calculateChassisSpeeds() {
        Translation2d linearVelocity = calculateXYVal();

        // Flip the x and y values if the robot is red and field relative
        // Required since robot defines forwards as away from the blue alliance wall
        if (isFieldRelative
                && DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            linearVelocity = linearVelocity.rotateBy(Rotation2d.fromRadians(Math.PI));
        }

        return new ChassisSpeeds(
                linearVelocity.getX(), linearVelocity.getY(), calculateRotationVal());
    }

    public static Translation2d calcLinearVelocity(double x, double y) {
        // Apply deadband
        double linearMagnitude =
                MathUtil.applyDeadband(Math.hypot(x, y), GeneralConfig.joystickDeadband);
        Rotation2d linearDirection = new Rotation2d(x, y);

        // Square magnitude
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Calcaulate new linear velocity
        Translation2d linearVelocity =
                new Pose2d(new Translation2d(), linearDirection)
                        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                        .getTranslation();
        return linearVelocity;
    }
}
