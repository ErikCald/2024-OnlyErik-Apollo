package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.lib.lib2706.button.FakeCommandXboxController;
import frc.lib.lib2706.button.FakeCommandXboxController.FakeXboxController;
import frc.lib.lib2706.button.XBoxControllerUtil;
import frc.robot.Config.GeneralConfig;
import frc.robot.Config.SwerveConfig.TeleopSpeeds;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * The TeleopSwerve class represents a command for controlling the swerve drive subsystem during teleoperated mode.
 * It handles the user input from the driver controller and calculates the desired chassis speeds.
 *
 * Extend this class and override the controls to implement custom logic on a part of the controls.
 */
public class TeleopSwerve extends Command {
    private final boolean CONVERT_XY_TO_POLAR = GeneralConfig.convertXYToPolar;

    private SwerveSubsystem m_swerve;
    private CommandXboxController m_driver;

    private final int translationAxis;
    private final int strafeAxis;
    private final int rotationAxis;

    private static TeleopSpeeds s_speed = TeleopSpeeds.MAX;
    private static boolean s_isFieldRelative = true;

    /**
     * A command that allows for teleoperated control of a swerve drive system.
     * This command takes input from an Xbox controller and translates it into
     * commands for the swerve subsystem.
     *
     * @param m_driver The Xbox controller used for input.
     */
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

        this.m_swerve = SwerveSubsystem.getInstance();
        addRequirements(m_swerve);
    }

    /**
     * Sets the teleop speeds for the robot.
     *
     * @param speed the new teleop speeds to set
     */
    public static void setSpeeds(TeleopSpeeds speed) {
        s_speed = speed;
    }

    /**
     * Sets the field-relative vs robot-relative mode for the teleop swerve command.
     *
     * @param isFieldRelative true for field-relative, false for robot-relative
     */
    public static void setFieldRelative(boolean isFieldRelative) {
        s_isFieldRelative = isFieldRelative;
    }

    @Override
    public void execute() {
        m_swerve.drive(calculateChassisSpeeds(), s_isFieldRelative, false);
    }

    /**
     * Returns the X-axis value of the joystick for translation.
     *
     * @return The X-axis value of the joystick for translation.
     */
    protected double getJoystickX() {
        return -m_driver.getRawAxis(translationAxis);
    }

    /**
     * Returns the Y-axis value of the joystick for strafing.
     *
     * @return The Y-axis value of the joystick for strafing.
     */
    protected double getJoystickY() {
        return -m_driver.getRawAxis(strafeAxis);
    }

    /**
     * Calculates the rotation value for the teleop swerve command.
     * <p> Override this to change the behavior of the rotation control.
     *
     * @return The calculated rotation value.
     */
    protected double calculateRotationVal() {
        double rotationVal =
                MathUtil.applyDeadband(
                        -m_driver.getRawAxis(rotationAxis), GeneralConfig.joystickDeadband);
        rotationVal = Math.copySign(rotationVal * rotationVal, rotationVal);
        return rotationVal * s_speed.angularSpeed;
    }

    /**
     * Calculates the translation value for the teleop swerve command.
     * <p> Override this to change the behavior of the translation control.
     *
     * @return A Translation2d value with XY that represents meters per second.
     */
    protected Translation2d calculateXYVal() {
        Translation2d linearVelocity;
        if (CONVERT_XY_TO_POLAR) {
            linearVelocity =
                    XBoxControllerUtil.convertXYToPolar(
                            getJoystickX(), getJoystickY(), true, GeneralConfig.joystickDeadband);
        } else {
            linearVelocity =
                    XBoxControllerUtil.calcLinearVelocity(
                            getJoystickX(), getJoystickY(), true, GeneralConfig.joystickDeadband);
        }

        return new Translation2d(
                linearVelocity.getX() * s_speed.translationalSpeed,
                linearVelocity.getY() * s_speed.translationalSpeed);
    }

    /**
     * Calculates the chassis speeds based on the linear velocity and rotation values.
     * If the robot is red and field relative, the x and y values are flipped.
     * <p> Override this to change the behavior completely override the behaviour.
     *
     * @return The calculated ChassisSpeeds object.
     */
    protected ChassisSpeeds calculateChassisSpeeds() {
        Translation2d linearVelocity = calculateXYVal();

        // Flip the x and y values if the robot is red and field relative
        // Required since robot defines forwards as away from the blue alliance wall
        if (s_isFieldRelative
                && DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            linearVelocity = linearVelocity.rotateBy(Rotation2d.fromRadians(Math.PI));
        }

        return new ChassisSpeeds(
                linearVelocity.getX(), linearVelocity.getY(), calculateRotationVal());
    }
}
