package frc.lib.lib2706.button;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XBoxControllerUtil {
    /**
     * This file should not be constructed. It should only have static factory methods.
     */
    private XBoxControllerUtil() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Create a Trigger that works when the DownLeft, Left or UpLeft POV button is pressed.
     * This accounts for the fact that POV buttons are easy to press the wrong button.
     *
     * @param controller to create the trigger from.
     * @return Trigger that works for the DownLeft, Left or UpLeft buttons
     */
    public static Trigger leftPOV(CommandXboxController controller) {
        return controller.povDownLeft().or(controller.povLeft().or(controller.povUpLeft()));
    }

    /**
     * Create a Trigger that works when the DownRight, Right or UpRight POV button is pressed.
     * This accounts for the fact that POV buttons are easy to press the wrong button.
     *
     * @param controller to create the trigger from.
     * @return Trigger that works for the DownRight, Right or UpRigh buttons
     */
    public static Trigger rightPOV(CommandXboxController controller) {
        return controller.povDownRight().or(controller.povRight().or(controller.povUpRight()));
    }

    /**
     * Takes X, Y joystick inputs and applies a deadband in the hypotenuse of the vector and squares it.
     *
     * This function was made by FRC Team 6328 Mechanical Advantage.
     *
     * @param x                the x value of the joystick from -1.0 to 1.0
     * @param y                the y value of the joystick from -1.0 to 1.0
     * @param squareMagnitude  true to square the magnitude of the XY, false to leave it linear
     * @param joystickDeadband the deadband value for the joystick
     * @return the calculated linear velocity as a Translation2d object
     */
    public static Translation2d calcLinearVelocity(
            double x, double y, boolean squareMagnitude, double joystickDeadband) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), joystickDeadband);
        Rotation2d linearDirection = new Rotation2d(x, y);

        // Square magnitude
        if (squareMagnitude) {
            linearMagnitude = linearMagnitude * linearMagnitude;
        }

        // Calculate new linear velocity
        Translation2d linearVelocity =
                new Pose2d(new Translation2d(), linearDirection)
                        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                        .getTranslation();
        return linearVelocity;
    }

    /**
     * Convert the x and y values of a joystick to polar coordinates.
     *
     * @param xVal             the x value of the joystick
     * @param yVal             the y value of the joystick
     * @param squareMagnitude  true to square the magnitude of the XY, false to leave it linear
     * @param joystickDeadband the deadband value for the joystick
     * @return a Translation2d object representing the joystick values in polar coordinates
     */
    public static Translation2d convertXYToPolar(
            double xVal, double yVal, boolean squareMagnitude, double joystickDeadband) {
        // Define control hypotenuse and control signed angle
        double linearMagnitude = Math.hypot(xVal, yVal);
        Rotation2d linearDirection = new Rotation2d(xVal, yVal);

        // Define limited angle from [0, PI/2]
        double limitedAngle = Math.atan2(Math.abs(yVal), Math.abs(xVal));

        // Apply the joystick deadband on the magnitude
        linearMagnitude = MathUtil.applyDeadband(linearMagnitude, joystickDeadband);

        // Square magnitude
        if (squareMagnitude) {
            linearMagnitude = linearMagnitude * linearMagnitude;
        }

        // If this is greater, then subtract
        double unitHypot;
        if (limitedAngle > Math.PI / 4) {
            // Unit hypotenuse reverse of tan function's domain. Shift [PI/4, PI/2] to -> [PI/4, 0]
            unitHypot = Math.hypot(1, Math.tan(Math.PI / 2 - limitedAngle));
        } else {
            // Unit hypotenuse in line with tan function's domain [0, PI/4]
            unitHypot = Math.hypot(1, Math.tan(limitedAngle));
        }

        // Scales current control (i.e. 1.41) to unit hypotenuse (i.e. 1.41)
        linearMagnitude = linearMagnitude / unitHypot;

        // Calculate new linear velocity
        Translation2d linearVelocity =
                new Pose2d(new Translation2d(), linearDirection)
                        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                        .getTranslation();
        return linearVelocity;
    }
}
