// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Config.ArmConfig;

public class ArmDisplay {
    private final double length = Units.metersToInches(ArmConfig.LENGTH_METERS);


    private final double windowX = 140;
    private final double windowY = 85;

    private final double pivot1X = windowX / 2;
    private final double pivot1Y = 15;

    private Mechanism2d m_mech2d;

    private SingleJointArmDisplay m_measurementDisplay;
    private SingleJointArmDisplay m_setpointDisplay;

    /** Creates a new ArmDisplay. */
    public ArmDisplay() {
        m_mech2d = new Mechanism2d(windowX, windowY);

        MechanismRoot2d m_arm1PivotPoint =
                m_mech2d.getRoot("ArmBumperPoint", pivot1X + 13.4, pivot1Y - 14.7 + 3);
        m_arm1PivotPoint.append(
                new MechanismLigament2d(
                        "ArmBumperRectangle",
                        Units.metersToInches(1),
                        -180,
                        35,
                        new Color8Bit(Color.kBlue)));

        MechanismRoot2d m_armBasePivotPoint = m_mech2d.getRoot("ArmBasePivot", pivot1X, pivot1Y);
        m_armBasePivotPoint.append(
                new MechanismLigament2d("ArmBase", 13, -90, 20, new Color8Bit(Color.kBlue)));

        m_setpointDisplay =
                new SingleJointArmDisplay(m_mech2d, 3, new Color8Bit(Color.kGreen), "Setpoint");
        m_measurementDisplay =
                new SingleJointArmDisplay(m_mech2d, 12, new Color8Bit(Color.kRed), "Measurement");

        // Put Mechanism 2d to SmartDashboard
        Shuffleboard.getTab("ArmDisplay").add("ArmDisplayMech", m_mech2d);
        // SmartDashboard.putData("ArmDisplaySmart", m_mech2d);

        m_setpointDisplay.updateDisplay(Math.toRadians(70));
        m_measurementDisplay.updateDisplay(Math.toRadians(80));
    }

    private class SingleJointArmDisplay {

        private MechanismRoot2d m_arm2PivotPoint;

        private MechanismLigament2d m_armMainLine;

        private SingleJointArmDisplay(
                Mechanism2d mech2d, double lineWidth, Color8Bit color, String name) {
            MechanismRoot2d m_armPivotPoint = mech2d.getRoot(name + "ArmPivot", pivot1X, pivot1Y);

            m_armMainLine = new MechanismLigament2d(name + "Arm1Line", length, 80, lineWidth,
            color);
            m_armPivotPoint.append(m_armMainLine);

            // m_arm2PivotPoint = m_mech2d.getRoot(name + "Arm2Pivot", windowX - 30, windowY - 15);

            // m_arm2Line = new MechanismLigament2d(name + "Arm2Line", length2, -80, lineWidth,
            // color);
            // m_arm2PivotPoint.append(m_arm2Line);
        }

        private void updateDisplay(double angleRad) {
            m_armMainLine.setAngle(Math.toDegrees(angleRad));

            // Translation2d deltaArm1 = new Translation2d(length1, new Rotation2d(encoderAngle1));
            // Translation2d arm2Pivot = new Translation2d(pivot1X, pivot1Y).plus(deltaArm1);

            // m_arm2PivotPoint.setPosition(arm2Pivot.getX(), arm2Pivot.getY());

            // m_arm2Line.setAngle(Math.toDegrees(encoderAngle1 - Math.PI + encoderAngle2));
        }
    }

    public void updateMeasurementDisplay(double measuredAngleRad) {
        m_measurementDisplay.updateDisplay(measuredAngleRad);
    }

    public void updateSetpointDisplay(double setpointAngleRad) {
        m_setpointDisplay.updateDisplay(setpointAngleRad);
    }
}
