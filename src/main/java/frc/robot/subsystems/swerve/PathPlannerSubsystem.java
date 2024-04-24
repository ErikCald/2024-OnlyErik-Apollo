// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.lib2706.networktables.AdvantageUtil;
import frc.robot.Config.NTConfig;
import frc.robot.subsystems.swerve.LocalADStarLoggable.GridPosition;

public class PathPlannerSubsystem extends SubsystemBase {
    private static PathPlannerSubsystem m_instance= null;
    private LocalADStarLoggable m_pathFinder;
    private DoubleArrayPublisher pubStaticObstacles;

    public static PathPlannerSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new PathPlannerSubsystem();
        }
        return m_instance;
    }

    /** Creates a new PathFinderSubsystem. */
    private PathPlannerSubsystem() {
        m_pathFinder = new LocalADStarLoggable();

        NetworkTable table = NTConfig.pathfindingTable;
        pubStaticObstacles = table.getDoubleArrayTopic("staticObstacles").publish();

        Commands.sequence(
            Commands.waitSeconds(15),
            Commands.runOnce(() -> logStaticObstacles())).withName("Log Static Obstacles").ignoringDisable(true).schedule();
        ;
    }

    @Override
    public void periodic() {
        // logStaticObstacles();
    }

    public void logStaticObstacles() {
        ArrayList<Pose2d> poses = new ArrayList<>();
        for (GridPosition pos : m_pathFinder.staticObstacles) {
            poses.add(new Pose2d(m_pathFinder.gridPosToTranslation2d(pos), new Rotation2d()));
        }

        pubStaticObstacles.accept(AdvantageUtil.deconstructPose2ds(poses));
        
    }
}
