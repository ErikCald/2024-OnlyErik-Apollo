// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.sharing_files;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import frc.lib.lib2706.networktables.AdvantageUtil;
import frc.robot.Config.NTConfig;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ApriltagSubsystem;

/**
 * Represents a Photon AprilTag camera used for vision processing.
 * This class provides methods to initialize the camera, estimate pose, and publish data to NetworkTables.
 */
public class PhotonAprilTagCamera {
    private static final AprilTagFieldLayout s_fieldLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

    private PhotonCamera m_camera;
    private PhotonPoseEstimator m_estimator;

    private DoubleArrayPublisher pubEstPose, pubTagPoses;
    private IntegerArrayPublisher pubTagIds;
    private StringPublisher pubDebugMsg;

    /**
     * Represents a Photon AprilTag camera used for vision processing.
     * This class provides methods to initialize the camera, estimate pose, and publish data to NetworkTables.
     *
     * @param cameraName The name of the camera.
     * @param robotToCamera The transformation matrix representing the pose of the camera relative to the robot.
     */
    public PhotonAprilTagCamera(
            String cameraName, Transform3d robotToCamera) {
        m_camera = new PhotonCamera(cameraName);
        m_estimator =
                new PhotonPoseEstimator(
                        s_fieldLayout,
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        m_camera,
                        robotToCamera);

        NetworkTable table = NTConfig.visionTable.getSubTable(cameraName);

        pubEstPose = table.getDoubleArrayTopic("estimatedPose").publish(PubSubOption.periodic(0.02));
        pubTagPoses = table.getDoubleArrayTopic("detectedTagPoses").publish(PubSubOption.periodic(0.02));
        pubTagIds = table.getIntegerArrayTopic("detectedTagIds").publish(PubSubOption.periodic(0.02));
        pubDebugMsg = table.getStringTopic("DebuggingMessage").publish(PubSubOption.periodic(0.02));  
    }

    /**
     * Updates the PhotonAprilTagCamera by processing the latest camera result and estimating the robot pose.
     * <p>
     * If no new targets are detected, it publishes empty arrays for tag IDs, tag poses, and estimated pose.
     * If targets are detected, it creates arrays of the detected tag IDs and tag poses, and publishes them.
     * It then applies filters to the estimated pose and publishes the filtered pose if it passes all filters.
     * Finally, it logs the estimated pose and sends the measurement to the SwerveSubsystem.
     *
     * @return True if the data was successfully passed to the SwerveSubsystem, false otherwise.
     */
    public boolean update() {
        PhotonPipelineResult cameraResult = m_camera.getLatestResult();
        Optional<EstimatedRobotPose> estimate = m_estimator.update(cameraResult);

        // Check if an no new data is available
        if (estimate.isEmpty()) {
            pubDebugMsg.set("No new targets detected");
            pubTagIds.accept(new long[0]);
            pubTagPoses.accept(new double[0]);
            pubEstPose.accept(new double[0]);
            return false;
        }

        // Create an array of the detected tag IDs and array of tag poses
        long[] detectedTagIds = new long[estimate.get().targetsUsed.size()];
        ArrayList<Pose3d> detectedTagPoses = new ArrayList<>();
        for (int i = 0; i < estimate.get().targetsUsed.size(); i++) {
            PhotonTrackedTarget target = estimate.get().targetsUsed.get(i);
            detectedTagIds[i] = target.getFiducialId();

            Optional<Pose3d> tagPose = s_fieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isPresent()) {
                detectedTagPoses.add(tagPose.get());
            }
        }
        pubTagIds.accept(detectedTagIds);
        pubTagPoses.accept(AdvantageUtil.deconstructPose3ds(detectedTagPoses));

        // Must use multi-tag PnP
        if (estimate.get().strategy != PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
            && estimate.get().strategy != PoseStrategy.MULTI_TAG_PNP_ON_RIO) {
            pubDebugMsg.accept("Invalid strategy");
            return false;
        }

        // Must have more than 1 tag in frame
        if (estimate.get().targetsUsed.size() <= 1) {
            pubDebugMsg.accept("Only " + estimate.get().targetsUsed.size() + " tags detected");
            return false;
        }

        // Log the estimated pose
        pubEstPose.accept(AdvantageUtil.deconstruct(estimate.get().estimatedPose));
        pubDebugMsg.accept("Accepted data");

        // Send measurement to the swerve subsystem
        SwerveSubsystem.getInstance()
                .addVisionMeasurement(
                        estimate.get().estimatedPose.toPose2d(),
                        estimate.get().timestampSeconds);
        return true;
    }
}
