// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.apriltag;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

import static frc.robot.constants.FieldConstants.compFieldLength;
import static frc.robot.constants.FieldConstants.compFieldWidth;
import static frc.robot.constants.FieldConstants.compLayout;
import static frc.robot.constants.FieldConstants.shopFieldLength;
import static frc.robot.constants.FieldConstants.shopFieldWidth;
import static frc.robot.constants.FieldConstants.shopLayout;
import static frc.robot.vision.apriltag.VisionConstants.CAMERA_LOG_PATH;
import static frc.robot.vision.apriltag.VisionConstants.EXCLUDED_TAG_IDS;
import static frc.robot.vision.apriltag.VisionConstants.FIELD_BORDER_MARGIN;
import static frc.robot.vision.apriltag.VisionConstants.MAX_AMBIGUITY;
import static frc.robot.vision.apriltag.VisionConstants.THETA_STDDEV_MODEL;
import static frc.robot.vision.apriltag.VisionConstants.XY_STDDEV_MODEL;
import static frc.robot.vision.apriltag.VisionConstants.maxChangeDistance;
import static frc.robot.vision.apriltag.VisionConstants.multiTagModifier;
import static frc.robot.vision.apriltag.VisionConstants.stabilityModifier;
import static frc.robot.vision.apriltag.VisionConstants.thetaModifier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.GeomUtil;
import frc.robot.commons.TimestampedVisionUpdate;
import frc.robot.constants.FieldConstants;
import frc.robot.commons.GremlinLogger;

public class GremlinApriltagVision extends SubsystemBase {
  private GremlinPhotonCamera[] cameras;
  private List<TimestampedVisionUpdate> visionUpdates;

  private PhotonCameraSim[] simCameras;
  private VisionSystemSim visionSystemSim;
  private SimCameraProperties[] simCameraProperties;

  private static final AprilTagFieldLayout LAYOUT = FieldConstants.isShopField ? shopLayout : compLayout;
  private static final double fieldLength = FieldConstants.isShopField ? shopFieldLength : compFieldLength;
  private static final double fieldWidth = FieldConstants.isShopField ? shopFieldWidth : compFieldWidth;

  // Will be the function in drivetrain that adds vision estimate to pose
  // estimation
  private Consumer<List<TimestampedVisionUpdate>> visionConsumer = (visionUpdates) -> {
  };
  // Will be the function in driveTrain that supplies current pose estimate
  private Supplier<Pose2d> poseSupplier = () -> new Pose2d();

  /** Creates a new GremlinApriltagVision. */
  public GremlinApriltagVision(
      GremlinPhotonCamera[] cameras,
      Supplier<Pose2d> poseSupplier,
      Consumer<List<TimestampedVisionUpdate>> visionConsumer) {

    this.cameras = cameras;
    this.poseSupplier = poseSupplier;
    this.visionConsumer = visionConsumer;

    if (Utils.isSimulation()) {
      configSim();
    }
  }

  @Override
  public void periodic() {
    processVisionUpdates();
    visionConsumer.accept(visionUpdates);
    GremlinLogger.logSD("VISION/visionUpdatesSize", visionUpdates.size());
  }

  @SuppressWarnings("unused")
  public void processVisionUpdates() {
    // Reset VisionUpdates
    visionUpdates = new ArrayList<>();
    // Get current robot position
    Pose2d currentPose = poseSupplier.get();

    // loop through all cameras
    for (int i = 0; i < cameras.length; i++) {
      String logPath = CAMERA_LOG_PATH + cameras[i].getName();

      Pose3d cameraPose;

      // Camera specific variables
      Transform3d camToRobotTransform = GeomUtil.pose3dToTransform3d(cameras[i].getCameraPose()).inverse();
      List<PhotonPipelineResult> unreadResults = cameras[i].getAllUnreadResults();

      for (int j = 0; j < unreadResults.size(); j++) {
        Pose2d calculatedRobotPose;
        List<Pose3d> tagPose3ds = new ArrayList<>();
        PhotonPipelineResult unprocessedResult = unreadResults.get(j);
        double timestamp = unprocessedResult.getTimestampSeconds();
        double singleTagAdjustment = 1.0;

        GremlinLogger.logSD(logPath + "/Hastargets", unprocessedResult.hasTargets());
        GremlinLogger.logSD(logPath + "/Timestamp", timestamp);

        // Continue if the camera doesn't have any targets
        if (!unprocessedResult.hasTargets()) {
          continue;
        }

        // if it has a MultiTag result we prefer to use that
        boolean shouldUseMultiTag = unprocessedResult.getMultiTagResult().isPresent();

        if (shouldUseMultiTag) {
          // TODO: think about adding processing to compare best and alt
          cameraPose = GeomUtil.transform3dToPose3d(unprocessedResult.getMultiTagResult().get().estimatedPose.best);

          calculatedRobotPose = cameraPose.transformBy(camToRobotTransform).toPose2d();

          // Populate array of tag poses with tags used
          for (int id : unprocessedResult.getMultiTagResult().get().fiducialIDsUsed) {
            tagPose3ds.add(LAYOUT.getTagPose(id).get());
          }

          GremlinLogger.logSD(logPath + "/Multitag", true);
        } else {
          PhotonTrackedTarget target = unprocessedResult.getBestTarget();

          // We dont like some tags
          if (EXCLUDED_TAG_IDS.contains(target.getFiducialId()))
            continue;

          Pose3d tagPose = LAYOUT.getTagPose(target.getFiducialId()).get();

          Pose3d bestCamPose = tagPose.transformBy(target.getBestCameraToTarget().inverse());
          Pose3d altCamPose = tagPose.transformBy(target.getAlternateCameraToTarget().inverse());
          Pose2d bestRobotPose = bestCamPose.transformBy(camToRobotTransform).toPose2d();
          Pose2d altRobotPose = altCamPose.transformBy(camToRobotTransform).toPose2d();

          double ambiguity = target.getPoseAmbiguity();
          boolean betterRotationDiff = (Math.abs(bestRobotPose.getRotation().minus(currentPose.getRotation())
              .getRadians()) < Math.abs(altRobotPose.getRotation().minus(currentPose.getRotation()).getRadians()));

          if (ambiguity < MAX_AMBIGUITY) {
            // Best pose is significantly better than alt
            cameraPose = bestCamPose;
            calculatedRobotPose = bestRobotPose;
          } else if (betterRotationDiff) {
            // the rotation based on best pose is closer to our current estimated rotation
            cameraPose = bestCamPose;
            calculatedRobotPose = bestRobotPose;
          } else {
            cameraPose = altCamPose;
            calculatedRobotPose = altRobotPose;
          }

          tagPose3ds.add(tagPose);
          singleTagAdjustment = SingleTagAdjusters.getAdjustmentForTag(target.getFiducialId());

          GremlinLogger.logSD(logPath + "/Multitag", false);
        }

        if (cameraPose == null || calculatedRobotPose == null)
          continue;

        // Move on to next camera if robot pose is off the field
        if (calculatedRobotPose.getX() < -FIELD_BORDER_MARGIN
            || calculatedRobotPose.getX() > fieldLength + FIELD_BORDER_MARGIN
            || calculatedRobotPose.getY() < -FIELD_BORDER_MARGIN
            || calculatedRobotPose.getY() > fieldWidth + FIELD_BORDER_MARGIN) {
          continue;
        }

        if (calculatedRobotPose.getTranslation()
            .getDistance(poseSupplier.get().getTranslation()) > maxChangeDistance)
          continue;

        // Calculate average distance to tag
        double totalDistance = 0.0;
        for (Pose3d tagPose : tagPose3ds) {
          totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
        }

        double avgDistance = totalDistance / tagPose3ds.size();
        double xyStdDev = 0.0;
        double thetaStdDev = 0.0;

        if (shouldUseMultiTag) {
          xyStdDev = XY_STDDEV_MODEL.predict(avgDistance) * stabilityModifier * multiTagModifier;
          thetaStdDev = THETA_STDDEV_MODEL.predict(avgDistance) * stabilityModifier * multiTagModifier;
        } else {
          xyStdDev = XY_STDDEV_MODEL.predict(avgDistance) * stabilityModifier;
          thetaStdDev = THETA_STDDEV_MODEL.predict(avgDistance) * stabilityModifier;
        }

        Vector<N3> stdDevs = VecBuilder.fill(
            xyStdDev,
            xyStdDev,
            thetaStdDev * thetaModifier);

        if (!shouldUseMultiTag) {
          stdDevs.times(singleTagAdjustment);
        }

        visionUpdates.add(
            new TimestampedVisionUpdate(
                calculatedRobotPose,
                timestamp,
                stdDevs));

        logPoses(i, cameraPose, calculatedRobotPose, tagPose3ds.toArray(Pose3d[]::new));
        GremlinLogger.logSD(logPath + "/TagsUsed", tagPose3ds.size());
        GremlinLogger.logStdDevs(logPath + "/StdDevs", stdDevs);
      }
    }
  }

  private static final StructPublisher<Pose3d> FLcamPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic(CAMERA_LOG_PATH + "frontLeft" + "/Camera Pose", Pose3d.struct).publish();
  private static final StructPublisher<Pose2d> FLcalculatedPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic(CAMERA_LOG_PATH + "frontLeft" + "/Calculated Pose", Pose2d.struct).publish();
  private static final StructArrayPublisher<Pose3d> FLtagPosesPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic(CAMERA_LOG_PATH + "frontLeft" + "/Tag Poses", Pose3d.struct).publish();
  private static final StructPublisher<Pose3d> FRcamPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic(CAMERA_LOG_PATH + "frontRight" + "/Camera Pose", Pose3d.struct).publish();
  private static final StructPublisher<Pose2d> FRcalculatedPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic(CAMERA_LOG_PATH + "frontRight" + "/Calculated Pose", Pose2d.struct).publish();
  private static final StructArrayPublisher<Pose3d> FRtagPosesPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic(CAMERA_LOG_PATH + "frontRight" + "/Tag Poses", Pose3d.struct).publish();
  private static final StructPublisher<Pose3d> BLcamPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic(CAMERA_LOG_PATH + "backLeft" + "/Camera Pose", Pose3d.struct).publish();
  private static final StructPublisher<Pose2d> BLcalculatedPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic(CAMERA_LOG_PATH + "backLeft" + "/Calculated Pose", Pose2d.struct).publish();
  private static final StructArrayPublisher<Pose3d> BLtagPosesPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic(CAMERA_LOG_PATH + "backLeft" + "/Tag Poses", Pose3d.struct).publish();
  private static final StructPublisher<Pose3d> BRcamPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic(CAMERA_LOG_PATH + "backRight" + "/Camera Pose", Pose3d.struct).publish();
  private static final StructPublisher<Pose2d> BRcalculatedPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic(CAMERA_LOG_PATH + "backRight" + "/Calculated Pose", Pose2d.struct).publish();
  private static final StructArrayPublisher<Pose3d> BRtagPosesPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic(CAMERA_LOG_PATH + "backRight" + "/Tag Poses", Pose3d.struct).publish();

  /**
   * Log the CamPose, Calculated Pose, and TagPose
   * 
   * @param camID
   */
  private void logPoses(int camID, Pose3d camPose, Pose2d calculatedPose, Pose3d[] tagPoses) {
    switch (camID) {
      case 0:
        FLcamPosePublisher.set(camPose);
        FLcalculatedPosePublisher.set(calculatedPose);
        FLtagPosesPublisher.set(tagPoses);
      case 1:
        FRcamPosePublisher.set(camPose);
        FRcalculatedPosePublisher.set(calculatedPose);
        FRtagPosesPublisher.set(tagPoses);
      case 2:
        BLcamPosePublisher.set(camPose);
        BLcalculatedPosePublisher.set(calculatedPose);
        BLtagPosesPublisher.set(tagPoses);
      case 3:
        BRcamPosePublisher.set(camPose);
        BRcalculatedPosePublisher.set(calculatedPose);
        BRtagPosesPublisher.set(tagPoses);
    }
  }

  public void configSim() {
    visionSystemSim = new VisionSystemSim("ApriltagVision");
    AprilTagFieldLayout layout;
    try {
      layout = FieldConstants.isShopField ? FieldConstants.shopLayout
          : AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      visionSystemSim.addAprilTags(layout);
    } catch (IOException e) {
      e.printStackTrace();
    }

    simCameras = new PhotonCameraSim[cameras.length];
    simCameraProperties = new SimCameraProperties[cameras.length];

    for (int i = 0; i < cameras.length; i++) {
      simCameraProperties[i] = VisionConstants.getOV2311();
      simCameras[i] = new PhotonCameraSim(cameras[i].getPhotonCamera(), simCameraProperties[i]);
      simCameras[i].enableDrawWireframe(false);
      simCameras[i].enableRawStream(true); // (http://localhost:1181 / 1182)
      simCameras[i].enableProcessedStream(true);
      visionSystemSim.addCamera(simCameras[i], GeomUtil.pose3dToTransform3d(cameras[i].getCameraPose()));
    }
  }

  @Override
  public void simulationPeriodic() {
    visionSystemSim.update(poseSupplier.get());
    Field2d debugField = visionSystemSim.getDebugField();
    debugField.getObject("EstimatedRobot").setPose(poseSupplier.get());
    debugField.getRobotObject().setPose(poseSupplier.get());

    processVisionUpdates();
    visionConsumer.accept(visionUpdates);
    GremlinLogger.logSD("VISION/visionUpdatesSize", visionUpdates.size());
  }
}
