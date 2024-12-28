// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

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
import static frc.robot.vision.VisionConstants.CAMERA_LOG_PATH;
import static frc.robot.vision.VisionConstants.EXCLUDED_TAG_IDS;
import static frc.robot.vision.VisionConstants.FIELD_BORDER_MARGIN;
import static frc.robot.vision.VisionConstants.MAX_AMBIGUITY;
import static frc.robot.vision.VisionConstants.THETA_STDDEV_MODEL;
import static frc.robot.vision.VisionConstants.XY_STDDEV_MODEL;
import static frc.robot.vision.VisionConstants.maxChangeDistance;
import static frc.robot.vision.VisionConstants.multiTagModifier;
import static frc.robot.vision.VisionConstants.regressionModifier;
import static frc.robot.vision.VisionConstants.thetaModifier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
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


  //Will be the function in drivetrain that adds vision estimate to pose estimation
  private Consumer<List<TimestampedVisionUpdate>> visionConsumer = (visionUpdates) -> {};
  //Will be the function in driveTrain that supplies current pose estimate
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
      // Camera specific variables
      Transform3d camToRobotTransform = GeomUtil.pose3dToTransform3d(cameras[i].getCameraPose()).inverse();
      List<PhotonPipelineResult> unreadResults = cameras[i].getAllUnreadResults();

      for(int j = 0; j < unreadResults.size(); j++){
        Pose3d cameraPose;
        Pose2d calculatedRobotPose;
        List<Pose3d> tagPose3ds = new ArrayList<>();
        PhotonPipelineResult unprocessedResult = unreadResults.get(j);
        double timestamp = unprocessedResult.getTimestampSeconds();
        double singleTagAdjustment = 1.0;
        String logPath = CAMERA_LOG_PATH + cameras[i].getName();

        GremlinLogger.logSD(logPath + "/Hastargets", unprocessedResult.hasTargets());
        GremlinLogger.logSD(logPath + "/Timestamp", timestamp);

        // Continue if the camera doesn't have any targets
        if (!unprocessedResult.hasTargets()) {
          continue;
        }

        // if it has a MultiTag result we prefer to use that
        boolean shouldUseMultiTag = unprocessedResult.getMultiTagResult().isPresent();

        if (shouldUseMultiTag && false) {
          // TODO: think about adding processing to compare best and alt
          cameraPose = GeomUtil.transform3dToPose3d(unprocessedResult.getMultiTagResult().get().estimatedPose.best);

          calculatedRobotPose = cameraPose.transformBy(camToRobotTransform).toPose2d();

          // Populate array of tag poses with tags used
          for (int id : unprocessedResult.getMultiTagResult().get().fiducialIDsUsed) {
            tagPose3ds.add(LAYOUT.getTagPose(id).get());
            //TODO: add logs of each tag here
          }

          GremlinLogger.logSD(logPath + "/CameraPose (MultiTag)", cameraPose.toPose2d());
        } else {
          PhotonTrackedTarget target = unprocessedResult.getBestTarget();

          //We dont like some tags
          if(EXCLUDED_TAG_IDS.contains(target.getFiducialId())) continue;

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

          GremlinLogger.logSD(logPath + "/CameraPose (SingleTag)", cameraPose.toPose2d());
          GremlinLogger.logSD(logPath + "/Transform (Single Tag)", target.getBestCameraToTarget().inverse());
          GremlinLogger.logSD(logPath + "/TagPose", tagPose);
        }

        if (cameraPose == null || calculatedRobotPose == null)
          continue;

        // Move on to next camera if robot pose is off the field
        // if (calculatedRobotPose.getX() < -FIELD_BORDER_MARGIN
        //     || calculatedRobotPose.getX() > fieldLength+ FIELD_BORDER_MARGIN
        //     || calculatedRobotPose.getY() < -FIELD_BORDER_MARGIN
        //     || calculatedRobotPose.getY() > fieldWidth + FIELD_BORDER_MARGIN) {
        //   continue;
        // }

        if(calculatedRobotPose.getTranslation()
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
          xyStdDev = XY_STDDEV_MODEL.predict(avgDistance) * regressionModifier * multiTagModifier;
          thetaStdDev = THETA_STDDEV_MODEL.predict(avgDistance) * regressionModifier * multiTagModifier;
        } else {
          xyStdDev = XY_STDDEV_MODEL.predict(avgDistance) * regressionModifier;
          thetaStdDev = THETA_STDDEV_MODEL.predict(avgDistance) * regressionModifier;
        }

        Vector<N3> stdDevs = VecBuilder.fill(
          xyStdDev,
          xyStdDev,
          thetaStdDev * thetaModifier
        );

        if (!shouldUseMultiTag) {
          stdDevs.times(singleTagAdjustment);
        }


        
        visionUpdates.add(
          new TimestampedVisionUpdate(
            calculatedRobotPose, 
            timestamp, 
            stdDevs));

        GremlinLogger.logSD(logPath + "/VisionPose", calculatedRobotPose);
        GremlinLogger.logSD(logPath + "/TagsUsed", tagPose3ds.size());
        GremlinLogger.logStdDevs(logPath + "/StdDevs", stdDevs);
      }
    }
  }

  public void configSim() {
    visionSystemSim = new VisionSystemSim("ApriltagVision");
    AprilTagFieldLayout layout;
    try {
      layout = FieldConstants.isShopField ? FieldConstants. shopLayout : 
        AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
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
  }
}
