package frc.robot;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.constants.CameraConstants;

public class Camera {
    private static PhotonCamera camera;
    private static PhotonPoseEstimator poseEstimator;
    private static Optional<EstimatedRobotPose> lastEstimatedPose;
    
    public Camera(int constantsIndex, Pose3d starterPose) {
        camera = new PhotonCamera(CameraConstants.cameraNames[constantsIndex]);
        poseEstimator = new PhotonPoseEstimator(CameraConstants.fieldLayout, CameraConstants.strategy, CameraConstants.cameraTransforms[constantsIndex]);
        lastEstimatedPose = Optional.empty();
        poseEstimator.setReferencePose(starterPose);
    }

    //runs the camera update function, also fetches the last estimated pose if the camera sees any tags
    //ONLY WORKS IF VALID STARTER POSITION PASSED INTO CONSTRUCTOR
    public static void updateCamera() {
        List<PhotonPipelineResult> result = camera.getAllUnreadResults();

        for (PhotonPipelineResult photonPipelineResult : result) {
            
            if (photonPipelineResult.hasTargets()) {
                Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(photonPipelineResult);
                if (estimatedPose.isEmpty() == false) { //we want the last valid pose, so we ignore any Optional.empty()
                    lastEstimatedPose = estimatedPose;
                }
            }
        }
    }

    //will return a Optional.empty() if no pose was found before, MUST CHECK
    public static Optional<EstimatedRobotPose> getLastEstimatedPose() {
        return lastEstimatedPose;
    }
}
