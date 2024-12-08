package frc.robot;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.constants.CameraConstants;

public class Camera {
    private static PhotonCamera camera;
    private static PhotonPoseEstimator poseEstimator;
    
    public Camera(int constantsIndex) {
        camera = new PhotonCamera(CameraConstants.cameraNames[constantsIndex]);
        poseEstimator = new PhotonPoseEstimator(CameraConstants.fieldLayout, CameraConstants.strategy, CameraConstants.cameraTransforms[constantsIndex]);
    }

    public static void updateCamera() {
        List<PhotonPipelineResult> result = camera.getAllUnreadResults();

        for (PhotonPipelineResult photonPipelineResult : result) {
            if (photonPipelineResult.hasTargets()) {
                poseEstimator.update(photonPipelineResult);
            }
        }
    }
}
