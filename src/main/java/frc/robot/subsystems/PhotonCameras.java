// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Camera;
import frc.robot.constants.CameraConstants;

public class PhotonCameras extends SubsystemBase {
  /** Creates a new PhotonCamera. */
  private static final Camera[] cameras = {new Camera(0), new Camera(1), new Camera(2)};
  
  public PhotonCameras() {
    
  }
}
