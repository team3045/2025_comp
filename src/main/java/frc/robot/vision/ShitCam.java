// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import static frc.robot.constants.DriveConstants.drive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static frc.robot.vision.VisionConstants.*;

/** Add your docs here. */
public class ShitCam {
    private int idx;
    private CommandSwerveDrivetrain drivetrain;
    private LimelightHelpers.PoseEstimate mostRecentPoseEstimate;
    public ShitCam(int Idx, CommandSwerveDrivetrain Drivetrain) {
        idx = Idx;
        drivetrain = Drivetrain;
        LimelightHelpers.setCameraPose_RobotSpace(names[idx], poses[idx].getTranslation().getX(), poses[idx].getTranslation().getY(), poses[idx].getTranslation().getZ(), poses[idx].getRotation().getX(), poses[idx].getRotation().getY(), poses[idx].getRotation().getZ());
    }
    
    public void process() {
        LimelightHelpers.SetRobotOrientation(names[idx], drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            mostRecentPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(names[idx]);
        } else {
            mostRecentPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(names[idx]);
        }
    }

    public void addVisionMeasurement() {
        if (mostRecentPoseEstimate == null) return;
        if(mostRecentPoseEstimate.tagCount == 0) {
            SmartDashboard.putBoolean("SeesApriltag", false);
            return;
        }
        SmartDashboard.putNumberArray("LimelightPose", new Double[]{mostRecentPoseEstimate.pose.getTranslation().getX(), mostRecentPoseEstimate.pose.getTranslation().getY(), mostRecentPoseEstimate.pose.getRotation().getDegrees()});
        SmartDashboard.putBoolean("SeesApriltag", true);
        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 999999.0));
        drivetrain.addVisionMeasurement(mostRecentPoseEstimate.pose, mostRecentPoseEstimate.timestampSeconds);
    }
}
