package frc.robot.vision.oakD;

import edu.wpi.first.math.geometry.Pose2d;

//Data Structure to Store the Object Pose Updates
public record TimestampedObjectPose(Pose2d pose, double timestamp){} 