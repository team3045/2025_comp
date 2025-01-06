// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.oakD;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ObjectDetConstants {
    public static final double timeThreshold = 1.5; //one and a half seconds
    public static final double distanceTreshold = 0.5; //half meter
    public static final Translation2d fieldSize = new Translation2d(16.54, 8.21);
    

    public static final Pose3d OakDPose = new Pose3d(
        new Translation3d(Units.inchesToMeters(-2.5), Units.inchesToMeters(13.5), Units.inchesToMeters(6.5)),
        new Rotation3d(0, -Math.PI/2, 0)
    );
}
