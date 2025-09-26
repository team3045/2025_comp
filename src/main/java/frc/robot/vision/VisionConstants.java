// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class VisionConstants {
    public static String[] names = new String[]{"limelight-right", "limelight-left"};
    public static Pose3d[] poses = new Pose3d[]{
        new Pose3d( // right
            new Translation3d(
                Units.inchesToMeters(8.055),
                Units.inchesToMeters(6.35),
                Units.inchesToMeters(6.9205)),
            new Rotation3d(0, Units.degreesToRadians(20), Units.degreesToRadians(7.5))),
        new Pose3d(
            new Translation3d(
                Units.inchesToMeters(8.055),
                Units.inchesToMeters(-6.35),
                Units.inchesToMeters(6.9205)),
            new Rotation3d(0, Units.degreesToRadians(20), Units.degreesToRadians(-7.5))) // left
        };
}
