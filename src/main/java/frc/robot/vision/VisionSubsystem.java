// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  public static ShitCam camRight;
  public static ShitCam camLeft;
  public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
    camRight = new ShitCam(0, drivetrain);
    // camLeft = new ShitCam(1, drivetrain);
    camLeft = null;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    camRight.process();
    camRight.addVisionMeasurement();
    // camLeft.process();
    // camLeft.addVisionMeasurement();
  }
}
