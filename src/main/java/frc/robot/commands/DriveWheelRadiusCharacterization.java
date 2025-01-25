// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveWheelRadiusCharacterization extends Command {
  private static final double characterizationSpeed = 0.5; //Rads Per Sec
  private static final double driveRadius = DriveConstants.drivebaseRadius;

  private DoubleSupplier gyroYawRadsSupplier;
  private CommandSwerveDrivetrain drivetrain; 
  

  public enum Direction {
    CLOCKWISE(-1),
    COUNTER_CLOCKWISE(1);

    private final int value;

    private Direction(int value){this.value = value;}
  }

  private Direction omegaDirection;
  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

  private double lastGyroYawRads = 0.0;
  private double accumGyroYawRads = 0.0;

  private double[] startWheelPositions;

  private double currentEffectiveWheelRadius = 0.0;


  /** Creates a new DriveWheelRadiusCharacterization. */
  public DriveWheelRadiusCharacterization(CommandSwerveDrivetrain drive, Direction direction) {
    this.drivetrain = drive;
    this.gyroYawRadsSupplier = () -> drivetrain.getRawHeadingRadians();
    this.omegaDirection = direction;


    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     // Reset
     lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
     accumGyroYawRads = 0.0;
 
     startWheelPositions = drivetrain.getWheelPositionsRadians();
 
     omegaLimiter.reset(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.turnAtRotationalRate(
      omegaLimiter.calculate(omegaDirection.value * characterizationSpeed));

    // Get yaw and wheel positions
    accumGyroYawRads += MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    double averageWheelPosition = 0.0;
    double[] wheelPositiions = drivetrain.getWheelPositionsRadians();
    for (int i = 0; i < 4; i++) {
      averageWheelPosition += Math.abs(wheelPositiions[i] - startWheelPositions[i]);
    }
    averageWheelPosition /= 4.0;

    currentEffectiveWheelRadius = (accumGyroYawRads * driveRadius) / averageWheelPosition;
    SmartDashboard.putNumber("RadiusCharacterization/DrivePosition", averageWheelPosition);
    SmartDashboard.putNumber("RadiusCharacterization/AccumGyroYawRads", accumGyroYawRads);
    SmartDashboard.putNumber("RadiusCharacterization/effectiveWheelRadius", Units.metersToInches(currentEffectiveWheelRadius));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(DriveConstants.brake);
    if (accumGyroYawRads <= Math.PI * 2.0) {
      System.out.println("Not enough data for characterization");
    } else {
      System.out.println(
          "Effective Wheel Radius: "
              + Units.metersToInches(currentEffectiveWheelRadius)
              + " inches");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
