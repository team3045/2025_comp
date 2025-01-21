// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.DriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveWheelRadiusCharacterization extends Command {
  private static final double characterizationSpeed = 0.1; //Rads Per Sec
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
    this.gyroYawRadsSupplier = () -> drivetrain.getState().Pose.getRotation().getRadians();
    this.omegaDirection = direction;


    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     // Reset
     lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
     accumGyroYawRads = 0.0;
 
     startWheelPositions = null;
 
     omegaLimiter.reset(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
