// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commons.GeomUtil;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPose extends Command {
  private CommandSwerveDrivetrain drivetrain;
  private Supplier<Pose2d> targetPoseSupplier;
  private Pose2d targetPose;
  private PIDController translationController;
  private PIDController rotationController;
  private double translationTolerance;
  private double rotationTolerance;

  /** Creates a new PreciseDriveToPose. */
  public DriveToPose(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> poseSupplier, PIDController translationController, 
    PIDController rotationController, double translationToleranceMeters, double rotationToleranceDegrees) {
    this.drivetrain = drivetrain;
    this.targetPoseSupplier = poseSupplier;
    this.translationController = translationController;
    this.rotationController = rotationController;
    this.translationTolerance = translationToleranceMeters;
    this.rotationTolerance = rotationToleranceDegrees;

    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetPose = targetPoseSupplier.get();
    translationController.setTolerance(translationTolerance);
    rotationController.setTolerance(rotationTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xApplied = -translationController.calculate(drivetrain.getState().Pose.getX(), targetPose.getX());
    double yApplied = -translationController.calculate(drivetrain.getState().Pose.getY(), targetPose.getY());
    double omegaApplied = -rotationController.calculate(drivetrain.getState().Pose.getRotation().getDegrees(), targetPose.getRotation().getDegrees());

    drivetrain.setControl(
      DriveConstants.drive.withVelocityX(xApplied) // Drive forward with negative Y (forward)
                    .withVelocityY(yApplied)// Drive left with negative X (left)
                    .withRotationalRate(omegaApplied) // Drive counterclockwise with negative X (left)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!interrupted)
      drivetrain.setControl(DriveConstants.brake);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return GeomUtil.isNearPoseWithRotation(targetPose, drivetrain.getState().Pose, translationTolerance, rotationTolerance);
  }
}
