// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AutoScoreConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorPivot;
import frc.robot.vision.apriltag.GremlinLimelightCamera;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScoreFactory{
  private CommandSwerveDrivetrain drivetrain;
  private ElevatorPivot elevatorPivot;
  @SuppressWarnings("unused")
  private Claw claw;

  private IntegerSubscriber poleNumberSub = NetworkTableInstance.getDefault().getTable("Scoring Location")
    .getIntegerTopic("Pole").subscribe(0);
  private IntegerSubscriber heightSub = NetworkTableInstance.getDefault().getTable("Scoring Location")
    .getIntegerTopic("Height").subscribe(0);



  /** Creates a new FullAutoScore. */
  public AutoScoreFactory(CommandSwerveDrivetrain drivetrain, ElevatorPivot elevatorPivot, Claw clawRef) {
    this.drivetrain = drivetrain;
    this.elevatorPivot = elevatorPivot;
    this.claw = clawRef;
  }


  public Command getPathFindCommand(){
    //Get values from GUI application
    return drivetrain.pathFindToPose(
      () -> AutoScoreConstants.kScorePoseMap.getOrDefault((int) poleNumberSub.get(), drivetrain.getState().Pose), 
      () -> 0); 
  }

  public Command getPrecisePidCommand(){
    return drivetrain.preciseTargetPose(
      () -> AutoScoreConstants.kScorePoseMap.getOrDefault((int) poleNumberSub.get(), drivetrain.getState().Pose));
  }

  public Command setElevatorHeight(){
    return elevatorPivot.goToPosition(
      () -> AutoScoreConstants.kScoreHeightMap.getOrDefault((int) heightSub.get(), elevatorPivot.getHeight()),
      () -> AutoScoreConstants.kScoreAngleMap.getOrDefault((int) heightSub.get(), elevatorPivot.getPivotAngleDegrees()));
  }

  public DynamicPathfindWithFeedback pathFindWithApriltagFeeback(GremlinLimelightCamera rightFeedbackCamera, GremlinLimelightCamera leftFeedbackCamera){

    rightFeedbackCamera.setValidIDsMT2(AutoScoreConstants.kReefAprilTagIds);
    leftFeedbackCamera.setValidIDsMT2(AutoScoreConstants.kReefAprilTagIds);

    //Basically we alternate between right or left cameras, depending on the pole number.
    //Odd pole numbers use rightCamera, Even pole numbers use leftCamera
    Supplier<Pose2d> targetPoseSupplier = () -> {
      int poleNumber =(int) poleNumberSub.get();
      if(poleNumber % 2 == 0){
        return leftFeedbackCamera .getBotPoseEstimateMT2().pose;
      } else {
        return rightFeedbackCamera.getBotPoseEstimateMT2().pose;
      }
    };

    BooleanSupplier shouldOverride = () -> {
      int poleNumber = (int) poleNumberSub.get();

      if(poleNumber % 2 == 0){
        return leftFeedbackCamera.seesObject() && drivetrain.withinDistanceOfReef(FieldConstants.reefDistanceTolerance);
      } else {
        return rightFeedbackCamera.seesObject() && drivetrain.withinDistanceOfReef(FieldConstants.reefDistanceTolerance);
      }
    };

    DoubleSupplier timestampSupplier = () -> {
      int poleNumber = (int) poleNumberSub.get();

      if(poleNumber % 2 == 0){
        return leftFeedbackCamera.getBotPoseEstimateMT2().timestampSeconds;
      } else {
        return rightFeedbackCamera.getBotPoseEstimateMT2().timestampSeconds;
      }
    };

    return new DynamicPathfindWithFeedback(
      () -> AutoScoreConstants.kScorePoseMap.getOrDefault((int) poleNumberSub.get(), drivetrain.getState().Pose), 
      () -> 0, 
      DriveConstants.autoScoreConstraints, 
      drivetrain, 
      targetPoseSupplier, 
      shouldOverride,
      timestampSupplier);
  }
  
}
