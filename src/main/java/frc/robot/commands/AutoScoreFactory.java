// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.constants.DriveConstants.drive;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.GremlinRobotState;
import frc.robot.GremlinRobotState.DriveState;
import frc.robot.constants.AutoScoreConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ElevatorPivotConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorPivot;
import frc.robot.vision.apriltag.GremlinLimelightCamera;
import frc.robot.vision.apriltag.VisionConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScoreFactory{
  public static final GremlinRobotState M_ROBOT_STATE = GremlinRobotState.getRobotState();
  private CommandSwerveDrivetrain drivetrain;
  private ElevatorPivot elevatorPivot;
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

  public Command setElevatorHeight(Supplier<Integer> heightLevel){
    return elevatorPivot.goToPosition(
      () -> AutoScoreConstants.kScoreHeightMap.getOrDefault((int) heightLevel.get(), elevatorPivot.getHeight()),
      () -> AutoScoreConstants.kScoreAngleMap.getOrDefault((int) heightLevel.get(), elevatorPivot.getPivotAngleDegrees()));
  }

  public DynamicPathfindWithFeedback pathfindToScoring(GremlinLimelightCamera rightFeedbackCamera, GremlinLimelightCamera leftFeedbackCamera){
    return pathFindWithApriltagFeeback(
      () -> AutoScoreConstants.kScorePoseMap.getOrDefault((int) poleNumberSub.get(), drivetrain.getState().Pose), 
      rightFeedbackCamera, 
      leftFeedbackCamera);
  }

  public DynamicPathfindWithFeedback pathFindWithApriltagFeeback(Supplier<Pose2d> desiredPose, GremlinLimelightCamera rightFeedbackCamera, GremlinLimelightCamera leftFeedbackCamera){

    rightFeedbackCamera.setValidIDsMT2(AutoScoreConstants.kReefAprilTagIds);
    leftFeedbackCamera.setValidIDsMT2(AutoScoreConstants.kReefAprilTagIds);

    //Basically we alternate between right or left cameras, depending on the pole number.
    //Odd pole numbers use rightCamera, Even pole numbers use leftCamera
    Supplier<Pose2d> robotPoseSupplier = () -> {
      int poleNumber =(int) poleNumberSub.get();

      Pose2d targetPose = AutoScoreConstants.kScorePoseMap.getOrDefault((int) poleNumberSub.get(), drivetrain.getState().Pose);

      SmartDashboard.putNumberArray("targetposition", new double[]{targetPose.getX(), targetPose.getY()});
      SmartDashboard.putNumber("polenum", (int) poleNumberSub.get());

      if(poleNumber % 2 == 0){
        return leftFeedbackCamera.getBotPoseEstimateMT2().isPresent() ? leftFeedbackCamera.getBotPoseEstimateMT2().get().pose : drivetrain.getState().Pose;
      } else {
        return rightFeedbackCamera.getBotPoseEstimateMT2().isPresent() ? rightFeedbackCamera.getBotPoseEstimateMT2().get().pose : drivetrain.getState().Pose;
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
        return leftFeedbackCamera.getBotPoseEstimateMT2().isPresent() ? leftFeedbackCamera.getBotPoseEstimateMT2().get().timestampSeconds : Utils.getCurrentTimeSeconds();
      } else {
        return rightFeedbackCamera.getBotPoseEstimateMT2().isPresent() ? rightFeedbackCamera.getBotPoseEstimateMT2().get().timestampSeconds : Utils.getCurrentTimeSeconds();
      }
    };

    return new DynamicPathfindWithFeedback(
      desiredPose,
      () -> 0, 
      DriveConstants.autoScoreConstraints, 
      drivetrain, 
      robotPoseSupplier, 
      shouldOverride,
      timestampSupplier);
  }

  public Command fullAutoScoreCommand(){
    return pathfindToScoring(VisionConstants.limelights[0], VisionConstants.limelights[1]) //righ and left
        .alongWith(setElevatorHeight())
        .andThen(claw.clawOutake())
        .andThen(Commands.waitSeconds(0.4))
        .andThen(drivetrain.driveBack())
        .finallyDo(() -> {
            M_ROBOT_STATE.setDriveState(DriveState.TELEOP);
        }); //REDENDUNCY TO ALWAYS SET BACK TO TELEOP AFTER SCORE
  }

  public Command fullAutoScoreCommand(Supplier<Pose2d> desiredPose, Supplier<Integer> desiredHeight){
    return pathFindWithApriltagFeeback(
      desiredPose, VisionConstants.limelights[0], VisionConstants.limelights[1])
      .alongWith(setElevatorHeight(desiredHeight))
      .andThen(claw.clawOutake())
      .andThen(Commands.waitSeconds(0.4))
      .andThen(drivetrain.driveBack())
      .finallyDo(() -> {
        M_ROBOT_STATE.setDriveState(DriveState.TELEOP);
    }); //REDENDUNCY TO ALWAYS SET BACK TO TELEOP AFTER SCORE
  }


  public Command goToNearestAlgea(Supplier<Pose2d> poseSupplier, GremlinLimelightCamera feedbackCamera){
    Supplier<Pose2d> targetPoseSupplier = () -> {
      List<Pose2d> poseList = AutoBuilder.shouldFlip() ? FieldConstants.flippedAlgeaPoses : FieldConstants.algeaPoses;

      Pose2d closest = poseList.get(0);
      int closestNum = 0;

      for(int i = 1; i < poseList.size(); i++){
        if(
          poseList.get(i).getTranslation().getDistance(poseSupplier.get().getTranslation()) < 
          closest.getTranslation().getDistance(poseSupplier.get().getTranslation())) 
        {
          closest = poseList.get(i);
          closestNum = i;
        }
      }

      return FieldConstants.algeaPoses.get(closestNum);
    };

    return new DynamicPathfindWithFeedback(
      targetPoseSupplier, 
      () -> 0, 
      DriveConstants.autoScoreConstraints, 
      drivetrain,
      () -> feedbackCamera.getBotPoseEstimateMT2().isPresent() ? feedbackCamera.getBotPoseEstimateMT2().get().pose : drivetrain.getState().Pose,
      feedbackCamera::seesObject,
      () -> feedbackCamera.getBotPoseEstimateMT2().isPresent() ? feedbackCamera.getBotPoseEstimateMT2().get().timestampSeconds : Utils.getCurrentTimeSeconds()
    );
  }

  public Command getAlgeaRemoveCommand(GremlinLimelightCamera feedbackCamera, DoubleSupplier xSpeeds, DoubleSupplier ySpeeds){
    DoubleSupplier heightSupplier = () -> {
      List<Pose2d> poseList = AutoBuilder.shouldFlip() ? FieldConstants.flippedAlgeaPoses : FieldConstants.algeaPoses;

              Pose2d closest = poseList.get(0);
              int closestNum = 0;
        
              for(int i = 1; i < poseList.size(); i++){
                if(
                  poseList.get(i).getTranslation().getDistance(drivetrain.getState().Pose.getTranslation()) < 
                  closest.getTranslation().getDistance(drivetrain.getState().Pose.getTranslation())) 
                {
                  closest = poseList.get(i);
                  closestNum = i;
                }
              }

              if(closestNum % 2 == 0){
                return ElevatorPivotConstants.HeightPositions.HIGH_ALGEA.getHeight();
              } else {
                return ElevatorPivotConstants.HeightPositions.LOW_ALGEA.getHeight();
              }
    };

    DoubleSupplier AngleSupplier = () -> {
      List<Pose2d> poseList = AutoBuilder.shouldFlip() ? FieldConstants.flippedAlgeaPoses : FieldConstants.algeaPoses;

              Pose2d closest = poseList.get(0);
              int closestNum = 0;
        
              for(int i = 1; i < poseList.size(); i++){
                if(
                  poseList.get(i).getTranslation().getDistance(drivetrain.getState().Pose.getTranslation()) < 
                  closest.getTranslation().getDistance(drivetrain.getState().Pose.getTranslation())) 
                {
                  closest = poseList.get(i);
                  closestNum = i;
                }
              }

              if(closestNum % 2 == 0){
                return ElevatorPivotConstants.AnglePositions.HIGH_ALGEA.getAngle();
              } else {
                return ElevatorPivotConstants.AnglePositions.LOW_ALGEA.getAngle();
              }
    };

    return drivetrain.driveFacingAlgea(xSpeeds, ySpeeds)
      .alongWith(
        elevatorPivot.goToPosition(heightSupplier, AngleSupplier))
      .alongWith(claw.algeaIntake()).until(ElevatorPivot.hasAlgea)
      .andThen(drivetrain.driveBackAlgea());
  }

  public Command AutonomousPeriodAutoScore(Supplier<Integer> heightSup, GremlinLimelightCamera leftFeedbackCamera, GremlinLimelightCamera rightFeedbackCamera){
    //Basically we alternate between right or left cameras, depending on the pole number.
    //Odd pole numbers use rightCamera, Even pole numbers use leftCamera
    Supplier<Pose2d> robotPoseSupplier = () -> {
      int poleNumber =(int) poleNumberSub.get();

      Pose2d targetPose = AutoScoreConstants.kScorePoseMap.getOrDefault((int) poleNumberSub.get(), drivetrain.getState().Pose);

      SmartDashboard.putNumberArray("targetposition", new double[]{targetPose.getX(), targetPose.getY()});
      SmartDashboard.putNumber("polenum", (int) poleNumberSub.get());

      if(poleNumber % 2 == 0){
        return leftFeedbackCamera.getBotPoseEstimateMT2().isPresent() ? leftFeedbackCamera.getBotPoseEstimateMT2().get().pose : drivetrain.getState().Pose;
      } else {
        return rightFeedbackCamera.getBotPoseEstimateMT2().isPresent() ? rightFeedbackCamera.getBotPoseEstimateMT2().get().pose : drivetrain.getState().Pose;
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
        return leftFeedbackCamera.getBotPoseEstimateMT2().isPresent() ? leftFeedbackCamera.getBotPoseEstimateMT2().get().timestampSeconds : Utils.getCurrentTimeSeconds();
      } else {
        return rightFeedbackCamera.getBotPoseEstimateMT2().isPresent() ? rightFeedbackCamera.getBotPoseEstimateMT2().get().timestampSeconds : Utils.getCurrentTimeSeconds();
      }
    };
    
    return Commands.run(
        () -> {
          if(shouldOverride.getAsBoolean()){
            drivetrain.addVisionMeasurement(
              robotPoseSupplier.get(), 
              Utils.fpgaToCurrentTime(timestampSupplier.getAsDouble()),
              VecBuilder.fill(0.001,0.001,0.001));
          }
        }
    ).alongWith(setElevatorHeight(heightSup))
      .andThen(claw.clawOutake())
      .andThen(Commands.waitSeconds(0.4))
      .andThen(drivetrain.driveBack());
  }
}