// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commons.GeomUtil;
import frc.robot.constants.AutoScoreConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorPivot;

import static frc.robot.constants.AutoScoreConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FullAutoScore extends SequentialCommandGroup {
  private CommandSwerveDrivetrain drivetrain;
  private ElevatorPivot elevatorPivot;
  private Claw claw;

  private IntegerSubscriber poleNumberSub = NetworkTableInstance.getDefault().getTable("Scoring Location")
    .getIntegerTopic("Pole").subscribe(0);
  private IntegerSubscriber heightSub = NetworkTableInstance.getDefault().getTable("Scoring Location")
    .getIntegerTopic("Height").subscribe(0);



  /** Creates a new FullAutoScore. */
  public FullAutoScore(CommandSwerveDrivetrain drivetrain, ElevatorPivot elevatorPivot, Claw clawRef) {
    this.drivetrain = drivetrain;
    this.elevatorPivot = elevatorPivot;
    this.claw = clawRef;

    addCommands(getPathFindCommand(), getPrecisePidCommand(), setElevatorHeight(), claw.clawOutake().withTimeout(0.3));
  }


  public Command getPathFindCommand(){
    if(poleNumberSub.get() == 0){
      System.out.println("Hi");
      return Commands.none();
    }

    //Get values from GUI application
    Pose2d targetPose = kScorePoseMap.get((int) poleNumberSub.get());
    return AutoBuilder.pathfindToPose(
      targetPose, 
      DriveConstants.pathFollowingConstraints, 
      AutoScoreConstants.kMaxVelError).until( //TODO: change target velocity to the predicted PID value
        () -> GeomUtil.isNearPose(targetPose, drivetrain.getState().Pose, kMaxPathFindTranslationError)); 
  }

  public Command getPrecisePidCommand(){
    if(poleNumberSub.get() == 0){
      return Commands.none();
    }

    Pose2d targetPose = kScorePoseMap.get((int) poleNumberSub.get());

    return drivetrain.preciseTargetPose(targetPose);
  }

  public Command setElevatorHeight(){
    if(poleNumberSub.get() == 0){
      return Commands.none();
    }

    double targetHeight = kScoreHeightMap.get((int) heightSub.get());
    double targetAngle = kScoreAngleMap.get((int) heightSub.get());

    return elevatorPivot.goToPosition(() -> targetHeight, () -> targetAngle);
  }
}
