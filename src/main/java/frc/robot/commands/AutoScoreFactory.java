// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commons.GeomUtil;
import frc.robot.constants.AutoScoreConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorPivot;

import static frc.robot.constants.AutoScoreConstants.*;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScoreFactory{
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

  public Command setElevatorHeight(Supplier<Integer> heightSupplier){
    // if(poleNumberSub.get() == 0){
    //   return Commands.none();
    // }

    // double targetHeight = kScoreHeightMap.get((int) heightSub.get());
    // double targetAngle = kScoreAngleMap.get((int) heightSub.get());

    // return elevatorPivot.goToPosition(() -> targetHeight, () -> targetAngle);
    int kHeightValue = heightSupplier.get();
    return Commands.print("Elevator Height Number: " + kHeightValue);
  }
}
