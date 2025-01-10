// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.commons.AutoScoreState;
import frc.robot.commons.GremlinUtil;
import frc.robot.constants.AutoScoreConstants;

import static frc.robot.constants.DriveConstants.*;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoScore extends Command {
  /** Creates a new AutoScore. */
  private AutoScoreState m_ScoreState;
  private CommandSwerveDrivetrain m_DrivetrainRef;
  private Elevator m_ElevatorRef;
  private double smallestDist = 100000;
  private boolean m_AtPostition = false;
  private Transform2d m_MovementVector;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(0).withRotationalDeadband(0) // Add a 5% deadband
    .withDriveRequestType(DriveRequestType.Velocity); // Use close-loop control for drive motors

  public AutoScore(AutoScoreState scoreState, CommandSwerveDrivetrain drivetrainRef, Elevator elevatorRef) {
    m_ScoreState = scoreState;
    m_DrivetrainRef = drivetrainRef;
    m_ElevatorRef = elevatorRef;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_AtPostition) {
      //Going to target (kinda shitty gotta figure out how to use path planner)
      Pose2d poseTarget = Pose2d.kZero;
      if (m_ScoreState.m_PoseIDX.isEmpty()) {
        for (Pose2d pose : AutoScoreConstants.kAllScorePoses) {
          double dist = m_DrivetrainRef.getState().Pose.getTranslation().getDistance(pose.getTranslation());
          if (dist < smallestDist) {
            smallestDist = dist;
            poseTarget = pose;
          }
        }
      } else {
        poseTarget = AutoScoreConstants.kAllScorePoses[m_ScoreState.m_PoseIDX.get()];
      }

      //Calculates the desired movement vector (ignores obstacles, need to switch to path planner)
      m_MovementVector = m_DrivetrainRef.getState().Pose.minus(poseTarget);

      if ((Math.abs(m_MovementVector.getX()) <= AutoScoreConstants.kMaxError) || (Math.abs(m_MovementVector.getY()) <= AutoScoreConstants.kMaxError) || (Math.abs(m_MovementVector.getRotation().getMeasure().magnitude()) <= AutoScoreConstants.kMaxError)) {
        m_AtPostition = true;
      }

      //Makes the drivetrain request
      m_DrivetrainRef.applyRequest(() -> drive.withVelocityX(GremlinUtil.clamp(m_MovementVector.getX(), -MaxSpeed, MaxSpeed)).
        withVelocityY(GremlinUtil.clamp(m_MovementVector.getY(), -MaxSpeed, MaxSpeed)).
        withRotationalRate(GremlinUtil.clamp(m_MovementVector.getRotation().getMeasure().magnitude(), -MaxAngularRate, MaxAngularRate))
      );
    } else {
      //Scoring
      m_ElevatorRef.goToHeight(() -> {return AutoScoreConstants.kElevatorHeights[m_ScoreState.m_ScoreLevel];}); //Going to the target height
      if (m_ElevatorRef.atTargetHeight()) {
        //Pivot and score
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
