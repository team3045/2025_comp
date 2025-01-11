// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorPivot;
import frc.robot.commons.AutoScoreState;
import frc.robot.commons.GremlinUtil;
import frc.robot.constants.AutoScoreConstants;
import frc.robot.constants.DriveConstants;

import static frc.robot.constants.DriveConstants.*;

import java.util.Arrays;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoScore extends Command {
  /** Creates a new AutoScore. */
  private AutoScoreState m_ScoreState;
  private CommandSwerveDrivetrain m_DrivetrainRef;
  private ElevatorPivot m_ElevatorRef;
  private boolean m_Scored = false;
  private boolean m_Reset = false;

  private Command m_PathfindingCommand = AutoBuilder.pathfindToPose(
    Pose2d.kZero,
    DriveConstants.pathFollowingConstraints,
    AutoScoreConstants.kMaxVelError // Goal end velocity in meters/sec
  );

  public AutoScore(AutoScoreState scoreState, CommandSwerveDrivetrain drivetrainRef, ElevatorPivot elevatorRef) {
    m_ScoreState = scoreState;
    m_DrivetrainRef = drivetrainRef;
    m_ElevatorRef = elevatorRef;

    RobotConfig config = new RobotConfig(0, 0, null, Translation2d.kZero);
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace(); //if this doesn't load then the entire path planner system is screwed
    }
    
    AutoBuilder.configure(
      () -> {return m_DrivetrainRef.getState().Pose;},
      (Pose2d pose) -> {m_DrivetrainRef.resetPose(pose);},
      () -> {return m_DrivetrainRef.getState().Speeds;},
      (ChassisSpeeds speeds, DriveFeedforwards feedForward) -> {drivetrainRef.driveRobotRelative(speeds);},
      pathFollowingController,
      config,
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      m_DrivetrainRef);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Going to target (kinda shitty gotta figure out how to use path planner)
    Pose2d targetPose = Pose2d.kZero;
    if (m_ScoreState.m_PoseIDX.isEmpty()) {
      targetPose = m_DrivetrainRef.getState().Pose.nearest(Arrays.asList(AutoScoreConstants.kAllScorePoses));
    } else {
      targetPose = AutoScoreConstants.kAllScorePoses[m_ScoreState.m_PoseIDX.get()];
    }

    m_PathfindingCommand = AutoBuilder.pathfindToPose(
      targetPose,
      DriveConstants.pathFollowingConstraints,
      AutoScoreConstants.kMaxVelError // Goal end velocity in meters/sec
    );

    m_PathfindingCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_PathfindingCommand.isFinished()) {
      if (m_Scored == false) {
        //Scoring
        m_ElevatorRef.goToHeight(() -> {return AutoScoreConstants.kElevatorHeights[m_ScoreState.m_ScoreLevel];}); //Going to the target height
        if (m_ElevatorRef.atTargetHeight()) {
          //Pivot and score
          m_ElevatorRef.goToAngleDegrees(() -> {return AutoScoreConstants.kPivotAngles[m_ScoreState.m_ScoreLevel];});
          if (m_ElevatorRef.atTargetAngle()) {
            //Eject coral, scoring is done
            m_Scored = true;

            m_PathfindingCommand = AutoBuilder.pathfindToPose(
              GremlinUtil.movePoseForward(m_DrivetrainRef.getState().Pose, AutoScoreConstants.kBackUpDist),
              DriveConstants.pathFollowingConstraints,
              AutoScoreConstants.kMaxVelError // Goal end velocity in meters/sec
            );

            m_PathfindingCommand.schedule();
          }
        }
      } else {
        m_ElevatorRef.goToAngleDegrees(() -> {return AutoScoreConstants.kPivotResetAngle;});
        m_ElevatorRef.goToHeight(() -> {return AutoScoreConstants.kElevatorResetHeight;});
        m_Reset = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Reset;
  }
}
