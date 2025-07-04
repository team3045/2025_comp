// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commons.GremlinPathFinder;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public static final Pathfinder pathfinder = new GremlinPathFinder();

  private final RobotContainer m_robotContainer;

  public Robot() {
    Pathfinding.setPathfinder(pathfinder);
    m_robotContainer = new RobotContainer();
   // m_robotContainer.drivetrain.resetPose(new Pose2d(5.23,2.55, Rotation2d.fromDegrees(123)));
    m_robotContainer.drivetrain.resetPose(new Pose2d(1.73,4.08, Rotation2d.kZero));
    // m_robotContainer.drivetrain.resetPose(new Pose2d(10.1,3.7,Rotation2d.kZero));
    //m_robotContainer.drivetrain.resetPose(new Pose2d(7.121, 1.310, Rotation2d.k180deg));
    //m_robotContainer.drivetrain.resetPose(new Pose2d(3.17, 3.7, Rotation2d.kZero)); //auto
    //auto
    //m_robotContainer.drivetrain.resetPose(new Pose2d(1.427,0.794,Rotation2d.fromDegrees(55)));
    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  @Override
  public void robotInit() {
    PathfindingCommand.warmupCommand().schedule();
    FollowPathCommand.warmupCommand().schedule();

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    CommandScheduler.getInstance().cancel(
                m_robotContainer.autoScoreFactory.addLimelightPose(1),
                m_robotContainer.autoScoreFactory.addLimelightPose(0));

    m_robotContainer.vision.setRejectAllUpdates(false);
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
