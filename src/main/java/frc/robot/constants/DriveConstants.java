// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.*;

/** Add your docs here. */
public class DriveConstants {
    public static final String DRIVE_LOG_PATH = "DriveState/";

    public static final double drivebaseRadius = Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY);

    public static double MaxSpeed = 3; //TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);//RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public static final double deadband = 0.025; //2.5% deadband

    public static final double MAX_VELOCITY = 2; //Meters per Second
    public static final double MAX_VELOCITY_AUTO = MAX_VELOCITY*0.75; 
    public static final double MAX_ACCEL = 2; //Meters per Second Squared
    public static final double MAX_ACCEL_AUTO = MAX_ACCEL * 0.75;
    public static final double MAX_ANGULAR_VELOCITY = 3*Math.PI / 2; //Radians per Second
    public static final double MAX_ANGULAR_VELOCITY_AUTO = MAX_ANGULAR_VELOCITY*0.75; 
    public static final double MAX_ANGULAR_ACCEL = Math.PI; //Radians per Second Squared
    public static final double MAX_ANGULAR_ACCEL_AUTO = MAX_ANGULAR_ACCEL * 0.75;
    public static final double MAX_VELO_AUTOSCORE = 2;
    public static final double MAX_ACCEL_AUTOSCORE = 1;
    public static final double MAX_ANGULAR_VELOCITY_AUTOSCORE = Math.PI;
    public static final double MAX_ANGULAR_ACCEL_AUTOSCORE = Math.PI / 2;


    /* Setting up bindings for necessary control of the swerve drive platform */
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * deadband).withRotationalDeadband(MaxAngularRate * deadband) // Add a 5% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use close-loop control for drive motors
    public static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    public static final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public static final SwerveRequest.RobotCentric driveBack = new SwerveRequest.RobotCentric()
        .withVelocityX(MaxSpeed*-0.2);

    public static final SwerveRequest.ApplyFieldSpeeds APPLY_FIELD_SPEEDS = new ApplyFieldSpeeds()
        .withDesaturateWheelSpeeds(true)
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        .withDriveRequestType(DriveRequestType.Velocity);

    public static final double MAX_STEER_VELOCITY = 10; //radians per second

    public static final PPHolonomicDriveController pathFollowingController = new PPHolonomicDriveController(
        // PID constants for translation
        new PIDConstants(3, 0, 0),
        // PID constants for rotation
        new PIDConstants(6, 0, 0)
    );

    public static final double preciseTranslationkP = 8;
    public static final double preciseTranslationkI = 0;
    public static final double preciseTranslationkD = 0;
    public static final double preciseRotationkP = 10;
    public static final double preciseRotationkI = 0;
    public static final double preciseRotationkD = 0;

    public static final double preciseTranslationTolerance = 0.02;
    public static final double preciseRotationTolerance = 0.8;
    public static final double kMaxPathFindTranslationError = 0.2;

    public static final PIDController preciseTranslationController = new PIDController(preciseTranslationkP, preciseTranslationkI, preciseTranslationkD);
    public static final PIDController preciseRotationController = new PIDController(preciseRotationkP, preciseRotationkI, preciseRotationkD);

    public static final PathConstraints pathFollowingConstraints = new PathConstraints(
        MAX_VELOCITY_AUTO, 
        MAX_ACCEL_AUTO, 
        MAX_ANGULAR_VELOCITY_AUTO, 
        MAX_ANGULAR_ACCEL_AUTO);
    
    public static final PathConstraints autoScoreConstraints = new PathConstraints(
        MAX_VELO_AUTOSCORE, 
        MAX_ACCEL_AUTOSCORE, 
        MAX_ANGULAR_VELOCITY_AUTOSCORE, 
        MAX_ANGULAR_ACCEL_AUTOSCORE);
}
