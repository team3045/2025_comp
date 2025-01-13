// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;

import static edu.wpi.first.units.Units.*;

/** Add your docs here. */
public class DriveConstants {
    public static final String DRIVE_LOG_PATH = "DriveState/";

    public static double MaxSpeed = 3; //TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);//RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public static final double MAX_VELOCITY = 4; //Meters per Second
    public static final double MAX_VELOCITY_AUTO = MAX_VELOCITY*0.75; 
    public static final double MAX_ACCEL = 3; //Meters per Second Squared
    public static final double MAX_ACCEL_AUTO = MAX_ACCEL * 0.75;
    public static final double MAX_ANGULAR_VELOCITY = 3*Math.PI / 2; //Radians per Second
    public static final double MAX_ANGULAR_VELOCITY_AUTO = MAX_ANGULAR_VELOCITY*0.75; 
    public static final double MAX_ANGULAR_ACCEL = Math.PI; //Radians per Second Squared
    public static final double MAX_ANGULAR_ACCEL_AUTO = MAX_ANGULAR_ACCEL * 0.75;

    public static final double MAX_STEER_VELOCITY = 10; //radians per second

    public static final double deadband = 0.05; //5% deadband

    public static final PPHolonomicDriveController pathFollowingController = new PPHolonomicDriveController(
        // PID constants for translation
        new PIDConstants(1, 0, 0),
        // PID constants for rotation
        new PIDConstants(9, 0, 0)
    );

    public static final double preciseTranslationkP = 8;
    public static final double preciseTranslationkI = 0;
    public static final double preciseTranslationkD = 0;
    public static final double preciseRotationkP = 8;
    public static final double preciseRotationkI = 0;
    public static final double preciseRotationkD = 0;

    public static final double preciseTranslationTolerance = 0.1;
    public static final double kMaxPathFindTranslationError = 0.2;

    public static final PIDController preciseTranslationController = new PIDController(preciseTranslationkP, preciseTranslationkI, preciseTranslationkD);
    public static final PIDController preciseRotationController = new PIDController(preciseRotationkP, preciseRotationkI, preciseRotationkD);

    public static final PathConstraints pathFollowingConstraints = new PathConstraints(
        MAX_VELOCITY_AUTO, 
        MAX_ACCEL_AUTO, 
        MAX_ANGULAR_VELOCITY_AUTO, 
        MAX_ANGULAR_ACCEL_AUTO);
}
