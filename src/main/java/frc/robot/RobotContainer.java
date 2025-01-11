// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.constants.DriveConstants.deadband;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.commands.PathPlannerAuto;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commons.GremlinPS4Controller;
import frc.robot.commons.GremlinUtil;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorPivot;
import frc.robot.vision.apriltag.GremlinApriltagVision;
import frc.robot.vision.apriltag.VisionConstants;

import static frc.robot.constants.DriveConstants.MaxSpeed;
import static frc.robot.constants.DriveConstants.MaxAngularRate;;


public class RobotContainer {
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * deadband).withRotationalDeadband(MaxAngularRate * deadband) // Add a 5% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use close-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final GremlinPS4Controller joystick = new GremlinPS4Controller(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final GremlinApriltagVision vision = new GremlinApriltagVision(VisionConstants.cameras,
        () -> drivetrain.getState().Pose, 
        (drivetrain::addVisionMeasurements));
    public final ElevatorPivot elevatorPivot = new ElevatorPivot();

    public RobotContainer() {
        DogLog.setOptions(new DogLogOptions());
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(GremlinUtil.squareDriverInput(joystick.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(GremlinUtil.squareDriverInput(joystick.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(GremlinUtil.squareDriverInput(-joystick.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.triangle().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.circle().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.options().and(joystick.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.options().and(joystick.cross()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.share().and(joystick.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.share().and(joystick.cross()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // joystick.cross().whileTrue(drivetrain.applyRequest(() -> 
        //     drive.withVelocityX(MaxSpeed)
        //     .withVelocityY(0)
        //     .withRotationalRate(0))
        // );

        joystick.R1().whileTrue(elevatorPivot.increasePosition().repeatedly());
        joystick.L1().whileTrue(elevatorPivot.decreasePosition().repeatedly());
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Curvy Square");
    }
}
