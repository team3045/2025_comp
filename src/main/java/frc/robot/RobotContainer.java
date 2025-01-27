// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoScoreFactory;
import frc.robot.commands.DriveWheelRadiusCharacterization;
import frc.robot.commands.IntakeSequenceFactory;
import frc.robot.commons.GremlinPS4Controller;
import frc.robot.commons.GremlinUtil;
import frc.robot.constants.DriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorPivot;
import frc.robot.vision.apriltag.GremlinApriltagVision;
import frc.robot.vision.apriltag.VisionConstants;

import static frc.robot.constants.DriveConstants.MaxSpeed;
import static frc.robot.constants.DriveConstants.MaxAngularRate;;


public class RobotContainer {
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final GremlinPS4Controller joystick = new GremlinPS4Controller(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final GremlinApriltagVision vision = new GremlinApriltagVision(VisionConstants.cameras,
        () -> drivetrain.getState().Pose, 
        VisionConstants.limelight,
        (drivetrain::addVisionMeasurements));
    public final ElevatorPivot elevatorPivot = new ElevatorPivot();
    public final Claw claw = new Claw();

    /*Auto Score Stuff */
    public final AutoScoreFactory autoScoreFactory = new AutoScoreFactory(drivetrain, elevatorPivot, claw);

    /* intake sequence */
    public final IntakeSequenceFactory intakeSequenceFactory = new IntakeSequenceFactory(drivetrain, elevatorPivot, claw);


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
                DriveConstants.drive.withVelocityX(GremlinUtil.squareDriverInput(joystick.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(GremlinUtil.squareDriverInput(joystick.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(GremlinUtil.squareDriverInput(-joystick.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
//TODO: uncomment
        // joystick.triangle().whileTrue(drivetrain.applyRequest(() -> DriveConstants.brake));
        // joystick.circle().whileTrue(drivetrain.applyRequest(() ->
        //     DriveConstants.point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

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

        // joystick.cross().whileTrue(
        //     intakeSequenceFactory.getPathFindCommand()
        //     .andThen(elevatorPivot.goToIntakeReady()).onlyIf(() -> intakeSequenceFactory.isNearSubstation()).andThen( //NEEDS TO BE .andThen (if not, the pathfinding command is not run)
        //         Commands.waitSeconds(1))
        //     .andThen(intakeSequenceFactory.moveElevatorAndIntake())); //TODO: cancel / end behavior;

        joystick.circle().onTrue(elevatorPivot.goToIntakeReady());
        joystick.triangle().onTrue(intakeSequenceFactory.moveElevatorAndIntake());

        joystick.cross().whileTrue(new DriveWheelRadiusCharacterization(drivetrain, DriveWheelRadiusCharacterization.Direction.COUNTER_CLOCKWISE));

        
        // joystick.square().whileTrue(
        //     autoScoreFactory.getPathFindCommand()
        //     // .andThen(autoScoreFactory.getPrecisePidCommand())
        //     .alongWith(autoScoreFactory.setElevatorHeight())
        //     .andThen(elevatorPivot.goToPosition(() -> elevatorPivot.getHeight() - 0.1, () -> elevatorPivot.getPivotAngleDegrees())
        //     .andThen(claw.clawOutake())));

        joystick.square().whileTrue(
            autoScoreFactory.pathFindWithApriltagFeeback(VisionConstants.limelight[0])
            .alongWith(autoScoreFactory.setElevatorHeight())
            .andThen(elevatorPivot.goToPosition(() -> elevatorPivot.getHeight() - 0.15, () -> elevatorPivot.getPivotAngleDegrees())
            .andThen(Commands.waitSeconds(0.3).andThen(claw.clawOutake().andThen(Commands.waitSeconds(0.2)))
            .andThen(drivetrain.driveBack().andThen(elevatorPivot.stowArm().alongWith(claw.stop()))))));

                
        // joystick.circle().whileTrue(elevatorPivot.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // joystick.cross().whileTrue(elevatorPivot.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // joystick.square().whileTrue(elevatorPivot.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // joystick.triangle().whileTrue(elevatorPivot.sysIdDynamic(SysIdRoutine.Direction.kReverse));


        //joystick.share().onTrue(elevatorPivot.goToIntakeReady());
        joystick.L1().onTrue(autoScoreFactory.setElevatorHeight());
        joystick.R1().onTrue(elevatorPivot.stowArm());
        joystick.R2().onTrue(claw.stop());
        // joystick.square().onTrue(elevatorPivot.zeroHeight());
        //joystick.L2().whileTrue(elevatorPivot.decreaseAngle().repeatedly());
        // joystick.R2().whileTrue(elevatorPivot.increaseAngle().repeatedly());

        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return AutoBuilder.buildAuto("Curvy Square");
    }
}
