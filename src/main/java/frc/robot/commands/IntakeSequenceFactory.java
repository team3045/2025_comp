package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.IntakeSequenceConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorPivot;

public class IntakeSequenceFactory {
    private CommandSwerveDrivetrain drivetrain; 
    private ElevatorPivot elevatorPivot;
    private Claw claw; 

    private Boolean pastMidPoint = drivetrain.getState().Pose.getX() < 5;


    public IntakeSequenceFactory(CommandSwerveDrivetrain drivetrain, ElevatorPivot elevatorPivot, Claw claw){
        this.drivetrain = drivetrain;
        this.elevatorPivot = elevatorPivot;
        this.claw = claw; 
    }

    public Command getPathFindCommand(){
        return new ConditionalCommand(
            drivetrain.pathFindToPose(
               IntakeSequenceConstants.leftSubstationPose, 
                IntakeSequenceConstants.desiredEndVelocity), 
            drivetrain.pathFindToPose(
                IntakeSequenceConstants.rightSubstationPose,
                IntakeSequenceConstants.desiredEndVelocity),
            () -> pastMidPoint);
    } 

    public Command setElevatorPivotPosition(){
        return new SequentialCommandGroup(
            elevatorPivot.goToHeight(IntakeSequenceConstants.intakeReadyHeight)
            .andThen(Commands.waitUntil(elevatorPivot.atTargetHeight)),
            
            elevatorPivot.goToAngleDegrees(IntakeSequenceConstants.intakeReadyAngle))
            .andThen(Commands.waitUntil(elevatorPivot.atTargetAngle)
        );
    }

    public Command moveElevatorAndIntake(){
        return 
            elevatorPivot.goToHeight(IntakeSequenceConstants.intakingHeight)
            .alongWith(claw.clawIntake())
            .andThen(Commands.waitUntil(claw.hasObject))
                .withTimeout(IntakeSequenceConstants.timeOutTime)
            .andThen(elevatorPivot.goToPosition
                (IntakeSequenceConstants.stowHeight,
                IntakeSequenceConstants.stowAngle));
    }

}
