package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commons.GremlinPS4Controller;
import frc.robot.constants.IntakeSequenceConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorPivot;

public class IntakeSequenceFactory {
    private CommandSwerveDrivetrain drivetrain; 
    private ElevatorPivot elevatorPivot;
    private Claw claw; 

    public IntakeSequenceFactory(CommandSwerveDrivetrain drivetrain, ElevatorPivot elevatorPivot, Claw claw){
        this.drivetrain = drivetrain;  
        this.elevatorPivot = elevatorPivot;
        this.claw = claw; 
    }
    
    public Command getPathFindCommand(){
    
            return new ConditionalCommand(
                drivetrain.pathFindToPose(
                        ()->  IntakeSequenceConstants.leftSubstationPose, 
                        ()-> IntakeSequenceConstants.desiredEndVelocity),
                new ConditionalCommand(
                    drivetrain.pathFindToPose(
                                ()-> IntakeSequenceConstants.rightSubstationPose,
                                ()-> IntakeSequenceConstants.desiredEndVelocity), 
                    Commands.none(), 
                    () -> isWithinRightRange(drivetrain.getState().Pose)), 
                ()-> isWithinLeftRange(drivetrain.getState().Pose));
    } 

    private boolean isWithinLeftRange(Pose2d pose){
        return IntakeSequenceConstants.leftSubstation.contains(pose);
    }

    private boolean isWithinRightRange(Pose2d pose){
        return IntakeSequenceConstants.rightSubstation.contains(pose);
    }


/* Sets elevator & arm ready to intake  */
    public Command setElevatorPivotPosition(){
        return  //TODO: use and then sugaring, also "goTo" Methods already wait until its at the height before ending
            elevatorPivot.goToHeight(()-> IntakeSequenceConstants.intakeReadyHeight) //TODO: Use GoToPosition rather than goToHeight or GoToAngle, 
            .andThen(Commands.waitUntil(elevatorPivot.atTargetHeight)),
            elevatorPivot.goToAngleDegrees(()-> IntakeSequenceConstants.intakeReadyAngle)) //TODO: Use GoToPosition rather than goToHeight or GoToAngle, 
            .andThen(Commands.waitUntil(elevatorPivot.atTargetAngle)
        );
    }

/* moves elevator & arm down & intakes coral */
    public Command moveElevatorAndIntake(){
        return 
            elevatorPivot.goToPosition(
                ()-> IntakeSequenceConstants.intakingHeight, 
                ()-> elevatorPivot.getPivotAngleDegrees())
                .alongWith(claw.clawIntake())
                .andThen(Commands.waitUntil(claw.hasObject))
                .andThen(goToStow());
    } 

    //should probably go in elevatorpivot 
    public Command goToStow(){
        return elevatorPivot.goToPosition(
            ()-> IntakeSequenceConstants.stowHeight, 
            ()-> IntakeSequenceConstants.stowAngle);
    }

}

