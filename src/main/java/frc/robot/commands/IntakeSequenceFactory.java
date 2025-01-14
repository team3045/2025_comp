package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorPivot;
import frc.robot.constants.IntakeSequenceConstants.*;

public class IntakeSequenceFactory {
    private CommandSwerveDrivetrain drivetrain; 
    private ElevatorPivot elevatorPivot;
    private Claw claw; 


    public IntakeSequenceFactory(CommandSwerveDrivetrain drivetrain, ElevatorPivot elevatorPivot, Claw claw){
        this.drivetrain = drivetrain;
        this.elevatorPivot = elevatorPivot;
        this.claw = claw; 
    }

   

    // public Command getPathFindCommand(){
    //      return drivetrain.pathFindToPose(0,0);
    // }

   

    // public Command setElevatorHeight(){
    //      return elevatorPivot.goToPosition()
    //  }



}
