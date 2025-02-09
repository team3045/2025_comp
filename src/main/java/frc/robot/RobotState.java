// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class RobotState {

    private DriveState driveState;
    private static RobotState mRobotState;

    public enum DriveState{
        TELEOP,
        AUTOSCORE,
        INTAKE,
        ALGEA
    }

    private RobotState(){
        this.driveState = DriveState.TELEOP;
    }

    public static RobotState getRobotState(){
        if(mRobotState == null){
            mRobotState = new RobotState();
        }

        return mRobotState;
    }

    public DriveState getDriveState(){
        return driveState;
    }

    public void setDriveState(DriveState newDriveState){
        this.driveState = newDriveState;
    }
}
