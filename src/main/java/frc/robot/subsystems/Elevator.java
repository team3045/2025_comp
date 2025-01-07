// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.constants.ElevatorConstants.*;


/* */
public class Elevator extends SubsystemBase {
  private TalonFX rightMotor = new TalonFX(rightMotorId,canbus);
  private TalonFX leftMotor = new TalonFX(leftMotorId, canbus);
  private CANcoder heightCancoder = new CANcoder(cancoderId, canbus);

  private double targetHeight;

  public Trigger atTargetHeight = new Trigger(() -> atTargetHeight());

  /** Creates a new Elevator. */
  public Elevator() {
    targetHeight = getHeight();
  }

  /**
   * Configure the Can devices for this subsystem
   */
  public void configDevices(){}

  /** 
   * Get the Height of the elevator from the rotations of the Encoder.
   * Elevator should be zeroes so that zero rotations is minimum height. 
   * 
   * @return the height of the carriage of the elevator in meters
   */
  public double getHeight(){
    return heightCancoder.getPosition().getValueAsDouble() * rotationToLengthRatio + minimumHeight;
  }

  /**
   * Determine whether our current height is at the currently
   * set target height. If it is within a constant threshold than it is considered
   * at the target height
   * 
   * @return whether the current height of the elevator is at the target
   */
  public boolean atTargetHeight(){
    return Math.abs(getHeight() - targetHeight) < heightTolerance;
  }

  /** 
   * Convert a height in meters to rotations for the encoder 
   * that will position the elevator at that height
   * 
   * @param height the height in meters you want to turn into rotations
   * @return the number of rotations of the encoder that correspond to the passed in height
   */
  public double convertHeightToRotations(double height){
    return (height - minimumHeight) / rotationToLengthRatio;
  }

  /**
   * Set the height that the elevator should try to go to
   * This function sets the target height as well as actively
   * begins moving the elevator towards that height
   * 
   * @param targetHeight the desired height in meters
   */
  private void setHeightTarget(double targetHeight){
    this.targetHeight = targetHeight;

    if(targetHeight > maxHeight){
      targetHeight = maxHeight;
      System.out.println("Requested Height above Max"); //TODO: Consider replacing with WPILIB Alert
    } else if (targetHeight < minimumHeight) {
      targetHeight= minimumHeight;
      System.out.println("Requested Height below Min"); //TODO: Consider Replacing with WPILIB Alert
    }

    double targetRotations = convertHeightToRotations(targetHeight);

    MotionMagicVoltage request = new MotionMagicVoltage(targetRotations)
      .withEnableFOC(true).withSlot(0).withUpdateFreqHz(500); //every 2 ms

    rightMotor.setControl(request);
    leftMotor.setControl(request);
  }

  /**
   * The public command we expose to direct the elevator to a height.
   * All subsystem actions should be controlled through commands not direct functions. 
   * 
   * @param desiredHeight the desired height in meters
   * @return a command directing this subsytem to go to desiredheight
   */
  public Command goToHeight(double desiredHeight){
    return this.run(() -> setHeightTarget(desiredHeight));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  } 
}
