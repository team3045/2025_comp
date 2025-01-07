// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.constants.ElevatorConstants.*;


/* */
public class Elevator extends SubsystemBase {
  private TalonFX rightMotor = new TalonFX(rightMotorId,canbus);
  private TalonFX leftMotor = new TalonFX(leftMotorId, canbus);
  private CANcoder heightCancoder = new CANcoder(cancoderId, canbus);

  /** Creates a new Elevator. */
  public Elevator() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
