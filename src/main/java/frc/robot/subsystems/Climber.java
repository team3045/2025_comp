// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.ClimberConstants.*;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private static TalonFX motor = new TalonFX(climberID, canbus);
  /** Creates a new Climber. */
  public Climber() {
    configDevices();
    // CurrentLimitsConfigs configs = new CurrentLimitsConfigs();
    // configs.SupplyCurrentLimit = maxAmps;
    // configs.SupplyCurrentLowerLimit = minAmps;
    // configurator.apply(configs);
  }

  public void configDevices() {
    motor.getConfigurator().apply(motorConfig);
  }

  @SuppressWarnings("unused")
  private double targetSpeed;

public void spin(double numRotations) {
        PositionVoltage request = new PositionVoltage(motor.getPosition().getValueAsDouble() - numRotations)
                .withSlot(1)
                .withUpdateFreqHz(1000);

        motor.setControl(request);
    }

    
  public void setClimbTargetSpeed(double speedRPS) {
      targetSpeed = speedRPS;
      VelocityVoltage request = new VelocityVoltage(speedRPS)
              .withEnableFOC(true)
              .withSlot(0)
              .withUpdateFreqHz(1000);

      motor.setControl(request);
  }


 public Command spinClimberIn() {
  return this.runOnce(() -> spin(-ClimberConstants.spinAmount));
 }

 public Command spinClimberOut() {
  return this.runOnce(() -> spin(ClimberConstants.spinAmount));
 }


  public Command runForward(){
    return this.runOnce(()-> setClimbTargetSpeed(climberSpeed));
  }

  public Command stop(){
    return this.runOnce(()-> setClimbTargetSpeed(0));
  }

  public Command runBackword(){
    return this.runOnce(()-> setClimbTargetSpeed(-climberSpeed));
  }

  public Command climberOut(){
    return this.runOnce(() -> motor.setVoltage(10));
  }

  public Command climberIn(){
    return this.runOnce(() -> motor.setVoltage(-10));
  }

  public Command zeroClimber(){
    return this.runOnce(() -> motor.setVoltage(0));
  }

  // private void setControl(float voltage) {
  //   VoltageOut config = new VoltageOut(Math.abs(voltage));
  //   MotorOutputConfigs motorConfig = new MotorOutputConfigs();
  //   if (voltage < 0) {
  //     motorConfig.Inverted = InvertedValue.Clockwise_Positive;
  //   } else {
  //     motorConfig.Inverted = InvertedValue.CounterClockwise_Positive;
  //   }
  //   configurator.apply(motorConfig);
  //   motor.setControl(config);
  // }

  // public Command climb() {
  //   return this.runOnce(() -> setControl(volts));
  // }

  // public Command stop() {
  //   return this.runOnce(() -> setControl(0));
  // }

  // public Command lower() {
  //   return this.runOnce(() -> setControl(-volts));
  // }
}
