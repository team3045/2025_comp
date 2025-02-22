// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.ClimberConstants.*;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private static TalonFX motor = new TalonFX(climberID, canbus);
  private static TalonFXConfigurator configurator = motor.getConfigurator();
  /** Creates a new Climber. */
  public Climber() {
    CurrentLimitsConfigs configs = new CurrentLimitsConfigs();
    configs.SupplyCurrentLimit = maxAmps;
    configs.SupplyCurrentLowerLimit = minAmps;
    configurator.apply(configs);
  }

  private void setControl(float voltage) {
    VoltageOut config = new VoltageOut(Math.abs(voltage));
    MotorOutputConfigs motorConfig = new MotorOutputConfigs();
    if (voltage < 0) {
      motorConfig.Inverted = InvertedValue.Clockwise_Positive;
    } else {
      motorConfig.Inverted = InvertedValue.CounterClockwise_Positive;
    }
    configurator.apply(motorConfig);
    motor.setControl(config);
  }

  public Command climb() {
    return this.runOnce(() -> setControl(volts));
  }

  public Command stop() {
    return this.runOnce(() -> setControl(0));
  }

  public Command lower() {
    return this.runOnce(() -> setControl(-volts));
  }
}
