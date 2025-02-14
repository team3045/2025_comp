package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.constants.ClimberConstants.*;

public class Climber extends SubsystemBase {
    private TalonFX climberMotor = new TalonFX(climberID, canbus); 
    private double targetSpeed;

    public Climber(){
        configDevices();
    }

    private void configDevices() {
        climberMotor.getConfigurator().apply(motorConfig);
    }

    public void setTargetSpeed(double _targetSpeed){
        targetSpeed = _targetSpeed;
        VelocityVoltage request = new VelocityVoltage(_targetSpeed)
            .withEnableFOC(true)
            .withSlot(0)
            .withUpdateFreqHz(1000);

        climberMotor.setControl(request);
    }

    public void startMotorClimb() {
        setTargetSpeed(climberClimbVelocity);
    }

    public void startMotorLower() {
        setTargetSpeed(-climberClimbVelocity);
    }

    public void stopMotor() {
        setTargetSpeed(0);
    }
}
