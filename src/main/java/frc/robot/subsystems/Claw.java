package frc.robot.subsystems;

import static frc.robot.constants.ClawConstants.clawID;
import static frc.robot.constants.ClawConstants.clawSpeed;
import static frc.robot.constants.ElevatorPivotConstants.canbus;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClawConstants.*;

public class Claw extends SubsystemBase {

    private TalonFX clawMotor = new TalonFX(clawID,canbus); 
// add beam break when caden decides 

    private double gearing = 1; 
  
    private static final FlywheelSim CLAW_MOTOR_SIM = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 1, 1),
        DCMotor.getKrakenX60(1),
        0.1);
    
    public Claw(){
        configDevices();

        if(Utils.isSimulation()){
            configSim();
          }
    }

    public void configDevices(){
        clawMotor.getConfigurator().apply(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive));
    }

    public Command clawIntake(){
        return this.run(()-> clawMotor.set(clawSpeed));
    } 

    public Command clawOutake(){
        return this.run(()-> clawMotor.set(-clawSpeed));
    } 

    public void stopRunnable(){
        clawMotor.stopMotor();
    }

    public Command stop(){
        return this.runOnce(()-> clawMotor.stopMotor());
    }

    private TalonFXSimState clawMotorSim;

    public void configSim(){
        clawMotorSim = clawMotor.getSimState();

        clawMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());


    }


    @Override
    public void simulationPeriodic() {
        clawMotorSim = clawMotor.getSimState();

        CLAW_MOTOR_SIM.setInputVoltage(clawMotorSim.getMotorVoltage());
        CLAW_MOTOR_SIM.update(0.02);

        double motorVelocity = CLAW_MOTOR_SIM.getAngularVelocityRadPerSec() * (60/(2*Math.PI) * gearing);
        clawMotorSim.setRotorVelocity(motorVelocity);

        SmartDashboard.putNumber("Claw Motor Sim Velocity", motorVelocity);
    }



}
