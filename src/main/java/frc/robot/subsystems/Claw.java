package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.ClawConstants.*;

public class Claw extends SubsystemBase {

    private TalonFX clawMotor = new TalonFX(clawID,canbus); 
    // add beam break when caden decides 
    private double targetSpeed;
  
    private static final FlywheelSim CLAW_MOTOR_SIM = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), flywheelMOI, gearing),
        DCMotor.getKrakenX60(1));
    
    
    public Claw(){
        configDevices();

        if(Utils.isSimulation()){
            configSim();
          }
    }

    public void configDevices(){
        clawMotor.getConfigurator().apply(motorConfig);
    }

    public void setTargetSpeed(double speedRPS){
        targetSpeed = speedRPS;
        VelocityVoltage request = new VelocityVoltage(speedRPS)
            .withEnableFOC(true)
            .withSlot(0)
            .withUpdateFreqHz(1000);

        clawMotor.setControl(request);
        System.out.println(clawMotor.getClosedLoopReference().getValueAsDouble());
        System.out.println(speedRPS);
    }

    public Command clawIntake(){
        return this.runOnce(() -> setTargetSpeed(intakeSpeed));
    } 

    public Command clawOutake(){
        return this.runOnce(() -> setTargetSpeed(outtakeSpeed));
    } 

    public Command hold(){
        return this.runOnce(() -> setTargetSpeed(holdSpeed));
    }

    public void stopRunnable(){
        clawMotor.stopMotor();
    }

    public Command stop(){
        return this.runOnce(()-> {
            clawMotor.stopMotor();
            targetSpeed = 0;
        });
    }

    private TalonFXSimState clawMotorSim;

    public void configSim(){
        clawMotorSim = clawMotor.getSimState();
    }


    @Override
    public void simulationPeriodic() {
        clawMotorSim = clawMotor.getSimState();
        clawMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        CLAW_MOTOR_SIM.setInputVoltage(clawMotorSim.getMotorVoltage());
        CLAW_MOTOR_SIM.update(0.02);

        double motorVelocity = Units.radiansToRotations(CLAW_MOTOR_SIM.getAngularVelocityRadPerSec() * gearing);
        clawMotorSim.setRotorVelocity(motorVelocity);

        SmartDashboard.putNumber("Claw Motor Sim Velocity", motorVelocity);
        SmartDashboard.putNumber("Target Velocity", targetSpeed);
    }
}
