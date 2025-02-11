package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commons.GremlinUtil;
import frc.robot.constants.ClawConstants;

import static frc.robot.constants.ClawConstants.*;

public class Claw extends SubsystemBase {
    private TalonFX clawMotor = new TalonFX(clawID,canbus); 
    private CANrange coralSensor = new CANrange(canRangeId, canbus);
    private TalonFX hopperMotor = new TalonFX(hopperId,canbus);
    private double targetSpeed;
  
    private static final FlywheelSim CLAW_MOTOR_SIM = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), flywheelMOI, gearing),
        DCMotor.getKrakenX60(1));
    
    public final Trigger hasCoral = new Trigger(() -> hasCoral());
    public final Trigger atSpeed = new Trigger(() -> atSpeed());
    
    public Claw(){
        configDevices();

        if(Utils.isSimulation()){
            configSim();
          }
    }

    public void configDevices(){
        clawMotor.getConfigurator().apply(motorConfig);
        coralSensor.getConfigurator().apply(canRangeConfig);
        hopperMotor.getConfigurator().apply(motorConfig.withMotorOutput(
            MotorOutputConfigs.withInverted(InvertedValue.Clockwise_Positive)));
    }

    public void setClawTargetSpeed(double speedRPS){
        targetSpeed = speedRPS;
        VelocityVoltage request = new VelocityVoltage(speedRPS)
            .withEnableFOC(true)
            .withSlot(0)
            .withUpdateFreqHz(1000);

        clawMotor.setControl(request);
    }

    public void setHopperTargetSpeed(double speedRPS){
        targetSpeed = speedRPS;
        VelocityVoltage request = new VelocityVoltage(speedRPS)
            .withEnableFOC(true)
            .withSlot(0)
            .withUpdateFreqHz(1000);

        hopperMotor.setControl(request);
    }


    public Command hopperIntake(){
        return this.runOnce(() -> setHopperTargetSpeed(intakeSpeed));
    }

    public Command clawIntake(){
        return this.runOnce(() -> setClawTargetSpeed(intakeSpeed));
    } 

    public Command fullIntake(){
        return this.runOnce(() -> {
            setHopperTargetSpeed(hopperSpeed);
            setClawTargetSpeed(intakeSpeed);
        });
    }

    public Command fullOutake(){
        return this.runOnce(() -> {
            setHopperTargetSpeed(outtakeSpeed);
            setClawTargetSpeed(outtakeSpeed);
        });
    }

    public Command fullHold(){
        return this.runOnce(() -> {
            setHopperTargetSpeed(holdSpeed);
            setClawTargetSpeed(holdSpeed);
        });
    }

    public Command clawOutake(){
        return this.runOnce(() -> setClawTargetSpeed(outtakeSpeed));
    } 

    public Command hold(){
        return this.runOnce(() -> setClawTargetSpeed(holdSpeed));
    }

    public Command slowIntake(){
        return this.runOnce(() -> setClawTargetSpeed(slowSpeed));
    }

    public Command slowBackup(){
        return this.runOnce(() -> setClawTargetSpeed(-slowSpeed));
    }

    public Command runAndHold() {
        return clawIntake().until(() -> {return !hasCoral();}).andThen(hold()).withTimeout(ClawConstants.timeoutSeconds);
    }

    public Command outputAndStop() {
        return clawOutake().until(() -> {return !hasCoral();}).andThen(Commands.waitSeconds(ClawConstants.outputWaitTime)).andThen(stop()).withTimeout(ClawConstants.timeoutSeconds);
    }

    public boolean hasCoral(){
        return coralSensor.getIsDetected(true).getValue();
    }

    public boolean atSpeed(){
        return GremlinUtil.withinTolerance(targetSpeed, clawMotor.getVelocity().getValueAsDouble(), speedTolerance);
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
    public void periodic(){
        SmartDashboard.putBoolean("Has Coral", hasCoral());
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
