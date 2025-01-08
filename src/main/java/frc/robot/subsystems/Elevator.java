// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commons.GremlinUtil;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ElevatorConstants.*;

/* */
public class Elevator extends SubsystemBase {
  private TalonFX rightMotor = new TalonFX(rightMotorId,canbus);
  private TalonFX leftMotor = new TalonFX(leftMotorId, canbus);
  private CANcoder heightCancoder = new CANcoder(cancoderId, canbus);

  private double targetHeight = minimumHeight;

  public Trigger atTargetHeight = new Trigger(() -> atTargetHeight());

  /** Creates a new Elevator. */
  public Elevator() {
    configDevices();

    if(Utils.isSimulation()){
      configSim();
    }

    targetHeight = getHeight(); //This should go after configDevices to make sure that the elevator is zeroed
  }

  /**
   * Configure the Can devices for this subsystem
   * Also Resets position to zero
   */
  public void configDevices(){
    rightMotor.getConfigurator().apply(motorConfig.withMotorOutput(motorOutputConfigs.withInverted(rightInverted)));
    leftMotor.getConfigurator().apply(motorConfig.withMotorOutput(motorOutputConfigs.withInverted(leftInverted)));
    heightCancoder.getConfigurator().apply(cancoderConfig);

    //Clear Sticky faults
    rightMotor.clearStickyFaults();
    leftMotor.clearStickyFaults();
    heightCancoder.clearStickyFaults();

    //We assume elevator starts at lowest position
    rightMotor.setPosition(0);
    leftMotor.setPosition(0);
    heightCancoder.setPosition(0);
  }

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
   * Convert a velocity in meters per second to rotations per second
   * @param velocityMPS
   * @return the number of rotations per second that correspond to the passed in velocity
   */
  public double convertVelocityToRotations(double velocityMPS){
    return velocityMPS / rotationToLengthRatio;
  }

  /**
   * Set the height that the elevator should try to go to
   * This function sets the target height as well as actively
   * begins moving the elevator towards that height
   * 
   * @param targetHeight the desired height in meters
   */
  private void setHeightTarget(double targetHeight){
    System.out.println(targetHeight);
    this.targetHeight = GremlinUtil.clampWithLogs(maxHeight, minimumHeight, targetHeight);

    double targetRotations = convertHeightToRotations(targetHeight);

    MotionMagicVoltage request = new MotionMagicVoltage(targetRotations)
      .withEnableFOC(true).withSlot(0).withUpdateFreqHz(50); //every 2 ms

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

  public Command increaseHeight(){
    return goToHeight(getHeight() + 0.5);
  }

  public Command decreaseHeight(){
    return goToHeight(getHeight() - 0.1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  } 

  /*SIMULATION*/
  private final ElevatorSim elevatorSim = new ElevatorSim(
    DCMotor.getKrakenX60Foc(2), 
    rotorToSensorRatio,   
    carriageMass, 
    drumRadius, 
    minimumHeight, 
    maxHeight, 
    true, 
    minimumHeight);
  
  private TalonFXSimState rightMotorSim;
  private TalonFXSimState leftMotorSim;
  private CANcoderSimState heightCancoderSim;

  //These dimensions are arbitrary but we'll standardize to meters
  private Mechanism2d elevatorMechanism = new Mechanism2d(canvasWidth, canvasHeight);

  //(0,0) is bottom left
  // private MechanismRoot2d stage1Left = elevatorMechanism.getRoot("1Left", 0.2, 0); 
  // private MechanismRoot2d stage1Right = elevatorMechanism.getRoot("1Right", 1.8, 0); 
  // private MechanismRoot2d stage2Left = elevatorMechanism.getRoot("2Left", 0.4, 0); 
  // private MechanismRoot2d stage2Right = elevatorMechanism.getRoot("2Right", 1.6, 0); 
  // private MechanismRoot2d stage3Left = elevatorMechanism.getRoot("3Left", 0.6, 0); 
  // private MechanismRoot2d stage3Right = elevatorMechanism.getRoot("3Right", 1.4, 0); 
  // private MechanismRoot2d stage4Left = elevatorMechanism.getRoot("4Left", 0.8, 0); 
  // private MechanismRoot2d stage4Right = elevatorMechanism.getRoot("4Right", 1.2, 0); 

  // private MechanismLigament2d stage1LeftLigament = stage1Left.append(
  //   new MechanismLigament2d("Stage1Left", firstStageLength, 90));
  // private MechanismLigament2d stage1RightLigament = stage1Right.append(
  //   new MechanismLigament2d("Stage1Right", firstStageLength, 90));
  // private MechanismLigament2d stage2LeftLigament = stage2Left.append(
  //   new MechanismLigament2d("Stage2Left", secondStageLength, 90));
  // private MechanismLigament2d stage2RightLigament = stage2Right.append(
  //   new MechanismLigament2d("Stage2Right", secondStageLength, 90));
  // private MechanismLigament2d stage3LeftLigament = stage3Left.append(
  //   new MechanismLigament2d("Stage3Left", thirdStageLength, 90));
  // private MechanismLigament2d stage3RightLigament = stage3Right.append(
  //   new MechanismLigament2d("Stage3Right", thirdStageLength, 90));
  // private MechanismLigament2d stage4LeftLigament = stage4Left.append(
  //   new MechanismLigament2d("Stage4Left", firstStageLength, 90));
  // private MechanismLigament2d stage4RightLigament = stage4Right.append(
  //   new MechanismLigament2d("Stage4Right", firstStageLength, 90));

  private MechanismRoot2d testingRoot = elevatorMechanism.getRoot("root", 1, 0);
  private MechanismLigament2d ligament = testingRoot.append(new MechanismLigament2d(
    "ligament", minimumHeight, 90));
  

  public void configSim(){
    rightMotorSim = rightMotor.getSimState();
    leftMotorSim = leftMotor.getSimState();
    heightCancoderSim = heightCancoder.getSimState();

    rightMotorSim.Orientation = ChassisReference.CounterClockwise_Positive;
    heightCancoderSim.Orientation = ChassisReference.CounterClockwise_Positive;
    leftMotorSim.Orientation = ChassisReference.CounterClockwise_Positive;

    elevatorSim.setState(minimumHeight, 0);
  }

  @Override
  public void simulationPeriodic(){
    //Update sim states
    rightMotorSim = rightMotor.getSimState();
    leftMotorSim = leftMotor.getSimState();
    heightCancoderSim = heightCancoder.getSimState();

    //update with latest simulated supply voltage
    rightMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    leftMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    heightCancoderSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    //get output voltage for motors, we assume both output same amount 
    //since they are identical just two different sides
    Voltage motorOutputvoltage = rightMotorSim.getMotorVoltageMeasure();

    //Update physics sim, assuming 20 ms default loop time
    elevatorSim.setInputVoltage(motorOutputvoltage.in(Volts));
    elevatorSim.update(0.020); 

    //Apply the elevator sims calculations to the cancoder
    //Since Talonfx use cancoder as remote sensor this should also apply to the motors
    //If directly applying to motors note that motors require rotor position/velocity (before gear ratio), but
    //DCMotorSim returns mechanism position/velocity (after gear ratio)
    heightCancoderSim.setRawPosition(convertHeightToRotations(elevatorSim.getPositionMeters() * sensorToMechanismRatio));
    heightCancoderSim.setVelocity(convertVelocityToRotations(elevatorSim.getVelocityMetersPerSecond() * sensorToMechanismRatio));
    rightMotorSim.setRawRotorPosition(convertHeightToRotations(elevatorSim.getPositionMeters()) * rotorToSensorRatio);
    leftMotorSim.setRawRotorPosition(convertHeightToRotations(elevatorSim.getPositionMeters()) * rotorToSensorRatio);
    rightMotorSim.setRotorVelocity(convertVelocityToRotations(elevatorSim.getVelocityMetersPerSecond()) * rotorToSensorRatio);
    leftMotorSim.setRawRotorPosition(convertVelocityToRotations(elevatorSim.getVelocityMetersPerSecond()) * rotorToSensorRatio);

    // System.out.println("Voltage Output: " + motorOutputvoltage.in(Volts));
    // System.out.println("Elevator Height: " + elevatorSim.getPositionMeters());
    // System.out.println("Elevator Velocity: " + elevatorSim.getVelocityMetersPerSecond());
    System.out.println("Cancoder Applied Position: " + convertHeightToRotations(elevatorSim.getPositionMeters() * sensorToMechanismRatio));
    // System.out.println("Right Motor Target: " + rightMotor.getClosedLoopReference().getValueAsDouble());
    // System.out.println("Right Motor Position: " + rightMotor.getRotorPosition().getValueAsDouble());
    System.out.println("CanCoder Position: " + heightCancoder.getPosition().getValueAsDouble());
    // System.out.println("Right Motor Error: " + rightMotor.getClosedLoopError().getValueAsDouble());

    updateMechanism2d();
  }


  /** 
   * Updates the mechanism2d which visualizes our mechanism in SmartDashboard, generally used for simulation
   */
  public void updateMechanism2d(){
    double currentHeight = getHeight();

    // if(currentHeight >= maxHeight){
    //   System.out.println("First if statement");
    //   stage4Left.setPosition(0.8, currentHeight - fourthStageLength);
    //   stage4Right.setPosition(1.2, currentHeight - fourthStageLength);
    //   stage3Left.setPosition(0.6, currentHeight - fourthStageLength - thirdStageLength);
    //   stage3Right.setPosition(1.4, currentHeight - fourthStageLength - thirdStageLength);
    //   stage2Left.setPosition(0.4, currentHeight - fourthStageLength - thirdStageLength - secondStageLength);
    //   stage2Right.setPosition(1.6, currentHeight - fourthStageLength - thirdStageLength - secondStageLength);
    // } else if (currentHeight > firstStageLength + secondStageLength + thirdStageLength) {
    //   System.out.println("Second if statement");
    //   stage4Left.setPosition(0.8, currentHeight);
    //   stage4Right.setPosition(1.2, currentHeight);
    //   stage3Left.setPosition(0.6, currentHeight - fourthStageLength);
    //   stage3Right.setPosition(1.4, currentHeight - fourthStageLength);
    //   stage2Left.setPosition(0.4, currentHeight - fourthStageLength - thirdStageLength);
    //   stage2Right.setPosition(1.6, currentHeight - fourthStageLength - thirdStageLength);
    // } else if (currentHeight > firstStageLength + secondStageLength ){
    //   System.out.println("third if statement");
    //   stage4Left.setPosition(0.8, currentHeight);
    //   stage4Right.setPosition(1.2, currentHeight);
    //   stage3Left.setPosition(0.6, currentHeight - fourthStageLength);
    //   stage3Right.setPosition(1.4, currentHeight - fourthStageLength);
    //   stage2Left.setPosition(0.4, 0);
    //   stage2Right.setPosition(1.6, 0);
    // } else {
    //   System.out.println("4th if statement");
    //   stage4Left.setPosition(0.8, currentHeight - fourthStageLength);
    //   stage4Right.setPosition(1.2, currentHeight - fourthStageLength);
    //   stage3Left.setPosition(0.6, 0);
    //   stage3Right.setPosition(1.4, 0);
    //   stage2Left.setPosition(0.4, 0);
    //   stage2Right.setPosition(1.6, 0);
    // } 

    ligament.setLength(currentHeight);

    SmartDashboard.putData("Elevator Mech2d", elevatorMechanism);
    SmartDashboard.putNumber("Elevator Height", currentHeight);
    SmartDashboard.putNumber("Target Heght", targetHeight);
  }
  
}
