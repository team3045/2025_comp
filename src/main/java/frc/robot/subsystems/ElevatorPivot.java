// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.FloatLogEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.commons.GremlinUtil;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ElevatorPivotConstants.*;

import java.util.function.DoubleSupplier;

/* */
public class ElevatorPivot extends SubsystemBase {
  private TalonFX rightMotor = new TalonFX(rightMotorId,canbus);
  private TalonFX leftMotor = new TalonFX(leftMotorId, canbus);
  private TalonFX pivotMotor = new TalonFX(pivorMotorId, canbus);
  private CANcoder pivotCancoder = new CANcoder(pivotCancoderId, canbus);

  private double targetHeight;
  private double targetAngleDegrees;

  public Trigger atTargetHeight = new Trigger(() -> atTargetHeight());
  public Trigger atTargetAngle = new Trigger(() -> atTargetAngle());

  /** Creates a new Elevator. */
  public ElevatorPivot() {
    configDevices();

    if(Utils.isSimulation()){
      configSim();
    }

    targetHeight = getHeight(); //This should go after configDevices to make sure that the elevator is zeroed
    targetAngleDegrees = getPivotAngleDegrees();
  }

  /**
   * Configure the Can devices for this subsystem
   * Also Resets position to zero
   */
  public void configDevices(){
    rightMotor.getConfigurator().apply(elevatorMotorConfig.withMotorOutput(motorOutputConfigs.withInverted(rightInverted)));
    leftMotor.getConfigurator().apply(elevatorMotorConfig.withMotorOutput(motorOutputConfigs.withInverted(leftInverted)));
    pivotMotor.getConfigurator().apply(pivotMotorConfig);
    pivotCancoder.getConfigurator().apply(pivotCancoderConfig);


    //Clear Sticky faults
    rightMotor.clearStickyFaults();
    leftMotor.clearStickyFaults();
    pivotMotor.clearStickyFaults();

    //We assume elevator starts at lowest position
    rightMotor.setPosition(0);
    leftMotor.setPosition(0);

    BaseStatusSignal.setUpdateFrequencyForAll(200, 
      rightMotor.getPosition(),
      leftMotor.getPosition(),
      pivotMotor.getPosition(),
      pivotCancoder.getPosition());
  }

  /** 
   * Get the Height of the elevator from the rotations of the Right Motor Encoder.
   * Elevator should be zeroes so that zero rotations is minimum height. 
   * 
   * @return the height of the carriage of the elevator in meters
   */
  public double getHeight(){
    return rightMotor.getPosition().getValueAsDouble() * rotationToLengthRatio + minimumHeight;
  }

  /** Get the Angle of pivot in Rotations.
   * 0 is defined as when the pivot is horizontal to the ground facing forward.
   * Will return a positive number between 0 - 1 
   * @return the current angle of the pivot in Rotations as a positve number between 0 - 1
   */
  public double getPivotAngleRotations(){
    return pivotCancoder.getPosition().getValueAsDouble();
  }

  /** Get the Angle of pivot in Degrees.
   * 0 is defined as when the pivot is horizontal to the ground facing forward.
   * Will return a positive number between 0 - 360 
   * @return the current angle of the pivot in Rotations as a positve number between 0 - 360
   */
  public double getPivotAngleDegrees(){
    return Units.rotationsToDegrees(pivotCancoder.getPosition().getValueAsDouble());
  }

  /** Get the Angle of pivot in Radians.
   * 0 is defined as when the pivot is horizontal to the ground facing forward.
   * Will return a positive number between 0 - 2pi 
   * @return the current angle of the pivot in Rotations as a positve number between 0 - 2pi
   */
  public double getPivotAngleRadians(){
    return Units.rotationsToRadians(pivotCancoder.getPosition().getValueAsDouble());
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
   * Determine whether our current Angle is at the currently
   * set target angle. If it is within a constant threshold than it is considered
   * at the target angle
   * 
   * @return whether the current angle of the pivot is at the target
   */
  public boolean atTargetAngle(){
    return Math.abs(getPivotAngleDegrees() - targetAngleDegrees) < angleToleranceDegrees;
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
    this.targetHeight = GremlinUtil.clampWithLogs(maxHeight, minimumHeight, targetHeight);

    double targetRotations = convertHeightToRotations(targetHeight);

    MotionMagicVoltage request = new MotionMagicVoltage(targetRotations)
      .withEnableFOC(true).withSlot(0).withUpdateFreqHz(1000); //every  1 ms
    Follower followerRequest = new Follower(rightMotorId, rightInverted != leftInverted);

    rightMotor.setControl(request);
    leftMotor.setControl(followerRequest);
  }

  private void setAngleTargetDegrees(double targetAngleDegrees){
    this.targetAngleDegrees = targetAngleDegrees;

    double targetAngleRotations = Units.degreesToRotations(targetAngleDegrees);

    MotionMagicVoltage request = new MotionMagicVoltage(targetAngleRotations)
      .withEnableFOC(true).withSlot(0).withUpdateFreqHz(1000); //every 1 ms
    
    pivotMotor.setControl(request);
  }

  private void setHeightAndAngle(double heightMeters, double angleDegrees){
    setHeightTarget(heightMeters);
    setAngleTargetDegrees(angleDegrees);
  }

  /**
   * The public command we expose to direct the elevator to a height.
   * All subsystem actions should be controlled through commands not direct functions. 
   * 
   * @param desiredHeight the desired height in meters
   * @return a command directing this subsytem to go to desiredheight
   */
  public Command goToHeight(DoubleSupplier desiredHeight){
    return this.runOnce(() -> setHeightTarget(desiredHeight.getAsDouble())).until(atTargetHeight);
  }

  /**
   * The public command we expose to direct the pivot to an angle.
   * All subsystem actions should be controlled through commands not direct functions. 
   * 
   * @param desiredAngle the desired angle in degrees
   * @return a command directing this subsytem to go to desiredAngle
   */
  public Command goToAngleDegrees(DoubleSupplier desiredAngle){
    return this.runOnce(() -> setAngleTargetDegrees(desiredAngle.getAsDouble())).until(atTargetAngle);
  }

  /**
   * The public command we expose to direct the elevator to a position;
   * A Position includes both a height and a pivot Angle;
   * All subsystem actions should be controlled through commands not direct functions. 
   * 
   * @param desiredAngle the desired angle in degrees
   * @param desiredHeight the desired height in meters
   * @return a command directing this subsytem to go to the desired position
   */
  public Command goToPosition(DoubleSupplier desiredHeight, DoubleSupplier desiredAngle){
    return this.runOnce(() -> {
      setAngleTargetDegrees(desiredAngle.getAsDouble());
      setHeightTarget(desiredHeight.getAsDouble());
    }).until(atTargetAngle.and(atTargetHeight));
  }

  public Command increaseHeight(){
    return goToHeight(() -> getHeight() + 0.2);
  }

  public Command decreaseHeight(){
    return goToHeight(() -> getHeight() - 0.2);
  }

  public Command increaseAngle(){
    return goToAngleDegrees(() -> getPivotAngleDegrees() + 5);
  }

  public Command decreaseAngle(){
    return goToAngleDegrees(() -> getPivotAngleDegrees() - 5);
  }

  public Command increasePosition(){
    return goToPosition(() -> getHeight() + 0.2, () -> getPivotAngleDegrees() + 5);
  }

  public Command decreasePosition(){
    return goToPosition(() -> getHeight() - 0.2, () -> getPivotAngleDegrees() - 5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  } 

  /*SIMULATION*/
  private final ElevatorSim elevatorSim = new ElevatorSim(
    DCMotor.getKrakenX60Foc(2), 
    totalGearing,   
    carriageMass, 
    drumRadius, 
    minimumHeight, 
    maxHeight, 
    true, 
    minimumHeight);

  private final SingleJointedArmSim armSim = new SingleJointedArmSim(
    DCMotor.getKrakenX60(1), 
    pivotTotalGearing,
    pivotMOI, 
    pivotArmLength, 
    Units.degreesToRadians(minAngleDegrees), 
    Units.degreesToRadians(maxAngleDegrees), 
    true, 
    Units.degreesToRadians(minAngleDegrees));
  
  private TalonFXSimState rightMotorSim;
  private TalonFXSimState leftMotorSim;
  private TalonFXSimState pivotMotorSim;
  private CANcoderSimState pivotCancoderSim;

  //These dimensions are arbitrary but we'll standardize to meters
  private Mechanism2d elevatorMechanism = new Mechanism2d(canvasWidth, canvasHeight);
  private Mechanism2d pivotMechanism = new Mechanism2d(canvasHeight, canvasHeight);

  //(0,0) is bottom left
  private MechanismRoot2d stage1Left = elevatorMechanism.getRoot("1Left", 0.2, 0); 
  private MechanismRoot2d stage1Right = elevatorMechanism.getRoot("1Right", 1.8, 0); 
  private MechanismRoot2d stage2Left = elevatorMechanism.getRoot("2Left", 0.4, 0); 
  private MechanismRoot2d stage2Right = elevatorMechanism.getRoot("2Right", 1.6, 0); 
  private MechanismRoot2d stage3Left = elevatorMechanism.getRoot("3Left", 0.6, 0); 
  private MechanismRoot2d stage3Right = elevatorMechanism.getRoot("3Right", 1.4, 0); 
  private MechanismRoot2d stage4Left = elevatorMechanism.getRoot("4Left", 0.8, 0); 
  private MechanismRoot2d stage4Right = elevatorMechanism.getRoot("4Right", 1.2, 0); 

  @SuppressWarnings("unused")
  private MechanismLigament2d stage1LeftLigament = stage1Left.append(
    new MechanismLigament2d("Stage1Left", firstStageLength, 90));
  @SuppressWarnings("unused")
  private MechanismLigament2d stage1RightLigament = stage1Right.append(
    new MechanismLigament2d("Stage1Right", firstStageLength, 90));
  @SuppressWarnings("unused")
  private MechanismLigament2d stage2LeftLigament = stage2Left.append(
    new MechanismLigament2d("Stage2Left", secondStageLength, 90));
  @SuppressWarnings("unused")
  private MechanismLigament2d stage2RightLigament = stage2Right.append(
    new MechanismLigament2d("Stage2Right", secondStageLength, 90));
    @SuppressWarnings("unused")
  private MechanismLigament2d stage3LeftLigament = stage3Left.append(
    new MechanismLigament2d("Stage3Left", thirdStageLength, 90));
  @SuppressWarnings("unused")
  private MechanismLigament2d stage3RightLigament = stage3Right.append(
    new MechanismLigament2d("Stage3Right", thirdStageLength, 90));
  @SuppressWarnings("unused")
  private MechanismLigament2d stage4LeftLigament = stage4Left.append(
    new MechanismLigament2d("Stage4Left", firstStageLength, 90));
  @SuppressWarnings("unused")
  private MechanismLigament2d stage4RightLigament = stage4Right.append(
    new MechanismLigament2d("Stage4Right", firstStageLength, 90));

  private MechanismRoot2d pivotRoot = pivotMechanism.getRoot("pivotRoot", 2, 3);
  private MechanismLigament2d pivotLigament = pivotRoot.append(
    new MechanismLigament2d("pivotLigament", pivotArmLength, minAngleDegrees)
  );
  

  public void configSim(){
    rightMotorSim = rightMotor.getSimState();
    leftMotorSim = leftMotor.getSimState();
    pivotMotorSim = pivotMotor.getSimState();
    pivotCancoderSim = pivotCancoder.getSimState();

    rightMotorSim.Orientation = ChassisReference.CounterClockwise_Positive;
    leftMotorSim.Orientation = ChassisReference.CounterClockwise_Positive;
    pivotMotorSim.Orientation = ChassisReference.CounterClockwise_Positive;
    pivotCancoderSim.Orientation = ChassisReference.CounterClockwise_Positive;

    elevatorSim.setState(minimumHeight, 0);
    armSim.setState(Units.degreesToRadians(minAngleDegrees), 0);
  }

  @Override
  public void simulationPeriodic(){
    //Update sim states
    rightMotorSim = rightMotor.getSimState();
    leftMotorSim = leftMotor.getSimState();
    pivotMotorSim = pivotMotor.getSimState();
    pivotCancoderSim = pivotCancoder.getSimState();

    //update with latest simulated supply voltage
    rightMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    leftMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    pivotMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    pivotCancoderSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    //get output voltage for motors, we assume both output same amount 
    //since they are identical just two different sides
    Voltage motorOutputvoltage = rightMotorSim.getMotorVoltageMeasure();
    
    Voltage pivotMotorOutputVoltage = pivotMotorSim.getMotorVoltageMeasure();

    //Update physics sim, assuming 20 ms default loop time
    elevatorSim.setInputVoltage(motorOutputvoltage.in(Volts));
    elevatorSim.update(0.020); 
    armSim.setInputVoltage(pivotMotorOutputVoltage.in(Volts));
    armSim.update(0.020);

    //Apply the elevator sims calculations to the cancoder
    //Since Talonfx use cancoder as remote sensor this should also apply to the motors
    //If directly applying to motors note that motors require rotor position/velocity (before gear ratio), but
    //DCMotorSim returns mechanism position/velocity (after gear ratio)
    rightMotorSim.setRawRotorPosition(convertHeightToRotations(elevatorSim.getPositionMeters()) * totalGearing);
    leftMotorSim.setRawRotorPosition(convertHeightToRotations(elevatorSim.getPositionMeters()) * totalGearing);
    rightMotorSim.setRotorVelocity(convertVelocityToRotations(elevatorSim.getVelocityMetersPerSecond()) * totalGearing);
    leftMotorSim.setRawRotorPosition(convertVelocityToRotations(elevatorSim.getVelocityMetersPerSecond()) * totalGearing);
    
    pivotCancoderSim.setRawPosition(Units.radiansToRotations(armSim.getAngleRads() * pivotSensorToMechanismRatio));
    pivotCancoderSim.setVelocity(Units.radiansToRotations(armSim.getVelocityRadPerSec() * pivotSensorToMechanismRatio));
    pivotMotorSim.setRawRotorPosition(Units.radiansToRotations(armSim.getAngleRads() * pivotTotalGearing));
    pivotMotorSim.setRotorVelocity(Units.radiansToRotations(armSim.getVelocityRadPerSec() * pivotTotalGearing));

    updateMechanism2d();
  }


  /** 
   * Updates the mechanism2d which visualizes our mechanism in SmartDashboard, generally used for simulation
   */
  public void updateMechanism2d(){
    double currentHeight = getHeight();
    double currentAngle = getPivotAngleDegrees();

    //Logic to determine the height of each elevator length
    if(currentHeight >= maxHeight){
      stage4Left.setPosition(0.8, currentHeight - fourthStageLength);
      stage4Right.setPosition(1.2, currentHeight - fourthStageLength);
      stage3Left.setPosition(0.6, currentHeight - fourthStageLength - thirdStageLength);
      stage3Right.setPosition(1.4, currentHeight - fourthStageLength - thirdStageLength);
      stage2Left.setPosition(0.4, currentHeight - fourthStageLength - thirdStageLength - secondStageLength);
      stage2Right.setPosition(1.6, currentHeight - fourthStageLength - thirdStageLength - secondStageLength);
    } else if (currentHeight > firstStageLength + secondStageLength + thirdStageLength) {
      stage4Left.setPosition(0.8, currentHeight - fourthStageLength);
      stage4Right.setPosition(1.2, currentHeight - fourthStageLength);
      stage3Left.setPosition(0.6, currentHeight - fourthStageLength - thirdStageLength);
      stage3Right.setPosition(1.4, currentHeight - fourthStageLength - thirdStageLength);
      stage2Left.setPosition(0.4, currentHeight - fourthStageLength - thirdStageLength - secondStageLength);
      stage2Right.setPosition(1.6, currentHeight - fourthStageLength - thirdStageLength - secondStageLength);
    } else if (currentHeight > firstStageLength + secondStageLength ){
      stage4Left.setPosition(0.8, currentHeight - fourthStageLength);
      stage4Right.setPosition(1.2, currentHeight - fourthStageLength);
      stage3Left.setPosition(0.6, currentHeight - fourthStageLength - thirdStageLength);
      stage3Right.setPosition(1.4, currentHeight - fourthStageLength - thirdStageLength);
      stage2Left.setPosition(0.4, 0);
      stage2Right.setPosition(1.6, 0);
    } else {
      stage4Left.setPosition(0.8, currentHeight - fourthStageLength);
      stage4Right.setPosition(1.2, currentHeight - fourthStageLength);
      stage3Left.setPosition(0.6, 0);
      stage3Right.setPosition(1.4, 0);
      stage2Left.setPosition(0.4, 0);
      stage2Right.setPosition(1.6, 0);
    } 

    pivotLigament.setAngle(180 - currentAngle);

    SmartDashboard.putData("Pivot Mech2d", pivotMechanism);
    SmartDashboard.putData("Elevator Mech2d", elevatorMechanism);
    SmartDashboard.putNumber("Elevator Height", currentHeight);
    SmartDashboard.putNumber("Target Heght", targetHeight);
    SmartDashboard.putNumber("Arm Target", targetAngleDegrees);
    SmartDashboard.putNumber("Arm Angle", currentAngle);
  }
  
}
