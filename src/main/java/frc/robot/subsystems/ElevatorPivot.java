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

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.FloatLogEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
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

import java.util.TooManyListenersException;
import java.util.function.DoubleSupplier;

/* */
public class ElevatorPivot extends SubsystemBase {
  private TalonFX rightMotor = new TalonFX(rightMotorId,canbus);
  private TalonFX leftMotor = new TalonFX(leftMotorId, canbus);
  private TalonFX pivotMotor = new TalonFX(pivorMotorId, canbus);
  private CANcoder pivotCancoder = new CANcoder(pivotCancoderId, canbus);

  private double targetHeight;
  private double targetAngleDegrees;

  private boolean travellingUpward;

  public Trigger atTargetHeight = new Trigger(() -> atTargetHeight());
  public Trigger atTargetAngle = new Trigger(() -> atTargetAngle());

  private final Timer upwardsTimer = new Timer();
  private final Timer downwardsTimer = new Timer();

  /** Creates a new Elevator. */
  public ElevatorPivot() {
    configDevices();

    if(Utils.isSimulation()){
      configSim();
    }

    targetHeight = getHeight(); //This should go after configDevices to make sure that the elevator is zeroed
    targetAngleDegrees = getPivotAngleDegrees();
    travellingUpward = true;
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
   * Get the Height of the top of the elevator carriage from the rotations of the Right Motor Encoder.
   * Elevator should be zeroes so that zero rotations is minimum height. 
   * 
   * @return the height of the carriage of the elevator in meters
   */
  public double getHeight(){
    return rightMotor.getPosition().getValueAsDouble() * rotationToLengthRatio + minimumHeight;
  }

  /**
   * Get the velocity of the elevator from the rotations of the Right Motor Encoder.
   * Units are meters per second, upwards is positive, downwards is negative. 
   * 
   * @return the velocity of the elevator in meters per second
   */
  public double getVerticalVelocity(){
    return rightMotor.getVelocity().getValueAsDouble() * rotationToLengthRatio;
  }

  /**Returns whether or not the elevator is traveling upwards,
   * defined as whether its velocity is positive or negative. 
   * Positive velocity is upwards.
   * 
   * @return whether or not the elevator is travelling upwards.
   */
  public boolean travelingUpwards(){
    return travellingUpward;
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

    //TODO: add check if travelling downwards then make sure the pivot wont hit the next stage bar

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
      setHeightAndAngle(desiredHeight.getAsDouble(), desiredAngle.getAsDouble());
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
    //if zero retain last value basically
    // if(getVerticalVelocity() > 0) {
    //   upwardsTimer.start();
    //   downwardsTimer.stop();
    //   downwardsTimer.reset();
    //   if (upwardsTimer.hasElapsed(verticalTimerThreshold)) {
    //     travellingUpward = true;
    //   }
    // }
    // else if (getVerticalVelocity() < 0) {
    //   upwardsTimer.stop();
    //   upwardsTimer.reset();
    //   downwardsTimer.start();
    //   if(downwardsTimer.hasElapsed(verticalTimerThreshold)){
    //     travellingUpward = false;
    //   }
    // } 

    if(getVerticalVelocity() > 0.2) travellingUpward = true;
    else if(getVerticalVelocity() < -0.2) travellingUpward = false;
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

  private double stage3Height = carriageToGround;
  private double stage2Height = carriageToGround;
  private double carriageHeight = carriageToGround;

  private Mechanism2d pivotMechanism = new Mechanism2d(canvasWidth, canvasHeight);

  private StructPublisher<Pose3d> stage2Publisher = NetworkTableInstance.getDefault().getTable(elevatorTable)
    .getStructTopic("stage2Pose3d", Pose3d.struct).publish();
  private StructPublisher<Pose3d> stage3Publisher = NetworkTableInstance.getDefault().getTable(elevatorTable)
    .getStructTopic("stage3Pose3d", Pose3d.struct).publish();
  private StructPublisher<Pose3d> carriagePublisher = NetworkTableInstance.getDefault().getTable(elevatorTable)
    .getStructTopic("carriagePose3d", Pose3d.struct).publish();
  private StructPublisher<Pose3d> pivotPublisher = NetworkTableInstance.getDefault().getTable(elevatorTable)
    .getStructTopic("pivotPose3d", Pose3d.struct).publish();

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

    stage3Height = carriageToGround;
    stage2Height = carriageToGround;
    carriageHeight = carriageToGround;
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
    carriageHeight = getHeight();
    double currentAngle = getPivotAngleDegrees();

    boolean travellingUpwards = travelingUpwards();

    //Update the max Heights
    double stage3Top = stage3Height + stage3StageLength;
    double stage2Top = stage2Height + stage2StageLength - stage3StageLength; 

    //Logic to handle the position of the elevator stages
    if(travellingUpwards){
      if(carriageHeight < stage3Top){
          //Do Nothing, carriageHeight is just carriage Height
      } else if (carriageHeight >= stage3Top && stage3Height < stage2Top) {
          stage3Height = carriageHeight - stage3StageLength;
          //Stage2Height remains the same
      } else if (carriageHeight >= stage3Top && stage3Height >= stage2Top){
          stage3Height = carriageHeight - stage3StageLength;
          stage2Height = carriageHeight - stage2StageLength;
      } else {
        try {
          throw new Exception("Something weird happened with the elevator sim heights");
        } catch (Exception e) {
          e.printStackTrace();
        }
      }
    } else {
      if(carriageHeight > stage3Height){
          //Do nothing, hasnt hit the bottom yet
      } else if (carriageHeight <= stage3Height && stage3Height > stage2Height) {
          stage3Height = carriageHeight;
      } else if (carriageHeight <= stage3Height && stage3Height <= stage2Height){
          stage3Height = carriageHeight;
          stage2Height = carriageHeight;
      }
    }

    double carriageZ = carriageHeight - carriageToGround; //Same no matter what
    double stage3Z = stage3Height - carriageToGround; //The heights of our stages are not the same as their z positions
    double stage2Z = stage2Height - carriageToGround;

    pivotLigament.setAngle(180 - currentAngle);

    stage2Publisher.set(new Pose3d(0,0,stage2Z, new Rotation3d(0, 0, 0)));
    stage3Publisher.set(new Pose3d(0,0, stage3Z, new Rotation3d(0, 0, 0)));
    carriagePublisher.set(new Pose3d(0,0, carriageZ, new Rotation3d()));
    pivotPublisher.set(new Pose3d(pivotOffsetX,pivotOffsetY,carriageZ + pivotOffsetZ, new Rotation3d(0,-getPivotAngleRadians(),0)));


    SmartDashboard.putData("Pivot Mech2d", pivotMechanism);
    SmartDashboard.putNumber("Elevator Height", carriageHeight);
    SmartDashboard.putNumber("Target Heght", targetHeight);
    SmartDashboard.putNumber("Arm Target", targetAngleDegrees);
    SmartDashboard.putNumber("Arm Angle", currentAngle);
  }
  
}
