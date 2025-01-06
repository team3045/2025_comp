// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.oakD;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.vision.oakD.ObjectDetConstants.*;


public class GremlinObjectDetection extends SubsystemBase {
  private GremlinOakD[] cameras;

  private TimeInterpolatableBuffer<Pose2d> robotPoseBuffer = TimeInterpolatableBuffer.createBuffer(timeThreshold);

  public ArrayList<TimestampedObjectPose> currentObjects;

  private StructArrayPublisher<Pose2d> objectPositionPublishe = NetworkTableInstance.getDefault()
    .getStructArrayTopic("Object Poses", Pose2d.struct).publish();

  /** Creates a new GremlinObjectDetection. */
  public GremlinObjectDetection(GremlinOakD... cameras) {
    this.cameras = cameras;
    currentObjects = new ArrayList<>();
  }

  public void updateWithValidEstimates(){
        ArrayList<TimestampedObjectPose> uncheckedUpdates = getRecentUpdates();
        
        for(TimestampedObjectPose possibleObject : uncheckedUpdates){
            //Check to make sure are inside the field
            Translation2d translation3d = possibleObject.pose().getTranslation();

            // if (translation3d.getX() <= distanceTreshold
            //     || translation3d.getX() >= fieldSize.getX() - distanceTreshold
            //     || translation3d.getY() <= distanceTreshold
            //     || translation3d.getY() >= fieldSize.getY() - distanceTreshold) 
            // {continue;}

            //Check to make sure they arent too close to any old notes
            //If they're close enough we're going to assume they are the same note and just update the position of the old one
            boolean isNearOldNote = false;
            for(int i = 0; i <currentObjects.size(); i++){
                if(isNearPose(possibleObject.pose(), currentObjects.get(i).pose(), distanceTreshold)){
                    currentObjects.set(i, possibleObject);
                    isNearOldNote = true;
                    break;
                }
            }

            if(!isNearOldNote) currentObjects.add(possibleObject);
        }
  }

  public ArrayList<TimestampedObjectPose> getRecentUpdates(){
    ArrayList<TimestampedObjectPose> recentUpdates = new ArrayList<>();

    for(GremlinOakD cam : cameras){
      recentUpdates.addAll(cam.getCurrentEstimations());
    }

    return recentUpdates;
  }

  public boolean isNearPose(Pose2d firstPose, Pose2d secondPose, double threshold){
        return firstPose.getTranslation().getDistance(secondPose.getTranslation()) < threshold;
    }

  @Override
  public void periodic() {
    //Clear out old object positions
    for(int i = 0; i < currentObjects.size(); i++){
      if(Math.abs(Timer.getFPGATimestamp() - currentObjects.get(i).timestamp()) > timeThreshold)
        currentObjects.remove(i);
    }

    updateWithValidEstimates();

    List<Pose2d> poses = currentObjects.stream().map(timestamp -> timestamp.pose())
      .toList();

    objectPositionPublishe.set(poses.toArray(Pose2d[]::new));
  }

  
}
