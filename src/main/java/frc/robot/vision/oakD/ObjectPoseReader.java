// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.oakD;

import java.util.ArrayList;
import java.util.function.Function;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

/** @deprecated this is just hear for reference right now and in case we want to thread this later*/
public class ObjectPoseReader implements Runnable{
    private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private static NetworkTable oakTable = inst.getTable("OAK");
    private static DoubleArraySubscriber xDistanceSubscriber;
    private static DoubleArraySubscriber yDistanceSubscriber;
    private static DoubleArraySubscriber zDistanceSubscriber;
    private static DoubleArraySubscriber latencySubscriber;

    private static ArrayList<TimestampedObjectPose> currentObjects = new ArrayList<>();

    private static final Transform3d robotToCam = new Transform3d();
    private static final double timeThreshold = 3; //Three seconds
    private static final double distanceTreshold = 0.5; //half meter
    public static final Translation2d fieldSize = new Translation2d(16.54, 8.21);

    private Function<Double, Pose2d> poseSupplier;

    public ObjectPoseReader(Function<Double, Pose2d> poseSupplier){
        xDistanceSubscriber = oakTable.getDoubleArrayTopic("XDistance").subscribe(null);
        yDistanceSubscriber = oakTable.getDoubleArrayTopic("YDistance").subscribe(null);
        zDistanceSubscriber = oakTable.getDoubleArrayTopic("ZDistance").subscribe(null);
        latencySubscriber = oakTable.getDoubleArrayTopic("latency").subscribe(null);

        this.poseSupplier = poseSupplier;
    }

    @Override
    public void run() {
        throw new UnsupportedOperationException("Unimplemented method 'run'");
    }


    public void updateCurrentObjects(){

        //Clear out old object positions
        for(int i = 0; i < currentObjects.size(); i++){
            if(Timer.getFPGATimestamp() - currentObjects.get(i).timestamp() > timeThreshold)
                currentObjects.remove(i);
        }

        updateWithValidEstimates();
        
    }

    public void updateWithValidEstimates(){
        TimestampedObjectPose[] uncheckedUpdates = getRecentUpdates();
        
        for(TimestampedObjectPose possibleObject : uncheckedUpdates){
            //Check to make sure are inside the field
            Translation2d translation3d = possibleObject.pose().getTranslation();

            if (translation3d.getX() <= distanceTreshold
                || translation3d.getX() >= fieldSize.getX() - distanceTreshold
                || translation3d.getY() <= distanceTreshold
                || translation3d.getY() >= fieldSize.getY() - distanceTreshold) 
            {continue;}

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

    public TimestampedObjectPose[] getRecentUpdates(){
        double NTtimestamp = xDistanceSubscriber.getLastChange();
        Transform3d[] transforms = getRecentTransforms();
        TimestampedObjectPose[] updates = new TimestampedObjectPose[transforms.length];

        //Get the Pose at the time each note was captured, add the robot cam transform and then the distance to the note
        for(int i = 0; i < transforms.length; i++){
            Pose3d pose = new Pose3d(poseSupplier.apply(NTtimestamp)).transformBy(robotToCam).transformBy(transforms[i]);
            updates[i] = new TimestampedObjectPose(pose.toPose2d(), NTtimestamp);
        }

        return updates;   
    }

    public Transform3d[] getRecentTransforms(){
        double[] recentXDistances = xDistanceSubscriber.get();
        double[] recentYDistances = yDistanceSubscriber.get();
        double[] recentZDistances = zDistanceSubscriber.get();
        double[] recentLatencyValues = latencySubscriber.get();

        if(recentXDistances == null || recentYDistances == null || recentZDistances == null || recentLatencyValues == null
            || recentXDistances.length != recentYDistances.length || recentXDistances.length != recentZDistances.length || 
            recentXDistances.length != recentLatencyValues.length
        ){
            return new Transform3d[0];
        }

        Transform3d[] transforms = new Transform3d[recentXDistances.length];

        for(int i = 0; i < recentXDistances.length; i++){
            Translation3d translation3d = new Translation3d(recentXDistances[i], recentYDistances[i], recentZDistances[i]);
            transforms[i] = new Transform3d(translation3d, new Rotation3d());
        }

        return transforms;
    }
    
    public boolean isNearPose(Pose3d firstPose, Pose3d secondPose, double threshold){
        return firstPose.getTranslation().getDistance(secondPose.getTranslation()) > threshold;
    }
    public boolean isNearPose(Pose2d firstPose, Pose2d secondPose, double threshold){
        return firstPose.getTranslation().getDistance(secondPose.getTranslation()) > threshold;
    }
}
