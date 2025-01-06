// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.oakD;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.commons.GeomUtil;

import static frc.robot.vision.oakD.ObjectDetConstants.*;

/** Add your docs here. */
public class GremlinOakD {
    private DoubleArraySubscriber xDistanceSubscriber;
    private DoubleArraySubscriber yDistanceSubscriber;
    private DoubleArraySubscriber zDistanceSubscriber;
    private DoubleSubscriber latencySubscriber;

    private Pose3d camPose;
    private String name;
    private String path;

    private Function<Double, Optional<Pose2d>> robotPoseSupplier;

    public GremlinOakD(String name, Pose3d camPose, Function<Double, Optional<Pose2d>> poseSupplier){
        this.name = name;
        this.camPose = camPose;
        this.path = "/" + name;
        this.robotPoseSupplier = poseSupplier;

        this.xDistanceSubscriber = NetworkTableInstance.getDefault().getDoubleArrayTopic(path  + "/x")
            .subscribe(new double[0]);
        this.yDistanceSubscriber = NetworkTableInstance.getDefault().getDoubleArrayTopic(path + "/y")
            .subscribe(new double[0]);
        this.zDistanceSubscriber = NetworkTableInstance.getDefault().getDoubleArrayTopic(path + "/z")
            .subscribe(new double[0]);  
        this.latencySubscriber = NetworkTableInstance.getDefault().getDoubleTopic(path + "/latency")
            .subscribe(-1.0);
    }

    public ArrayList<TimestampedObjectPose> getCurrentEstimations(){
        ArrayList<TimestampedObjectPose> currentNotePoses = new ArrayList<>();
        double[] xDistances = xDistanceSubscriber.get();
        double[] yDistances = yDistanceSubscriber.get();
        double[] zDistances = zDistanceSubscriber.get();
        double latency = latencySubscriber.get();
        double timestamp = Timer.getFPGATimestamp() - latency;

        if(xDistances.length != zDistances.length || xDistances.length != yDistances.length) return currentNotePoses;

        for(int i =0; i < xDistances.length; i++){
            Transform3d camToNote = new Transform3d(
                xDistances[i], 
                yDistances[i], 
                zDistances[i], 
                new Rotation3d());
            
            Optional<Pose2d> robotPose = robotPoseSupplier.apply(timestamp);

            if(robotPose.isPresent()) {
                Pose3d camPoseField = new Pose3d(robotPose.get()).transformBy(GeomUtil.pose3dToTransform3d(camPose).inverse());
                Pose3d notePoseField = camPoseField.transformBy(camToNote);
                currentNotePoses.add(new TimestampedObjectPose(notePoseField.toPose2d(), timestamp)); //Consider checking to make sure its near the ground
            }
        }

        return currentNotePoses;
    }
    

}
