package frc.robot.commons;

import edu.wpi.first.math.geometry.Pose2d;

public class BoundingBox {
    private final double minX, maxX, minY, maxY;

    public BoundingBox(double minX, double maxX, double minY, double maxY) {
        this.minX = minX;
        this.maxX = maxX;
        this.minY = minY;
        this.maxY = maxY;
    }

    public boolean contains(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        return x > minX && x < maxX && y > minY && y < maxY;
    }
}
