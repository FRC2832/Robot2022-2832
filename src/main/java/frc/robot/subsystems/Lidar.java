package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Queue;
import java.util.concurrent.PriorityBlockingQueue;

public class Lidar extends SubsystemBase {
    private static boolean isConnected;
    private static double distanceToTarget;
    private static double runningAvg;
    private final NetworkTableInstance netTableInstance = NetworkTableInstance.getDefault();
    private final NetworkTableEntry distanceEntry;
    private final NetworkTableEntry connectivityEntry;
    private final Queue<Double> cachedDistances = new PriorityBlockingQueue<>(3);

    public Lidar() {
        super();
        NetworkTable table = netTableInstance.getTable("lidar");
        distanceEntry = table.getEntry("distance");
        connectivityEntry = table.getEntry("lidarConnected");
    }

    public static double getDistanceToTarget() {
        return distanceToTarget;
    }

    public static double getRunningAvg() {
        return runningAvg;
    }

    public static boolean getIsConnected() {
        return isConnected;
    }

    public static double calculateTargetAngle(double distance) {
        // distance is in meters.
        double angle;
        double lidarOffset = 0.0;
        //double targetY = Pi.getTargetCenterY(); 2.81 m
        if (distance >= 3.9 + lidarOffset) { // && targetY >= 286.0) {
            angle = 51.375;
        } else if (distance >= 3.0 + lidarOffset) { // && targetY >= 220) {
            angle = 41.125;
        } else {
            angle = 20.0;
        }
        return angle;
    }

    @Override
    public void periodic() {
        isConnected = connectivityEntry.getBoolean(true); // TODO: Change this to false when we add it to Python code.
        Number distanceNum = distanceEntry.getNumber(-1.0);
        distanceToTarget = distanceNum.intValue() / 100.0;
        if (cachedDistances.size() >= 3) {
            cachedDistances.remove();
        }
        cachedDistances.add(distanceToTarget);
        double sum = 0.0;
        for (double d : cachedDistances) {
            sum += d;
        }
        runningAvg = sum / cachedDistances.size();
    }

    public boolean hasTarget() {
        return distanceToTarget > 0.0 && distanceToTarget <= 9.0; // TODO: May need to change these values.
    }

}
