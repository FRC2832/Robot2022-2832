package frc.robot;

import java.util.Queue;
import java.util.concurrent.PriorityBlockingQueue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lidar extends SubsystemBase {
    private final NetworkTableInstance netTableInstance = NetworkTableInstance.getDefault();
    private final NetworkTableEntry distanceEntry;
    private final NetworkTableEntry connectivityEntry;
    private final Queue<Double> cachedDistances = new PriorityBlockingQueue<>(3);
    private static boolean isConnected;
    private static double distanceToTarget;
    private static double runningAvg;

    public Lidar() {
        NetworkTable table = netTableInstance.getTable("lidar");
        distanceEntry = table.getEntry("distance");
        connectivityEntry = table.getEntry("lidarConnected");
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

    public static double getDistanceToTarget() {
        return distanceToTarget;
    }

    public boolean hasTarget() {
        return distanceToTarget > 0.0 && distanceToTarget <= 9.0; // TODO: May need to change these values.
    }

    public static double getRunningAvg() {
        return runningAvg;
    }
    public static boolean getIsConnected() {
        return isConnected;
    }
    
    public static double calculateTargetAngle(double distance) {
        // distance is in meters.
        double angle = 20.0;
        double targetY = Pi.getTargetCenterY();
        if (distance >= 4.25 && targetY >= 286.0) {
            angle = 51.375;
        } else if (distance >= 3.5 && targetY >= 220) {
            angle = 41.125;
        } else if (distance >= 3.086 && targetY >= 158.5) {
            angle = 25.0;
        }
        return angle;
    }

}
