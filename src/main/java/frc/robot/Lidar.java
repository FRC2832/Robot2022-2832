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
    private final Queue<Double> cachedDistances = new PriorityBlockingQueue<>(3);
    private static double distanceToTarget;
    private double runningAvg;

    public Lidar() {
        NetworkTable table = netTableInstance.getTable("lidar");
        distanceEntry = table.getEntry("distance");
    }


    @Override
    public void periodic() {
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
        return distanceToTarget > 0.0 && distanceToTarget < 16.4592; // TODO: May need to change these values.
    }

}
