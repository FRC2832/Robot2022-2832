package frc.robot.constants;

import edu.wpi.first.math.Pair;

import java.util.ArrayList;

public class ShooterConstants {
    public static final ArrayList<Pair<Double, Double>> LIDAR_DIST_TABLE = new ArrayList<>(13);
    private static final ArrayList<Pair<Double, Double>> VISION_DIST_TABLE = new ArrayList<>(15);
    private static final ArrayList<Pair<Double, Double>> DIST_RPM_TABLE = new ArrayList<>(15);
    private static final ArrayList<Pair<Double, Double>> DIST_HOOD_TABLE = new ArrayList<>(15);
    private static final double LIDAR_OFFSET = 0.0;

    public static void LoadConstants() {
        // table is input: pixel Y, output: in from target
        // data from Nighttime (3rd set) sheet in 2022 shooter vision table
        VISION_DIST_TABLE.add(new Pair<>(86.0, 60.0d));
        VISION_DIST_TABLE.add(new Pair<>(136.5, 72.0d));
        VISION_DIST_TABLE.add(new Pair<>(173.5, 84.0d));
        VISION_DIST_TABLE.add(new Pair<>(210.5, 96.0d));
        VISION_DIST_TABLE.add(new Pair<>(242.5, 108.0d));
        VISION_DIST_TABLE.add(new Pair<>(267.0, 120.0d));
        VISION_DIST_TABLE.add(new Pair<>(291.0, 132.0d));
        VISION_DIST_TABLE.add(new Pair<>(310.5, 144.0d));
        VISION_DIST_TABLE.add(new Pair<>(331.0, 156.0d));
        VISION_DIST_TABLE.add(new Pair<>(347.0, 168.0d));
        VISION_DIST_TABLE.add(new Pair<>(358.0, 180.0d));
        VISION_DIST_TABLE.add(new Pair<>(367.0, 192.0d));
        VISION_DIST_TABLE.add(new Pair<>(384.0, 204.0d));
        VISION_DIST_TABLE.add(new Pair<>(404.0, 216.0d));
        VISION_DIST_TABLE.add(new Pair<>(406.5, 228.0d));

        // table is input: distance in m, output: rpm
        // minimum shot distance 2.1-ish meters)
        LIDAR_DIST_TABLE.add(new Pair<>(0.97 + LIDAR_OFFSET, 2220.0)); // At hub, zero degrees on hood
        LIDAR_DIST_TABLE.add(new Pair<>(1.57 + LIDAR_OFFSET, 2220.0)); // 2 ft away, zero degrees on hood
        LIDAR_DIST_TABLE.add(new Pair<>(2.21 + LIDAR_OFFSET, 2470.0)); // 4 ft away, zero degrees on hood
        LIDAR_DIST_TABLE.add(new Pair<>(2.84 + LIDAR_OFFSET,
                                        2640.0)); // 6 ft away, zero degrees on hood (TODO: May need to verify hood
        // angle)
        LIDAR_DIST_TABLE.add(new Pair<>(3.44 + LIDAR_OFFSET, 2470.0)); // 8 ft away, 41.125 degrees on hood
        LIDAR_DIST_TABLE.add(new Pair<>(4.03 + LIDAR_OFFSET, 2600.0)); // 10 ft away, 51.375 degrees on hood
        LIDAR_DIST_TABLE.add(new Pair<>(4.59 + LIDAR_OFFSET, 2720.0)); // 12 ft away, 51.375 degrees on hood
        LIDAR_DIST_TABLE.add(new Pair<>(4.81 + LIDAR_OFFSET, 2840.0)); // 13 ft away, 51.375 degrees on hood
        LIDAR_DIST_TABLE.add(new Pair<>(5.15 + LIDAR_OFFSET, 2840.0)); // 14 ft away, 51.375 degrees on hood (for now)
        LIDAR_DIST_TABLE.add(new Pair<>(5.85 + LIDAR_OFFSET, 2840.0)); // 16 ft away, 51.375 degrees on hood (for now)
        LIDAR_DIST_TABLE.add(new Pair<>(6.43 + LIDAR_OFFSET, 2840.0)); // 18 ft away, 51.375 degrees on hood (for now)
        LIDAR_DIST_TABLE.add(new Pair<>(7.04 + LIDAR_OFFSET, 2840.0)); // 20 ft away, 51.375 degrees on hood (for now)
        LIDAR_DIST_TABLE.add(new Pair<>(7.52 + LIDAR_OFFSET, 2840.0)); // 21.5 ft away, 51.375 degrees on hood (for now)


        // table is input: distance in in, output: rpm
        // from auto shots sheet in 2022 shooter speed table
        DIST_RPM_TABLE.add(new Pair<>(60.0d, 2200.0d));
        DIST_RPM_TABLE.add(new Pair<>(72.0d, 2325.0d));
        DIST_RPM_TABLE.add(new Pair<>(84.0d, 2400.0d));
        DIST_RPM_TABLE.add(new Pair<>(96.0d, 2435.0d));
        DIST_RPM_TABLE.add(new Pair<>(108.0d, 2475.0d));
        DIST_RPM_TABLE.add(new Pair<>(120.0d, 2525.0d));
        DIST_RPM_TABLE.add(new Pair<>(132.0d, 2600.0d));
        DIST_RPM_TABLE.add(new Pair<>(144.0d, 2675.0d));
        DIST_RPM_TABLE.add(new Pair<>(156.0d, 2775.0d));
        DIST_RPM_TABLE.add(new Pair<>(168.0d, 2825.0d));
        DIST_RPM_TABLE.add(new Pair<>(180.0d, 2925.0d));
        DIST_RPM_TABLE.add(new Pair<>(192.0d, 3025.0d));
        DIST_RPM_TABLE.add(new Pair<>(204.0d, 3075.0d));
        DIST_RPM_TABLE.add(new Pair<>(216.0d, 3100.0d));
        DIST_RPM_TABLE.add(new Pair<>(228.0d, 3250.0d));

        // table is input: distance in in, output: angle in degrees
        // from auto shots sheet in 2022 shooter speed table
        // 88 degrees is when the hood is down and touching the limit switch
        // 36 degrees is when the hood is practically all the way up at knob 6
        // formula for measured angle conversion to angle in this table: 88 - measured
        // value + 20
        DIST_HOOD_TABLE.add(new Pair<>(60.0d, 31.0d)); // hood 2.5
        DIST_HOOD_TABLE.add(new Pair<>(72.0d, 31.0d)); // hood 2.5
        DIST_HOOD_TABLE.add(new Pair<>(84.0d, 33.0d)); // hood 2.75
        DIST_HOOD_TABLE.add(new Pair<>(96.0d, 38.0d)); // hood 3
        DIST_HOOD_TABLE.add(new Pair<>(108.0d, 42.0d)); // hood 3.75
        DIST_HOOD_TABLE.add(new Pair<>(120.0d, 50.0d)); // hood 4.25
        DIST_HOOD_TABLE.add(new Pair<>(132.0d, 51.0d)); // hood 4.5
        DIST_HOOD_TABLE.add(new Pair<>(144.0d, 52.5d)); // hood 4.5
        DIST_HOOD_TABLE.add(new Pair<>(156.0d, 53.0d)); // hood 4.5
        DIST_HOOD_TABLE.add(new Pair<>(168.0d, 53.0d)); // hood 4.5
        DIST_HOOD_TABLE.add(new Pair<>(180.0d, 56.0d)); // hood 5
        DIST_HOOD_TABLE.add(new Pair<>(192.0d, 57.0d)); // hood 5
        DIST_HOOD_TABLE.add(new Pair<>(204.0d, 57.0d)); // hood 5
        DIST_HOOD_TABLE.add(new Pair<>(216.0d, 63.0d)); // hood 5.5
        DIST_HOOD_TABLE.add(new Pair<>(228.0d, 63.0d)); // hood 5.5
    }

    public static ArrayList<Pair<Double, Double>> getVisionDistTable() {
        return VISION_DIST_TABLE;
    }

    public static ArrayList<Pair<Double, Double>> getDistRpmTable() {
        return DIST_RPM_TABLE;
    }

    public static ArrayList<Pair<Double, Double>> getDistHoodTable() {
        return DIST_HOOD_TABLE;
    }
}
