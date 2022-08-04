package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.Pair;

public class ShooterConstants {
    public static final ArrayList<Pair<Double, Double>> VISION_DIST_TABLE = new ArrayList<>(15);
    public static final ArrayList<Pair<Double, Double>> LIDAR_DIST_TABLE = new ArrayList<>(8);
    public static final ArrayList<Pair<Double, Double>> DIST_RPM_TABLE = new ArrayList<>(15);
    public static final ArrayList<Pair<Double, Double>> DIST_HOOD_TABLE = new ArrayList<>(15);
    private static final double LIDAR_OFFSET = 0.0;

    public static void LoadConstants() {
        // table is input: pixel Y, output: in from target
        // data from Nighttime (3rd set) sheet in 2022 shooter vision table
        VISION_DIST_TABLE.add(new Pair<>(86.0, 60d));
        VISION_DIST_TABLE.add(new Pair<>(136.5, 72d));
        VISION_DIST_TABLE.add(new Pair<>(173.5, 84d));
        VISION_DIST_TABLE.add(new Pair<>(210.5, 96d));
        VISION_DIST_TABLE.add(new Pair<>(242.5, 108d));
        VISION_DIST_TABLE.add(new Pair<>(267.0, 120d));
        VISION_DIST_TABLE.add(new Pair<>(291.0, 132d));
        VISION_DIST_TABLE.add(new Pair<>(310.5, 144d));
        VISION_DIST_TABLE.add(new Pair<>(331.0, 156d));
        VISION_DIST_TABLE.add(new Pair<>(347.0, 168d));
        VISION_DIST_TABLE.add(new Pair<>(358.0, 180d));
        VISION_DIST_TABLE.add(new Pair<>(367.0, 192d));
        VISION_DIST_TABLE.add(new Pair<>(384.0, 204d));
        VISION_DIST_TABLE.add(new Pair<>(404.0, 216d));
        VISION_DIST_TABLE.add(new Pair<>(406.5, 228d));

        // table is input: distance in m, output: rpm
        // minimum shot distance 2.1-ish meters)
        LIDAR_DIST_TABLE.add(new Pair<>(0.97 + LIDAR_OFFSET, 2220.0)); // At hub, zero degrees on hood
        LIDAR_DIST_TABLE.add(new Pair<>(1.57 + LIDAR_OFFSET, 2220.0)); // 2 ft away, zero degrees on hood
        LIDAR_DIST_TABLE.add(new Pair<>(2.21 + LIDAR_OFFSET, 2470.0)); // 4 ft away, zero degrees on hood
        LIDAR_DIST_TABLE.add(new Pair<>(2.84 + LIDAR_OFFSET, 2640.0)); // 6 ft away, zero degrees on hood (TODO: May need to verify hood angle)
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
        DIST_RPM_TABLE.add(new Pair<>(60d, 2200d));
        DIST_RPM_TABLE.add(new Pair<>(72d, 2325d));
        DIST_RPM_TABLE.add(new Pair<>(84d, 2400d));
        DIST_RPM_TABLE.add(new Pair<>(96d, 2435d));
        DIST_RPM_TABLE.add(new Pair<>(108d, 2475d));
        DIST_RPM_TABLE.add(new Pair<>(120d, 2525d));
        DIST_RPM_TABLE.add(new Pair<>(132d, 2600d));
        DIST_RPM_TABLE.add(new Pair<>(144d, 2675d));
        DIST_RPM_TABLE.add(new Pair<>(156d, 2775d));
        DIST_RPM_TABLE.add(new Pair<>(168d, 2825d));
        DIST_RPM_TABLE.add(new Pair<>(180d, 2925d));
        DIST_RPM_TABLE.add(new Pair<>(192d, 3025d));
        DIST_RPM_TABLE.add(new Pair<>(204d, 3075d));
        DIST_RPM_TABLE.add(new Pair<>(216d, 3100d));
        DIST_RPM_TABLE.add(new Pair<>(228d, 3250d));

        // table is input: distance in in, output: angle in degrees
        // from auto shots sheet in 2022 shooter speed table
        // 88 degrees is when the hood is down and touching the limit switch
        // 36 degrees is when the hood is practically all the way up at knob 6
        // formula for measured angle conversion to angle in this table: 88 - measured
        // value + 20
        DIST_HOOD_TABLE.add(new Pair<>(60d, 27d)); // hood 2.5
        DIST_HOOD_TABLE.add(new Pair<>(72d, 29d)); // hood 2.5
        DIST_HOOD_TABLE.add(new Pair<>(84d, 33d)); // hood 2.75
        DIST_HOOD_TABLE.add(new Pair<>(96d, 38d)); // hood 3
        DIST_HOOD_TABLE.add(new Pair<>(108d, 42d)); // hood 3.75
        DIST_HOOD_TABLE.add(new Pair<>(120d, 50d)); // hood 4.25
        DIST_HOOD_TABLE.add(new Pair<>(132d, 51d)); // hood 4.5
        DIST_HOOD_TABLE.add(new Pair<>(144d, 52.5d)); // hood 4.5
        DIST_HOOD_TABLE.add(new Pair<>(156d, 53d)); // hood 4.5
        DIST_HOOD_TABLE.add(new Pair<>(168d, 53d)); // hood 4.5
        DIST_HOOD_TABLE.add(new Pair<>(180d, 56d)); // hood 5
        DIST_HOOD_TABLE.add(new Pair<>(192d, 57d)); // hood 5
        DIST_HOOD_TABLE.add(new Pair<>(204d, 57d)); // hood 5
        DIST_HOOD_TABLE.add(new Pair<>(216d, 63d)); // hood 5.5
        DIST_HOOD_TABLE.add(new Pair<>(228d, 63d)); // hood 5.5
    }
}
