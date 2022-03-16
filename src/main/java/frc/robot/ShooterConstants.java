package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.Pair;
public class ShooterConstants {
    public final static ArrayList<Pair<Double,Double>> VISION_DIST_TABLE = new ArrayList<Pair<Double,Double>>();
    public final static ArrayList<Pair<Double,Double>> DIST_RPM_TABLE = new ArrayList<Pair<Double,Double>>();
    public final static ArrayList<Pair<Double,Double>> DIST_HOOD_TABLE = new ArrayList<Pair<Double,Double>>();

    public static void LoadConstants() {
        //table is input: pixel Y, output: in from target
        // data from Nighttime (3rd set) sheet in 2022 shooter vision table
        VISION_DIST_TABLE.add(new Pair<Double, Double>(86.0, 60d));
        VISION_DIST_TABLE.add(new Pair<Double, Double>(136.5, 72d));
        VISION_DIST_TABLE.add(new Pair<Double, Double>(173.5, 84d));
        VISION_DIST_TABLE.add(new Pair<Double, Double>(210.5, 96d));
        VISION_DIST_TABLE.add(new Pair<Double, Double>(242.5, 108d));
        VISION_DIST_TABLE.add(new Pair<Double, Double>(267.0, 120d));
        VISION_DIST_TABLE.add(new Pair<Double, Double>(291.0, 132d));
        VISION_DIST_TABLE.add(new Pair<Double, Double>(310.5, 144d));
        VISION_DIST_TABLE.add(new Pair<Double, Double>(331.0, 156d));
        VISION_DIST_TABLE.add(new Pair<Double, Double>(347.0, 168d));
        VISION_DIST_TABLE.add(new Pair<Double, Double>(358.0, 180d));
        VISION_DIST_TABLE.add(new Pair<Double, Double>(367.0, 192d));
        VISION_DIST_TABLE.add(new Pair<Double, Double>(384.0, 204d));
        VISION_DIST_TABLE.add(new Pair<Double, Double>(404.0, 216d));
        VISION_DIST_TABLE.add(new Pair<Double, Double>(406.5, 228d));

        //table is input: distance in in, output: rpm
        // from auto shots sheet in 2022 shooter speed table
        DIST_RPM_TABLE.add(new Pair<Double, Double>(60d, 2200d));
        DIST_RPM_TABLE.add(new Pair<Double, Double>(72d, 2350d));
        DIST_RPM_TABLE.add(new Pair<Double, Double>(84d, 2400d));
        DIST_RPM_TABLE.add(new Pair<Double, Double>(96d, 2450d));
        DIST_RPM_TABLE.add(new Pair<Double, Double>(108d, 2550d));
        DIST_RPM_TABLE.add(new Pair<Double, Double>(120d, 2650d));
        DIST_RPM_TABLE.add(new Pair<Double, Double>(132d, 2750d));
        DIST_RPM_TABLE.add(new Pair<Double, Double>(144d, 2800d));
        DIST_RPM_TABLE.add(new Pair<Double, Double>(156d, 2800d));
        DIST_RPM_TABLE.add(new Pair<Double, Double>(168d, 2850d));
        DIST_RPM_TABLE.add(new Pair<Double, Double>(180d, 3000d));
        DIST_RPM_TABLE.add(new Pair<Double, Double>(192d, 3050d));
        DIST_RPM_TABLE.add(new Pair<Double, Double>(204d, 3100d));
        DIST_RPM_TABLE.add(new Pair<Double, Double>(216d, 3100d));
        DIST_RPM_TABLE.add(new Pair<Double, Double>(228d, 3250d));
        
        //table is input: distance in in, output: angle in degrees
        // from auto shots sheet in 2022 shooter speed table
        // 88 degrees is when the hood is down and touching the limit switch
        // 36 degrees is when the hood is practically all the way up at knob 6
        // formula for measured angle conversion to angle in this table: 88 - measured value + 20
        DIST_HOOD_TABLE.add(new Pair<Double, Double>(60d, 31d)); // hood 2.5
        DIST_HOOD_TABLE.add(new Pair<Double, Double>(72d, 31d)); // hood 2.5
        DIST_HOOD_TABLE.add(new Pair<Double, Double>(84d, 31d)); // hood 2.5
        DIST_HOOD_TABLE.add(new Pair<Double, Double>(96d, 42d)); // hood 3.5
        DIST_HOOD_TABLE.add(new Pair<Double, Double>(108d, 42d)); // hood 3.5   
        DIST_HOOD_TABLE.add(new Pair<Double, Double>(120d, 53d)); // hood 4.5
        DIST_HOOD_TABLE.add(new Pair<Double, Double>(132d, 58d)); // hood 5
        DIST_HOOD_TABLE.add(new Pair<Double, Double>(144d, 53d)); // hood 4.5
        DIST_HOOD_TABLE.add(new Pair<Double, Double>(156d, 53d)); // hood 4.5
        DIST_HOOD_TABLE.add(new Pair<Double, Double>(168d, 53d)); // hood 4.5 
        DIST_HOOD_TABLE.add(new Pair<Double, Double>(180d, 58d)); // hood 5
        DIST_HOOD_TABLE.add(new Pair<Double, Double>(192d, 58d)); // hood 5
        DIST_HOOD_TABLE.add(new Pair<Double, Double>(204d, 58d)); // hood 5 
        DIST_HOOD_TABLE.add(new Pair<Double, Double>(216d, 63d)); // hood 5.5
        DIST_HOOD_TABLE.add(new Pair<Double, Double>(228d, 63d)); // hood 5.5

    }
}
