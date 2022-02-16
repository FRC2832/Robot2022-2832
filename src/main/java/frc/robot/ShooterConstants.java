package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.Pair;
public class ShooterConstants {
    public final static ArrayList<Pair<Double,Double>> VISION_DIST_TABLE = new ArrayList<Pair<Double,Double>>();
    public final static ArrayList<Pair<Double,Double>> DIST_RPM_TABLE = new ArrayList<Pair<Double,Double>>();
    public final static ArrayList<Pair<Double,Double>> DIST_HOOD_TABLE = new ArrayList<Pair<Double,Double>>();

    public static void LoadConstants() {
        //table is input: pixel Y, output: mm from target
        VISION_DIST_TABLE.add(new Pair<Double, Double>(94.0, 7.7724));
        VISION_DIST_TABLE.add(new Pair<Double, Double>(245.0, 2.667));
        VISION_DIST_TABLE.add(new Pair<Double, Double>(298.0, 2.1336));
        VISION_DIST_TABLE.add(new Pair<Double, Double>(372.0, 1.651));

        //table is input: distance in mm, output: rpm
        DIST_RPM_TABLE.add(new Pair<Double, Double>(1000d, 2350d));
        DIST_RPM_TABLE.add(new Pair<Double, Double>(1300d, 2600d));
        DIST_RPM_TABLE.add(new Pair<Double, Double>(1700d, 2400d));
        DIST_RPM_TABLE.add(new Pair<Double, Double>(2000d, 2550d));

        //table is input: distance in mm, output: angle in degrees
        DIST_HOOD_TABLE.add(new Pair<Double, Double>(1000d, 20d));
        DIST_HOOD_TABLE.add(new Pair<Double, Double>(1300d, 30d));
        DIST_HOOD_TABLE.add(new Pair<Double, Double>(1700d, 25d));
        DIST_HOOD_TABLE.add(new Pair<Double, Double>(2000d, 42d));
    }
}
