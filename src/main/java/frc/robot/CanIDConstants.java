package frc.robot;

public class CanIDConstants {

    // only for comp bot
    // swerve order: "FL", "FR", "RL", "RR"

    // swerve motors
    public static final byte[] SWERVE_DRIVES = { 32, 28, 38, 21 };
    public static final byte[] SWERVE_ROTS = { 30, 29, 39, 20 };
    public static final byte[] SWERVE_ROT_SENSORS = { 50, 49, 59, 40 };
    public static final double[] SWERVE_ZEROS = { 88.769531, -5.449219, 168.134766, -92.460938 };

    // intake and slipstream
    public static final int INTAKE_WHEELS = 27;
    public static final int INTAKE_LIFT = 31;
    public static final int STAGE_1 = 25;
    public static final int STAGE_2 = 23;

    // shooter
    public static final int SHOOTER_DRIVE = 24;

    // climber
    public static final int RUNG_1_2_WINCH = 33;
    public static final int RUNG_3_4_WINCH = 34;

    public static final int HOOD_MOTOR = 26;

}
