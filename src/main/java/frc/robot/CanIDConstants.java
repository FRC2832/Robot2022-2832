package frc.robot;

public class CanIDConstants {

    // only for comp bot
    // swerve order: "FL", "FR", "RL", "RR"

    // swerve motors
    public static byte[] swerveDrives = {32, 28, 38, 21};
    public static byte[] swerveRots = {30, 29, 39, 20};
    public static byte[] swerveRotSensors = {50, 49, 59, 40};
    public static double[] swerveZeros = {93.52, -2.29, 178, -92.5};

    // intake and slipstream
    public static int intakeWheels = 27;
    public static int intakeLift = 31;
    public static int stage1 = 25;
    public static int stage2 = 23;

    // shooter
    public static int shooterDrive = 24;
    
    // climber
    public static int rung12Winch = 33;
    public static int rung34Winch = 34;

    public static int hoodMotor = 26;
    
}
