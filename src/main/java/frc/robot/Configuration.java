package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Configuration {
    private static String[] wheelIds = new String[] { "FL", "FR", "RL", "RR" };
    private static NetworkTable table;
    private static final String SHOOTER_KEY = "Shooter CanId";
    private static final String DRIVEMOTOR_KEY = "DriveMotor CanId ";
    private static final String TURNMOTOR_KEY = "TurnMotor CanId ";
    private static final String CANCODER_KEY = "CanCoder CanId ";
    private static final String ZEROANGLE_KEY = "ZeroAngle ";

    public static String GetWheelName(byte whl) {
        if (whl < 0 || whl > 3)
            return "";
        return wheelIds[whl];
    }

    public static void SetPersistentKeys() {
        loadTable();

        makePeristent(table.getEntry(SHOOTER_KEY), 24);
        for (byte i = 0; i < 4; i++) {
            String id = GetWheelName(i);
            makePeristent(table.getEntry(DRIVEMOTOR_KEY + id), i);
            makePeristent(table.getEntry(TURNMOTOR_KEY + id), i + 10);
            makePeristent(table.getEntry(CANCODER_KEY + id), i + 20);
            makePeristent(table.getEntry(ZEROANGLE_KEY + id), 0);
        }
    }

    public static int GetDriveMotorId(byte whl) {
        loadTable();
        return (int) table.getEntry(DRIVEMOTOR_KEY + GetWheelName(whl)).getDouble(0);
    }

    public static int GetTurnMotorId(byte whl) {
        loadTable();
        return (int) table.getEntry(TURNMOTOR_KEY + GetWheelName(whl)).getDouble(0);
    }

    public static int GetCanCoderId(byte whl) {
        loadTable();
        return (int) table.getEntry(CANCODER_KEY + GetWheelName(whl)).getDouble(0);
    }

    public static double GetZeroAngle(byte whl) {
        loadTable();
        return table.getEntry(ZEROANGLE_KEY + GetWheelName(whl)).getDouble(0);
    }

    public static int GetShooterId() {
        loadTable();
        return (int) table.getEntry(SHOOTER_KEY).getDouble(0);
    }

    private static void loadTable() {
        if (table == null) {
            table = NetworkTableInstance.getDefault().getTable("Config");
        }
    }

    private static void makePeristent(NetworkTableEntry entry, Number value) {
        if (!entry.isPersistent()) {
            entry.setNumber(value);
            entry.setPersistent();
        }
    }

    /*
     * Sample Tables:
     * 
     * Swerve Bot
     * [NetworkTables Storage 3.0]
     * double "/Config/CanCoder CanId FL"=3
     * double "/Config/CanCoder CanId FR"=0
     * double "/Config/CanCoder CanId RL"=1
     * double "/Config/CanCoder CanId RR"=2
     * double "/Config/DriveMotor CanId FL"=7
     * double "/Config/DriveMotor CanId FR"=5
     * double "/Config/DriveMotor CanId RL"=4
     * double "/Config/DriveMotor CanId RR"=6
     * double "/Config/Shooter CanId"=0
     * double "/Config/TurnMotor CanId FL"=8
     * double "/Config/TurnMotor CanId FR"=9
     * double "/Config/TurnMotor CanId RL"=11
     * double "/Config/TurnMotor CanId RR"=10
     * double "/Config/ZeroAngle FL"=-6.1
     * double "/Config/ZeroAngle FR"=47.9
     * double "/Config/ZeroAngle RL"=25.2
     * double "/Config/ZeroAngle RR"=-153.1
     * 
     * Competition Bot
     * [NetworkTables Storage 3.0]
     * double "/Config/CanCoder CanId FL"=50
     * double "/Config/CanCoder CanId FR"=49
     * double "/Config/CanCoder CanId RL"=59
     * double "/Config/CanCoder CanId RR"=40
     * double "/Config/DriveMotor CanId FL"=32
     * double "/Config/DriveMotor CanId FR"=28
     * double "/Config/DriveMotor CanId RL"=38
     * double "/Config/DriveMotor CanId RR"=21
     * double "/Config/Shooter CanId"=24
     * double "/Config/TurnMotor CanId FL"=30
     * double "/Config/TurnMotor CanId FR"=29
     * double "/Config/TurnMotor CanId RL"=39
     * double "/Config/TurnMotor CanId RR"=40
     * double "/Config/ZeroAngle FL"=92.5
     * double "/Config/ZeroAngle FR"=-3.39
     * double "/Config/ZeroAngle RL"=164
     * double "/Config/ZeroAngle RR"=-81.8
     */
}
