package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class SwerveConstants {
    public String Name;
    public Translation2d Location;
    public byte Id;
    public byte TalonFXId;
    public byte SparkId;

    public DCMotor TurnMotor;
    public double TurnMotorGearRatio;
    public double TurnMotorKv;
    public double TurnMotorKa;
    public double TurnMotorP;
    public double TurnMotorI;
    public double TurnMotorD;

    public DCMotor DriveMotor;
    public double DriveMotorGearRatio;
    public double DriveMotorKv;
    public double DriveMotorKa;
    public double DriveMotorP;
    public double DriveMotorI;
    public double DriveMotorD;
    public double DriveMotorFF;
    public double DriveMotorIZone;
}
