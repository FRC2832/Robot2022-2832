package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class SwerveConstants {
    public String Name;
    public int DriveMotorId;
    public int TurnMotorId;
    public int CanCoderId;
    public Translation2d Location;
    public double ZeroAngle;
    
    public DCMotor TurnMotor;
    public double  TurnMotorGearRatio;
    public double  TurnMotorKv;
    public double  TurnMotorKa;
    public double  TurnMotorP;
    public double  TurnMotorI;
    public double  TurnMotorD;

    public DCMotor DriveMotor;
    public double  DriveMotorGearRatio;
    public double  DriveMotorKv;
    public double  DriveMotorKa;
    public double  DriveMotorP;
    public double  DriveMotorI;
    public double  DriveMotorD;
    public double  DriveMotorFF;
    public double  DriveMotorIZone;
}
