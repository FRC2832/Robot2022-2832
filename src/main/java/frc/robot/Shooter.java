package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase 
{
    TalonSRX shooterSrx;
    TalonSRX hoodMotor;
    //TODO: write home hood method

    //Distance to Target
    //Shooter Speed (RPM)
    //Hood Angle
    //Turret Angle?
    
    public Shooter() {
        // Example usage of a TalonSRX motor controller
        shooterSrx = new TalonSRX(0); // creates a new TalonSRX with ID 0
        shooterSrx.setNeutralMode(NeutralMode.Coast);
        hoodMotor = new TalonSRX(2);

        TalonSRXConfiguration config = new TalonSRXConfiguration();
        config.peakCurrentLimit = 40; // the peak current, in amps
        config.peakCurrentDuration = 1500; // the time at the peak current before the limit triggers, in ms
        config.continuousCurrentLimit = 30; // the current to maintain if the peak limit is triggered
        shooterSrx.configAllSettings(config); // apply the config settings; this selects the quadrature encoder
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Output Position", shooterSrx.getSelectedSensorPosition());
        SmartDashboard.putNumber("Shooter Output Velocity", getShooterVelocity());
        SmartDashboard.putNumber("Hood Angle Position", getHoodAngle());
    }

    public void setShootPct(double percent) {
        shooterSrx.set(ControlMode.PercentOutput, percent);
    }

    public void setShooterRpm(double rpm) {
        //TODO: set scale factor for RPM
        shooterSrx.set(ControlMode.Velocity, rpm);
    }

    public void setHoodAngle(double position) {
        //TODO: set hood angle
    }

    public double getShooterVelocity() {
        //TODO: convert raw units to RPM
        return shooterSrx.getSelectedSensorVelocity();
    }

    public double getHoodAngle() {
        //TODO: add angle scale factor and zeroing
        return hoodMotor.getSelectedSensorPosition();
    }
}
