package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase 
{
    final double SENSOR_UNITS_TO_RPM = 3.414;
    TalonFX shooterFx;
    //TalonSRX hoodMotor;

    //TODO: write home hood method
    //Distance to Target
    //Hood Angle
    //Turn robot to goal
    //Turret Angle?
    
    public Shooter() {
        // Example usage of a TalonSRX motor controller
        shooterFx = new TalonFX(0); // creates a new TalonSRX with ID 0
        shooterFx.setNeutralMode(NeutralMode.Coast);
        //hoodMotor = new TalonSRX(2);

        TalonFXConfiguration config = new TalonFXConfiguration();
        //PID values from calibration on field, 6878 units/100ms = 32.3% power, 47.89% = 9892, 16.62%=3572 units=1046 rpm
        config.slot0.kP = 0.8;
        config.slot0.kI = 0.001;
        config.slot0.kD = 16;
        config.slot0.kF = 0.05205;
        config.slot0.integralZone = 65;
        config.closedloopRamp = 0.1;         //take 100ms to ramp to max power
        shooterFx.configAllSettings(config); // apply the config settings; this selects the quadrature encoder
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Output Velocity", getShooterVelocity());
        SmartDashboard.putNumber("Hood Angle Position", getHoodAngle());
    }

    public void setShootPct(double percent) {
        shooterFx.set(ControlMode.PercentOutput, percent);
    }

    public void setShooterRpm(double rpm) {
        shooterFx.set(ControlMode.Velocity, rpm * SENSOR_UNITS_TO_RPM);
    }

    public void setHoodAngle(double position) {
        //TODO: set hood angle
    }

    /**
     * Get shooter speed in RPM
     * @return RPM
     */
    public double getShooterVelocity() {
        return shooterFx.getSelectedSensorVelocity() / SENSOR_UNITS_TO_RPM;
    }

    public double getHoodAngle() {
        //TODO: add angle scale factor and zeroing
        //return hoodMotor.getSelectedSensorPosition();
        return 0;
    }
}
