package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase 
{
    final double SENSOR_UNITS_TO_RPM = 3.414;
    TalonFX shooterFx;
    TalonSRX hoodMotor;
    boolean isHomed;    //report if hood has been homed
    double hoodSensorOffset;
    Pi pi;
    private double distance;
    private double hoodAngle;
    private double targetRpm;

    //TODO: write home hood method
    //Distance to Target
    //Hood Angle
    //Turn robot to goal
    //Turret Angle?
    
    public Shooter(Pi pi) {
        this.pi = pi;
        isHomed = false;
        // Example usage of a TalonSRX motor controller
        shooterFx = new TalonFX(0); // creates a new TalonSRX with ID 0
        shooterFx.setNeutralMode(NeutralMode.Coast);
        shooterFx.setInverted(false);
        hoodMotor = new TalonSRX(2);
        hoodMotor.setNeutralMode(NeutralMode.Brake);

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
        SmartDashboard.putNumber("Shot Distance", getShotDist());
        SmartDashboard.putNumber("Calc RPM", getTargetRpm());
        SmartDashboard.putNumber("Calc Hood Angle", getTargetHoodAngle());

        //if the limit switch is pressed, reset the hood angle position
        if (hoodMotor.isRevLimitSwitchClosed() > 0) {
            hoodMotor.setSelectedSensorPosition(0);
            isHomed = true;
        }
    }

    public void setShootPct(double percent) {
        shooterFx.set(ControlMode.PercentOutput, percent);
    }

    public void setShooterRpm(double rpm) {
        shooterFx.set(ControlMode.Velocity, rpm * SENSOR_UNITS_TO_RPM);
    }

    public boolean isHoodHomed() {
        return isHomed;
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
        return hoodMotor.getSelectedSensorPosition();
    }

    public void setHoodSpeedPct(double pct) { 
        //allow control if homed or only down if not homed
        if(isHomed == true || pct < 0) {
            hoodMotor.set(ControlMode.PercentOutput, pct);
        }
    }

    public void setHoodAngle(double position) {
        //TODO: set hood angle
    }

    public void calcShot() {
        //first, calculate distance to target
        double centerY = pi.getCenterY();
        distance = Pi.LinearInterp(ShooterConstants.VISION_DIST_TABLE, centerY);

        hoodAngle = Pi.LinearInterp(ShooterConstants.DIST_HOOD_TABLE, distance);
        targetRpm = Pi.LinearInterp(ShooterConstants.DIST_RPM_TABLE, distance);
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getTargetHoodAngle() {
        return hoodAngle;
    }

    public double getShotDist() {
        return distance;
    }
}
