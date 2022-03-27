package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DribbleShoot;

public class Shooter extends SubsystemBase {
    private final double SENSOR_UNITS_TO_RPM = 3.414;
    private TalonFX shooterFx;
    private TalonSRX hoodMotor;
    private boolean isHomed; // report if hood has been homed
    private boolean lastHomed;
    //private double hoodSensorOffset;
    private double distance;
    private double targetHoodAngle;
    private double targetRpm;
    private XboxController driveController;
    private XboxController operatorController;
    private static boolean coastMotor = false;
    final int HOOD_SENSOR_ACTIVE = 700;
    final int MAX_ANGLE_COUNTS = 400;
    final int MIN_ANGLE = 20;
    final int MAX_ANGLE = 70;
    private String currentCargoColor;
    private Ingestor ingestor;

    // TODO: write home hood method
    // Distance to Target
    // Hood Angle
    // Turn robot to goal
    // Turret Angle?

    public Shooter(XboxController driveController, XboxController operatorController, Ingestor ingestor) {
        this.driveController = driveController;
        this.operatorController = operatorController;
        this.ingestor = ingestor;
        currentCargoColor = "none";
        isHomed = false;
        hoodMotor = new TalonSRX(CanIDConstants.HOOD_MOTOR);
        hoodMotor.setNeutralMode(NeutralMode.Brake);

        shooterFx = new TalonFX(CanIDConstants.SHOOTER_DRIVE);
        shooterFx.setNeutralMode(NeutralMode.Coast);
        shooterFx.setInverted(false);
        // hoodMotor.setNeutralMode(NeutralMode.Brake);

        TalonFXConfiguration config = new TalonFXConfiguration();
        // PID values from calibration on field, 6878 units/100ms = 32.3% power, 47.89%
        // = 9892, 16.62%=3572 units=1046 rpm
        config.slot0.kP = 0.8;
        config.slot0.kI = 0.001;
        config.slot0.kD = 16;
        config.slot0.kF = 0.05205;
        config.slot0.integralZone = 65;
        config.closedloopRamp = 0.1; // take 100ms to ramp to max power
        shooterFx.configAllSettings(config); // apply the config settings; this selects the quadrature encoder

        TalonSRXConfiguration hoodConfig = new TalonSRXConfiguration();
        hoodMotor.getAllConfigs(hoodConfig);
        hoodConfig.slot0.kP = 9;
        hoodConfig.slot0.kI = 0.1;
        hoodConfig.slot0.kD = 90;
        hoodConfig.slot0.integralZone = 30;
        hoodConfig.slot0.allowableClosedloopError = 2;
        hoodConfig.slot0.closedLoopPeakOutput = 0.5;
        hoodMotor.configAllSettings(hoodConfig);
    }

    @Override
    public void periodic() {
        currentCargoColor = ColorSensor.getCargoColor();

        //System.out.println("currentCargoColor: " + currentCargoColor);

        String allianceString;
        Alliance alliance = DriverStation.getAlliance();
        if(alliance == Alliance.Red) {
            allianceString = "Red";
        } else {
            allianceString = "Blue";
        }

        if(currentCargoColor.equals("Unknown")){
            // no ball detected, so don't shoot
        }
        // if detected color != alliance color 
        else if(!(currentCargoColor.equalsIgnoreCase(allianceString))){
            CommandScheduler.getInstance().schedule(new DribbleShoot(this, ingestor));
        }
        else{
            // call Autoshoot (TODO)
            // print "AUTOSHOOTING!!!"
        }

        SmartDashboard.putNumber("Shooter Output Velocity", getShooterVelocity());
        SmartDashboard.putNumber("Hood Angle Position", getHoodAngle());
        SmartDashboard.putNumber("Shot Distance", getShotDist());
        SmartDashboard.putNumber("Calc RPM", getTargetRpm());
        SmartDashboard.putNumber("Calc Hood Angle", getTargetHoodAngle());
        // if the limit switch is pressed, reset the hood angle position
        //when we exit the home position, save the zero position
        if (lastHomed == true && !hoodBottom()){
            hoodMotor.setSelectedSensorPosition(0);
            isHomed = true;
        }
        // if (hoodMotor.isRevLimitSwitchClosed() > 0) {
        //     hoodMotor.setSelectedSensorPosition(0);
        //     isHomed = true;
        //     System.out.println("isHomed = true");
        // }
        lastHomed = hoodBottom();
        double hoodMotorSpeed = hoodMotor.getMotorOutputPercent();
        NeutralMode mode;

        if (!operatorController.getStartButton()) {
            if (driveController.getLeftBumper()) {
                hoodMotorSpeed = 0.25;
            } else if (driveController.getRightBumper()) {
                if (isHomed && hoodMotor.isRevLimitSwitchClosed() > 0) {
                    hoodMotorSpeed = 0.0;
                } else {
                    hoodMotorSpeed = -0.25;
                }
            } else {
                hoodMotorSpeed = 0.0;
            }
        }

        if (DriverStation.isDisabled()) {
            mode = NeutralMode.Coast;
        } else {
            mode = NeutralMode.Brake;
        }
        hoodMotor.set(ControlMode.PercentOutput, hoodMotorSpeed);
        hoodMotor.setNeutralMode(mode);
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
     * 
     * @return RPM
     */
    public double getShooterVelocity() {
        return shooterFx.getSelectedSensorVelocity() / SENSOR_UNITS_TO_RPM;
    }

    public double getHoodAngle() {
        double sensor = hoodMotor.getSelectedSensorPosition();
        return (sensor/MAX_ANGLE_COUNTS)*(MAX_ANGLE-MIN_ANGLE) + MIN_ANGLE;
    }

    public void setHoodSpeedPct(double pct) {
        // allow control if homed or only down if not homed
        double percentage;
        if(hoodBottom()) {
            if(pct > 0.1) {
                //if driving down, stop at home
                percentage = 0.0;
            } else {
                //slowly drive out to get accurate home
                percentage = 0.18;
            }
        }
        else if(hoodMotor.getSelectedSensorPosition() > MAX_ANGLE_COUNTS && pct > 0){
            percentage = 0.0;
        }
        else {
            percentage = pct;
        }
        hoodMotor.set(ControlMode.PercentOutput, percentage);
    }

    public boolean hoodBottom() {
        return hoodMotor.isRevLimitSwitchClosed() > 0;
    }

    public void setHoodAngle(double position) {
        double value = (position-MIN_ANGLE) * MAX_ANGLE_COUNTS/ (MAX_ANGLE-MIN_ANGLE);
        hoodMotor.set(ControlMode.Position, value);
    }

    public void calcShot() {
        // first, calculate distance to target
        double centerY = Pi.getTargetCenterY();
        distance = Pi.LinearInterp(ShooterConstants.VISION_DIST_TABLE, centerY);

        targetHoodAngle = Pi.LinearInterp(ShooterConstants.DIST_HOOD_TABLE, distance);
        targetRpm = Pi.LinearInterp(ShooterConstants.DIST_RPM_TABLE, distance);
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getTargetHoodAngle() {
        return targetHoodAngle;
    }

    public double getShotDist() {
        return distance;
    }

    public static void setCoast(boolean coast) {
        coastMotor = coast;
    }

    public static boolean getCoast() {
        return coastMotor;
    }

    public String getCurrentCargoColor(){
        return currentCargoColor;
    }
}
