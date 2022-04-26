package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ColorSensor.CargoColor;
import frc.robot.commands.shooting.DribbleShoot;
import frc.robot.constants.CanIDConstants;
import frc.robot.constants.ShooterConstants;

import java.sql.Driver;
import java.util.ArrayList;

public class Shooter extends SubsystemBase {
    private static final double SENSOR_UNITS_TO_RPM = 3.414;
    // private static final int HOOD_SENSOR_ACTIVE = 700;
    private static final int MAX_ANGLE_COUNTS = 400;
    public static final int MIN_ANGLE = 20;
    public static final int MAX_ANGLE = 70;
    private static boolean coastMotor;
    private final TalonFX shooterFx;
    private final TalonSRX hoodMotor;
    private final XboxController driveController;
    private final XboxController operatorController;
    private final Ingestor ingestor;
    private boolean isHomed; // report if hood has been homed
    private boolean lastHomed;
    // private double hoodSensorOffset;
    private double distance;
    private double targetHoodAngle;
    private double targetRpm;
    private CargoColor currentCargoColor;
    private CargoColor previousCargoColor;

    // TODO: write home hood method
    // Distance to Target
    // Hood Angle
    // Turn robot to goal
    // Turret Angle?

    public Shooter(XboxController driveController, XboxController operatorController, Ingestor ingestor) {
        super();
        this.driveController = driveController;
        this.operatorController = operatorController;
        this.ingestor = ingestor;
        currentCargoColor = CargoColor.Unknown;
        previousCargoColor = CargoColor.Unknown;
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
        hoodConfig.slot0.kI = 0.2;
        hoodConfig.slot0.kD = 90;
        hoodConfig.slot0.integralZone = 30;
        hoodConfig.slot0.allowableClosedloopError = 2;
        hoodConfig.slot0.closedLoopPeakOutput = 0.5;
        hoodMotor.configAllSettings(hoodConfig);
    }

    public static boolean getCoast() {
        return coastMotor;
    }

    /*
     * public void setShootPct(double percent) {
     * shooterFx.set(ControlMode.PercentOutput, percent);
     * }
     */

    public static void setCoast(boolean coast) {
        coastMotor = coast;
    }

    @Override
    public void periodic() {
        if (DriverStation.isTest()) {
            return;
        }
        previousCargoColor = currentCargoColor;
        currentCargoColor = ColorSensor.getCargoColor();

        // System.out.println("currentCargoColor: " + currentCargoColor);

        CargoColor allianceColor = CargoColor.Blue;
        Alliance alliance = DriverStation.getAlliance();
        if (alliance == Alliance.Red) {
            allianceColor = CargoColor.Red;
        }

        if (currentCargoColor == CargoColor.Unknown) {
            // no ball detected, so don't shoot
        } else if (currentCargoColor != allianceColor && previousCargoColor != allianceColor) {
            CommandScheduler.getInstance().schedule(new DribbleShoot(this, ingestor));
        } else {
            // call Autoshoot (TODO)
            // print "AUTOSHOOTING!!!"
        }

        double shooterVelocity = getShooterVelocity();
        double hoodAngle = getHoodAngle();

        SmartDashboard.putNumber("Shooter Output Velocity", shooterVelocity);
        SmartDashboard.putNumber("Hood Angle Position", hoodAngle);
        SmartDashboard.putNumber("Shot Distance", distance);
        SmartDashboard.putNumber("Calc RPM", targetRpm);
        SmartDashboard.putNumber("Calc Hood Angle", targetHoodAngle);
        // if the limit switch is pressed, reset the hood angle position
        // when we exit the home position, save the zero position
        if (lastHomed && !hoodBottom()) {
            hoodMotor.setSelectedSensorPosition(0);
            isHomed = true;
        }
        // if (hoodMotor.isRevLimitSwitchClosed() > 0) {
        // hoodMotor.setSelectedSensorPosition(0);
        // isHomed = true;
        // System.out.println("isHomed = true");
        // }
        lastHomed = hoodBottom();
        double hoodMotorSpeed = hoodMotor.getMotorOutputPercent();
        NeutralMode mode = NeutralMode.Brake;

        if (!operatorController.getStartButton()) {
            if (driveController.getLeftBumper()) {
                hoodMotorSpeed = 0.35;
            } else if (driveController.getRightBumper()) {
                if (isHomed && hoodMotor.isRevLimitSwitchClosed() > 0) {
                    hoodMotorSpeed = 0.0;
                } else {
                    hoodMotorSpeed = -0.35;
                }
            } else {
                hoodMotorSpeed = 0.0;
            }
        }

        if (DriverStation.isDisabled()) {
            mode = NeutralMode.Coast;
        }
        hoodMotor.set(ControlMode.PercentOutput, hoodMotorSpeed);
        hoodMotor.setNeutralMode(mode);
        // System.out.println(hoodMotorSpeed);
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
        return (sensor / MAX_ANGLE_COUNTS) * (MAX_ANGLE - MIN_ANGLE) + MIN_ANGLE;
    }

    public void setHoodAngle(double position) {
        double value = (position - MIN_ANGLE) * MAX_ANGLE_COUNTS / (MAX_ANGLE - MIN_ANGLE);
        hoodMotor.set(ControlMode.Position, value);
    }

    public void setHoodCoastMode(NeutralMode mode) {
        if (DriverStation.isTest()) {
            hoodMotor.setNeutralMode(mode);
        }
    }

    private boolean hoodBottom() {
        return hoodMotor.isRevLimitSwitchClosed() > 0;
    }

    public double getShotDist() {
        return distance;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getTargetHoodAngle() {
        return targetHoodAngle;
    }

    public void setShooterRpm(double rpm) {
        shooterFx.set(ControlMode.Velocity, rpm * SENSOR_UNITS_TO_RPM);
    }

    public boolean isHoodHomed() {
        return isHomed;
    }

    public void setHoodSpeedPct(double pct) {
        // allow control if homed or only down if not homed
        double percentage = pct;
        if (hoodBottom()) {
            if (pct > 0.1) {
                // if driving down, stop at home
                percentage = 0.0;
            } else {
                // slowly drive out to get accurate home
                percentage = 0.18;
            }
        } else if (hoodMotor.getSelectedSensorPosition() > MAX_ANGLE_COUNTS && pct > 0) {
            percentage = 0.0;
        }
        hoodMotor.set(ControlMode.PercentOutput, percentage);
    }

    public void calcShot(boolean useRunningAvg) {
        //double lidarDistance;
        //double visionDistance;
        // first, calculate distance to target
        // if (useRunningAvg) {
        //     lidarDistance = Lidar.getRunningAvg() * 39.3701; // inches
        // } else {
        //     lidarDistance = Lidar.getDistanceToTarget() * 39.3701;
        // }
        // SmartDashboard.putNumber("Lidar distance", lidarDistance);
        // double centerY = Pi.getTargetCenterY();
        // visionDistance = Pi.LinearInterp(ShooterConstants.VISION_DIST_TABLE, centerY); // inches
        // SmartDashboard.putNumber("Vision distance", visionDistance);
        // if (Math.abs(lidarDistance - visionDistance) > 40) { // TODO: change this on the practice field to account
        //  for lower hub wall in a different position
        //     distance = visionDistance;
        //     SmartDashboard.putBoolean("Using Lidar", false);
        //     targetHoodAngle = Pi.LinearInterp(ShooterConstants.DIST_HOOD_TABLE, distance);
        //     targetRpm = Pi.LinearInterp(ShooterConstants.DIST_RPM_TABLE, distance);
        // } else {
        //     distance = lidarDistance;
        //     SmartDashboard.putBoolean("Using Lidar", true);
        //     targetHoodAngle = Lidar.calculateTargetAngle(distance);
        //     targetRpm = Pi.LinearInterp(ShooterConstants.LIDAR_DIST_TABLE, distance / 39.3701); // table takes meters
        // }

        double centerY = Pi.getTargetCenterY();
        distance = LinearInterp(ShooterConstants.getVisionDistTable(), centerY);
        SmartDashboard.putBoolean("Using Lidar", false);
        targetHoodAngle = LinearInterp(ShooterConstants.getDistHoodTable(), distance);
        targetRpm = LinearInterp(ShooterConstants.getDistRpmTable(), distance);

        // TODO: only for tuning purposes!
        // boolean forceLidar = SmartDashboard.getBoolean("Force use lidar", true);
        // if (forceLidar) {
        //     distance = lidarDistance;
        //     SmartDashboard.putBoolean("Using Lidar", true);
        //     targetHoodAngle = Lidar.calculateTargetAngle(distance);
        //     targetRpm = Pi.LinearInterp(ShooterConstants.LIDAR_DIST_TABLE, distance / 39.3701); // table takes meters
        // } else {
        //     distance = visionDistance;
        //     SmartDashboard.putBoolean("Using Lidar", false);
        //     targetHoodAngle = Pi.LinearInterp(ShooterConstants.DIST_HOOD_TABLE, distance);
        //     targetRpm = Pi.LinearInterp(ShooterConstants.DIST_RPM_TABLE, distance);
        // }

        // if (Lidar.getIsConnected()) {
        //     targetHoodAngle = Lidar.calculateTargetAngle(distance);
        // } else {
        //     targetHoodAngle = Pi.LinearInterp(ShooterConstants.DIST_HOOD_TABLE, distance);
        // }
        // targetRpm = Pi.LinearInterp(ShooterConstants.LIDAR_DIST_TABLE, distance / 39.3701); // table takes meters
    }

    private static double LinearInterp(ArrayList<Pair<Double, Double>> list, double input) {
        // if input is smaller than the table, return the first element
        Pair<Double, Double> zeroPair = list.get(0);

        if (input < zeroPair.getFirst()) {
            return zeroPair.getSecond();
        }
        int size = list.size();
        Pair<Double, Double> lastPair = list.get(size - 1);
        // if input is larger than the table, return the last element
        if (input > lastPair.getFirst()) {
            return lastPair.getSecond();
        }
        // otherwise the value is in the table
        for (int i = 0; i < size - 1; i++) {
            Pair<Double, Double> iPair = list.get(i);
            Pair<Double, Double> iPlusOnePair = list.get(i + 1);
            double x0 = iPair.getFirst();
            double x1 = iPlusOnePair.getFirst();
            if ((x0 <= input) && (input <= x1)) {
                // see https://en.wikipedia.org/wiki/Linear_interpolation
                double y0 = iPair.getSecond();
                double y1 = iPlusOnePair.getSecond();
                return (y0 * (x1 - input) + y1 * (input - x0)) / (x1 - x0);
            }
        }
        // should never happen...
        return Double.NaN;
    }

    public double getShooterTemperature() {
        return shooterFx.getTemperature() * 1.8 + 32;
    }

    /*
     * public CargoColor getCurrentCargoColor() {
     * return currentCargoColor;
     * }
     */
}
