
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

/**
 * Represents a swerve drive style drivetrain.
 */
public class Drivetrain extends SubsystemBase {
    public static final int FL = 0;
    public static final int FR = 1;
    public static final int RL = 2;
    public static final int RR = 3;

    public static double kMaxSpeed = 3.0; // per Thrifty Bot, max speed with Falcon 500 is 15.9ft/s, or 4.85 m/s
    public static double kMaxAngularSpeed = 2 * Math.PI; // 1 rotation per second
    private static int loops;
    private final SwerveModule[] MODULES = new SwerveModule[4];
    private final SwerveConstants[] CONSTANTS = new SwerveConstants[4];
    //private final Pose2d[] modulePoses = new Pose2d[4];
    // private final AnalogGyro gyro = new AnalogGyro(0);
    private final PigeonIMU pigeon;
    private final SwerveDriveKinematics kinematics;
    private final Field2d field = new Field2d();
    // public SwerveDriveOdometry odometry;
    private int currentStep;
    private NetworkTable visionTable;
    private XboxController driverController;
    private Translation2d[] redBalls;
    private Translation2d[] blueBalls;
    //private ADXRS450_GyroSim gyroSim;
    private ADXRS450_Gyro gyroBase;

    public Drivetrain(XboxController driverController) {
        this.driverController = driverController;
        if (Robot.isSimulation()) {
            gyroBase = new ADXRS450_Gyro();
            //gyroSim = new ADXRS450_GyroSim(gyroBase);

            redBalls = new Translation2d[8];
            redBalls[0] = new Translation2d(8.57, 7.53);
            redBalls[1] = new Translation2d(10.85, 6.13);
            redBalls[2] = new Translation2d(10.95, 2.24);
            redBalls[3] = new Translation2d(14.4, 6.77);
            redBalls[4] = new Translation2d(14.89, 5.85); // virtual ball returned from human player near driver station
            // far side
            redBalls[5] = new Translation2d(8.75, 0.68);
            redBalls[6] = new Translation2d(4.57, 3.34);
            redBalls[7] = new Translation2d(5.99, 6.95);

            blueBalls = new Translation2d[8];
            blueBalls[0] = new Translation2d(5.06, 5.96);
            blueBalls[1] = new Translation2d(5.13, 2.06);
            blueBalls[2] = new Translation2d(7.38, 0.66);
            blueBalls[3] = new Translation2d(1.55, 1.43);
            blueBalls[4] = new Translation2d(1.08, 2.31); // virtual ball returned from human player near driver station
            // far side
            blueBalls[5] = new Translation2d(7.23, 7.48);
            blueBalls[6] = new Translation2d(11.40, 4.87);
            blueBalls[7] = new Translation2d(10.80, 1.27);

            NetworkTableInstance inst = NetworkTableInstance.getDefault();
            visionTable = inst.getTable("/vision");
        }

        TalonSRX motor = new TalonSRX(CanIDConstants.INTAKE_WHEELS);
        pigeon = new PigeonIMU(motor);

        // set defaults for all swerve moules
        for (int i = 0; i < CONSTANTS.length; i++) {
            CONSTANTS[i] = new SwerveConstants();
            CONSTANTS[i].Id = (byte) i;
            CONSTANTS[i].TurnMotor = DCMotor.getNeo550(1);
            CONSTANTS[i].TurnMotorGearRatio = 12.0 * (64.0 / 12); // 12:1 on the motor, 5.33 in the swerve
            CONSTANTS[i].DriveMotor = DCMotor.getFalcon500(1);
            CONSTANTS[i].DriveMotorGearRatio = 5.25; // 12t to 21t gear stage, 15t to 45t bevel gear stage

            CONSTANTS[i].TurnMotorP = 0.0;
            CONSTANTS[i].TurnMotorI = 0.0;
            CONSTANTS[i].TurnMotorD = 0.0;

            CONSTANTS[i].DriveMotorP = 0.0;
            CONSTANTS[i].DriveMotorI = 0.0;
            CONSTANTS[i].DriveMotorD = 0.0;
            CONSTANTS[i].DriveMotorFF = 0.0;
            CONSTANTS[i].DriveMotorIZone = 0.0;

            // TODO: These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
            // this should be once for the drivetrain
            CONSTANTS[i].DriveMotorKv = 4.0; // kvVoltSecondsPerMeter (default = 12/kMaxSpeed)
            CONSTANTS[i].DriveMotorKa = 0.0917; // kaVoltSecondsSquaredPerMeter
            // this should be done per turning motor
            CONSTANTS[i].TurnMotorKv = 0.6095; // VoltSecondsPerRadian (default = 12/19.686 (188RPM = 19.686Rad/S))
            CONSTANTS[i].TurnMotorKa = 0.0348; // VoltSecondsSquaredPerRadian = 0.0348
        }
        // per corner constants
        CONSTANTS[FL].Name = "SwerveDrive_FL";
        CONSTANTS[FL].Location = new Translation2d(0.261, 0.261);

        CONSTANTS[FR].Name = "SwerveDrive_FR";
        CONSTANTS[FR].Location = new Translation2d(0.261, -0.261);

        CONSTANTS[RL].Name = "SwerveDrive_RL";
        CONSTANTS[RL].Location = new Translation2d(-0.261, 0.261);

        CONSTANTS[RR].Name = "SwerveDrive_RR";
        CONSTANTS[RR].Location = new Translation2d(-0.261, -0.261);

        // create the swerve modules
        for (int i = 0; i < MODULES.length; i++) {
            MODULES[i] = new SwerveModule(CONSTANTS[i]);
        }
        kinematics = new SwerveDriveKinematics(CONSTANTS[FL].Location, CONSTANTS[FR].Location, CONSTANTS[RL].Location,
                CONSTANTS[RR].Location);
        // odometry = new SwerveDriveOdometry(kinematics, getHeading());

        // set the robot to x=0.5m, y=4m, rot=0*
        // odometry.resetPosition(new Pose2d(0.5, 4.0, new Rotation2d()), new
        // Rotation2d());

        pigeon.clearStickyFaults();
        SmartDashboard.putData("Field", field);
        // SmartDashboard.putBoolean("Reset Position", false);
    }

    public Rotation2d getHeading() {
        // TODO: can we get rid of this allocation
        return new Rotation2d(Math.toRadians(getAngle()));
    }

    /**
     * Gets the robot's angle in degrees
     *
     * @return Robot's angle since powerup
     */
    public double getAngle() {
        if (Robot.isReal()) {
            double[] ypr_deg = new double[3];
            pigeon.getYawPitchRoll(ypr_deg);
            return ypr_deg[0];
        }
        return gyroBase.getAngle();
    }

    /**
     * Reset the orientation of the robot (and in simulation, also the position)
     */
    public void resetRobot() {
        // odometry.resetPosition(new Pose2d(0.5, 4.0, getHeading()), getHeading());
    }

    /**
     * Updates the field relative position of the robot.
     */
    /*
     * public void updateOdometry() {
     * // update our estimation where we are on the field
     * /*odometry.update(getHeading(), MODULES[FL].getState(),
     * MODULES[FR].getState(), MODULES[RL].getState(),
     * MODULES[RR].getState());
     * Pose2d pose = getPose();
     * 
     * // Update the poses for the swerveModules. Note that the order of rotating
     * the
     * // position and then adding the translation matters
     * for (int i = 0; i < MODULES.length; i++) {
     * Translation2d modulePositionFromChassis =
     * CONSTANTS[i].Location.rotateBy(getHeading()).plus(pose.getTranslation());
     * 
     * // Module's heading is it's angle relative to the chassis heading
     * modulePoses[i] =
     * new Pose2d(modulePositionFromChassis,
     * MODULES[i].getState().angle.plus(pose.getRotation()));
     * }
     * 
     * // plot it on the simulated field
     * field.setRobotPose(pose);
     * if (Robot.isSimulation()) {
     * updateSimulationVision(pose);
     * }
     * field.getObject("Swerve Modules").setPoses(modulePoses);
     * }
     */

    /*
     * public Pose2d getPose() {
     * return odometry.getPoseMeters();
     * }
     */

    public void updateSimulationVision(Pose2d robot) {
        Translation2d[] balls;
        final double MAX_SIGHT_DIST = 1.219; // 48"

        Translation2d cameraMove = new Translation2d(0.381, new Rotation2d());// move the camera 15" forward to be at
        // the front of the robot
        Pose2d cameraPose = robot.transformBy(new Transform2d(cameraMove, new Rotation2d()));

        if (DriverStation.getAlliance() == Alliance.Red) {
            balls = redBalls;
        } else {
            balls = blueBalls;
        }

        ArrayList<Double> centerX = new ArrayList<>();
        ArrayList<Double> centerY = new ArrayList<>();
        for (Translation2d ball : balls) {
            Transform2d heading = calcHeading(cameraPose, ball);

            // ball must be within 48" and within a 90* FOV to be seen
            double angle = heading.getRotation().getDegrees();
            if (Math.abs(angle) < 45.0) {
                double dist = heading.getTranslation().getNorm();
                double x = Math.sin(Math.toRadians(angle)) * dist;
                double y = Math.cos(Math.toRadians(angle)) * dist;

                // 48" check
                if (y < MAX_SIGHT_DIST) {
                    // top left is 0,0
                    centerX.add((MAX_SIGHT_DIST - x) / MAX_SIGHT_DIST * 320); // since we are 90*, a 45* max triangle
                    // has equal sides, so we assumed max
                    // distance side to side also. 640 max
                    // pixels divided by 2
                    centerY.add((MAX_SIGHT_DIST - y) / MAX_SIGHT_DIST * 480);
                }
            }
        }

        visionTable.getEntry("cargoX").setDoubleArray(centerX.stream().mapToDouble(d -> d).toArray());
        visionTable.getEntry("cargoY").setDoubleArray(centerY.stream().mapToDouble(d -> d).toArray());
    }

    public static Transform2d calcHeading(Pose2d robot, Translation2d target) {
        Translation2d trans = target.minus(robot.getTranslation());
        double x = trans.getX();
        double y = trans.getY();
        double h = trans.getNorm();

        double angle;
        if (Math.abs(y) < 1e-9 && x < 0.0) {
            // handle 180* case
            angle = Math.PI;
        } else if (x >= 0.0) {
            // handle quadrants 1 and 4
            angle = Math.asin(y / h);
        } else if (y >= 0.0) {
            // handle quadrant 2
            angle = Math.acos(x / h);
        } else {
            // handle quadrant 3
            angle = Math.asin(-y / h) + Math.PI;
        }
        return new Transform2d(trans, (new Rotation2d(angle)).minus(robot.getRotation()));
    }

    public SwerveModule[] getModules() {
        return MODULES;
    }

    @Override
    public void periodic() {
        // put data on dashboard
        loops++;
        if (loops % 5 == 0) {
            for (SwerveModule module : MODULES) {
                SmartDashboard.putNumber("SwerveDrive/gyroAngle", getAngle());
                SmartDashboard.putNumber("SwerveDrive/gyroHeading", getHeading().getDegrees());
                module.putSmartDashboard();
            }
            loops = 0;
        }
    }

    @Override
    public void simulationPeriodic() {
        boolean reset = SmartDashboard.getBoolean("Reset Position", false);
        if (reset) {
            // set the robot to x=0.5m, y=4m, rot=0*
            // odometry.resetPosition(new Pose2d(6.5, 4.72, new Rotation2d()), new
            // Rotation2d());
            SmartDashboard.putBoolean("Reset Position", false);
        }

        double rate = Robot.kDefaultPeriod;
        SwerveModuleState[] states = new SwerveModuleState[4];

        // run the simulation to get the module's velocity/angle
        for (int i = 0; i < MODULES.length; i++) {
            MODULES[i].simulationPeriodic(rate);
            states[i] = MODULES[i].getState();
        }

        // calculate the robot's speed and angle (we only care about angle here)
        //double omega = kinematics.toChassisSpeeds(states).omegaRadiansPerSecond;
        // set the IMU to the calculated robot rotation
        //double angle = Math.toDegrees(omega * rate);
        // gyroSim.setAngle(odometry.getPoseMeters().getRotation().getDegrees() +
        // angle);
    }

    public double deadbandStick(double value) {
        final double deadband = 0.12;
        double absVal = Math.abs(value);

        if (absVal > deadband) {
            return Math.signum(value) * (deadband + ((0.52) * absVal * absVal)); // TODO: change to 1 - x(deadband)
        }
        return 0.0;
    }

    public void runTurtleMode(XboxController controller) {
        driverController = controller;
        /*
         * if(driverController.getRightTriggerAxis() >= TRIGGER_SENSITIVITY){
         * driveStickSlew.setLimiters(1.5, 1.5, 1.5);
         * }
         * else{
         * driveStickSlew.setLimiters(3.0, 3.0, 3.0);
         * }
         */
        if (driverController.getRightTriggerAxis() >= 0.5) {
            kMaxSpeed = 1.4;
            kMaxAngularSpeed = 2.5;
        } else {
            kMaxSpeed = 3.0;
            kMaxAngularSpeed = 2 * Math.PI;
        }
    }

    /*
     * public void setPosition(double xDesPosition, double yDesPosition, double
     * desRotation, double time, int step) {
     * Pose2d pos = odometry.getPoseMeters();
     * double xCurrentPos = pos.getX();
     * double yCurrentPos = pos.getY();
     * double rotCurrentPos = pos.getRotation().getRadians();
     * double xMove = xDesPosition - xCurrentPos;
     * double yMove = yDesPosition - yCurrentPos;
     * // rotCurrentPos = rotCurrentPos % Math.toRadians(360);
     * if (rotCurrentPos < 0.0) {
     * rotCurrentPos += Math.toRadians(360.0);
     * }
     * double rotMag = desRotation - rotCurrentPos;
     * 
     * double xSpeed = xMove / time;
     * double ySpeed = yMove / time;
     * double rotSpeed = rotMag / time;
     * 
     * if (step == currentStep) {
     * if (Math.abs(xCurrentPos - xDesPosition) > 0.1 || Math.abs(yCurrentPos -
     * yDesPosition) > 0.1 ||
     * Math.abs(rotCurrentPos - desRotation) > 0.1) {
     * swerveDrive(xSpeed, ySpeed, rotSpeed, true);
     * } else {
     * xSpeed = 0.0;
     * ySpeed = 0.0;
     * rotSpeed = 0.0;
     * swerveDrive(xSpeed, ySpeed, rotSpeed, false);
     * System.out.println("Arrived");
     * currentStep++;
     * }
     * }
     * 
     * }
     */

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    @SuppressWarnings("ParameterName")
    public void swerveDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // ask the kinematics to determine our swerve command
        ChassisSpeeds speeds;
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading());
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        }
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(speeds);

        // sometime the Kinematics spits out too fast of speeds, so this will fix this
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

        // command each swerve module
        for (int i = 0; i < MODULES.length; i++) {
            MODULES[i].setDesiredState(swerveModuleStates[i]);
        }

        // report our commands to the dashboard
        SmartDashboard.putNumber("SwerveDrive/xSpeed", xSpeed);
        SmartDashboard.putNumber("SwerveDrive/ySpeed", ySpeed);
        SmartDashboard.putNumber("SwerveDrive/rot", rot);
        SmartDashboard.putBoolean("SwerveDrive/fieldRelative", fieldRelative);
    }

    public void setBrakeMode(boolean brake) {
        for (SwerveModule module : MODULES) {
            module.setBrakeMode(brake);
        }
    }

    public int getCurrentStep() {
        return currentStep;
    }

    public void incrementCurrentStep() {
        currentStep++;
    }

    public void resetCurrentStep() {
        currentStep = 0;
    }
}